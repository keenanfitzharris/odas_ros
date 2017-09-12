#include "ros/ros.h"
#include "odas_ros/hop.h"

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

extern "C" {
    #include "odas/odas.h"
}

/*
 * Execute a system command and return the text output
 */
std::string exec(const char* cmd) {

    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);

    if (!pipe)
        throw std::runtime_error("popen() failed!");

    while (!feof(pipe.get())) {

        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }

    return result;
}

/*
 * Find hardware number by name
 */
bool find_soundcard(char* card_name, int &hw_number, int &device_number) {

    std::string arecord_output = exec("arecord -l");

    std::vector<std::string> arecord_output_vector;
    boost::split(arecord_output_vector, arecord_output, boost::is_any_of("\n"), boost::token_compress_on);

    int nLines = arecord_output_vector.size();

    std::string current_line, str_buffer;
    std::size_t character_position, number_str_len;

    for(int iLines = 0; iLines < nLines; iLines++) {

        current_line = arecord_output_vector.at(iLines);

        if(current_line.find(card_name) != std::string::npos) {

            character_position =  current_line.find("card") + 5;
            number_str_len = current_line.find(":",character_position) - character_position;

            str_buffer = current_line.substr(character_position, number_str_len);
            hw_number = atoi(str_buffer.c_str());

            character_position = current_line.find("device") + 7;
            number_str_len = current_line.find(":",character_position) - character_position;

            str_buffer = current_line.substr(character_position, number_str_len);
            device_number = atoi(str_buffer.c_str());

            return true;
        }
    }

    return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "src_soundcard");

    ROS_INFO("Initializing source soundcard........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get card name from arguments

    ROS_INFO("Reading card name........");

    if(argc < 2) {

        ROS_ERROR("Please provide source card name");
        return -1;
    }

    else if(argc > 2) {

        ROS_ERROR("Too many arguments, expecting only source card name");
        return -1;
    }

    char* card_name = argv[1];


    // Retrieve hardware and device number from card name

    int hw_number, device_number;
    bool card_found = find_soundcard(card_name, hw_number, device_number);

    if(card_found) {

        ROS_INFO("Found soundcard at hw:%d,%d", hw_number, device_number);

//TODO
//DL - What is the use of this???
/*
        std::stringstream record_string;
        record_string << "arecord -d 1  hw:" << hw_number << "," << device_number;
        exec(record_string.str().c_str());
	ROS_INFO("Waiting for soundcard........");
	sleep(5);
	ROS_INFO("Recording finished........");
*/
    }

    else {

        ROS_ERROR("Couln'd find specified soundcard");
        return -1;
    }

    // Get parameters from server

    ROS_INFO("Retreiving parameters........");

    int fS, hopSize, nChannels, nBits;
    std::string outTopic;

    if(!node_handle.getParam("config/fS", fS)) {

        ROS_ERROR("Couln'd retrieve config/fS");
        return -1;
    }

    if(!node_handle.getParam("config/hopSize", hopSize)) {

        ROS_ERROR("Couln'd retrieve config/hopSize");
        return -1;
    }

    if(!node_handle.getParam("config/nChannels", nChannels)) {

        ROS_ERROR("Couln'd retrieve config/nChannels");
        return -1;
    }

    if(!node_handle.getParam("config/nBits", nBits)) {

        ROS_ERROR("Couln'd retrieve config/nBits");
        return -1;
    }

    if(!node_handle.getParam("outTopic", outTopic)) {

        ROS_ERROR("Couln'd retrieve outTopic");
        return -1;
    }
    
    ROS_INFO("config/fS : %i",fS);
    ROS_INFO("config/hopSize : %i",hopSize);
    ROS_INFO("config/nChannels : %i",nChannels);
    ROS_INFO("config/nBits : %i",nBits);
    ROS_INFO("config/outTopic : %s",outTopic.c_str());

    ros::Publisher ros_publisher = public_handle.advertise<odas_ros::hop>(outTopic, 2000);


    // Contruct objects

    ROS_INFO("Constructing objects........");


    msg_hops_cfg* msg_cfg = msg_hops_cfg_construct();

    msg_cfg->fS = fS;
    msg_cfg->hopSize = hopSize;
    msg_cfg->nChannels = nChannels;

    msg_hops_obj* msg_out = msg_hops_construct(msg_cfg);


    src_hops_cfg* hops_cfg = src_hops_cfg_construct();

    hops_cfg->format = format_construct_bin(nBits);
    hops_cfg->interface = interface_construct_soundcard(hw_number,device_number);
    hops_cfg->fS = fS;

    src_hops_obj* src_obj = src_hops_construct(hops_cfg, msg_cfg);


    src_hops_connect(src_obj, msg_out);
    src_hops_open(src_obj);


    // Wait for subscribers

    ROS_INFO("Waiting for subscribers........");

    while(ros_publisher.getNumSubscribers() < 1 && ros::ok()) {

        sleep(1);
    }


    // Proccess signal

    ROS_INFO("Processing signal........");

    int return_value = 0;
    bool restored_card;

    while(return_value == 0 && ros::ok()) {

        return_value = src_hops_process(src_obj);
        odas_ros::hop ros_msg_out;

        if(return_value != 0) {

            ROS_ERROR("Lost soundcard!");

            src_hops_close(src_obj);
            src_hops_disconnect(src_obj);
            src_hops_destroy(src_obj);

            while(ros::ok()) {

                ROS_WARN("Retrying........");
                sleep(1);

                if(find_soundcard(card_name, hw_number, device_number)) {

                    hops_cfg->interface = interface_construct_soundcard(hw_number, device_number);
                    src_obj = src_hops_construct(hops_cfg, msg_cfg);

                    src_hops_connect(src_obj, msg_out);
                    src_hops_open(src_obj);

                    return_value = src_hops_process(src_obj);

                    if(return_value == 0) {

                        restored_card = true;
                    }

                    else {

                        restored_card = false;
                    }
                }

                else {

                    restored_card = false;
                }

                if(restored_card) {

                    ROS_INFO("Retrieved soundcard");
                    ROS_INFO("Processing signal........");
                    break;
                }

                else {

                    ROS_WARN("Failed to retrieve soundcard");
                }
            }
        }

        ros_msg_out.timeStamp = msg_out->timeStamp;
        ros_msg_out.nSignals = msg_out->hops->nSignals;
        ros_msg_out.hopSize = msg_out->hops->hopSize;
        ros_msg_out.fS = msg_out->fS;

        ros_msg_out.data = std::vector<float>(ros_msg_out.nSignals * ros_msg_out.hopSize);

        for (int iSample = 0; iSample < ros_msg_out.hopSize; iSample++) {

            for (int iChannel = 0; iChannel < ros_msg_out.nSignals; iChannel++) {

                ros_msg_out.data[iSample + iChannel*ros_msg_out.hopSize] = msg_out->hops->array[iChannel][iSample];
            }
        }

        ros_publisher.publish(ros_msg_out);
    }

    src_hops_disconnect(src_obj);
    src_hops_close(src_obj);


    // Destroy objects

    ROS_INFO("Destroying objects........");

    src_hops_destroy(src_obj);
    msg_hops_destroy(msg_out);

    msg_hops_cfg_destroy(msg_cfg);
    src_hops_cfg_destroy(hops_cfg);

    ROS_INFO("Quitting node........");

    return 0;
}
