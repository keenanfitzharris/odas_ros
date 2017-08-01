#include "ros/ros.h"
#include "odas_ros/spectra.h"
#include "../ros_parameters_parser.h"

extern "C" {
	#include "odas/odas.h"
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "src_spectra_file");

    ROS_INFO("Initializing source spectra file........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get filame from arguments

    int error_count = 0;

    char* filename = check_filename_arg(argc, argv, "filename", true, error_count);


    // Get parameters from server

    ROS_INFO("Retrieving parameters........");

    int fS = retrieve_int(node_handle, "config/fS", error_count);
    int halfFrameSize = retrieve_int(node_handle, "config/frameSize", error_count) / 2;
    int nChannels = retrieve_int(node_handle, "config/nChannels", error_count);

    std::string out_topic = retrieve_string(node_handle, "outTopic", error_count);

    if(error_count > 0) {
        ROS_ERROR("Failed to retrieve parameters");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_spectra_cfg* spectra_cfg = msg_spectra_cfg_construct();

    spectra_cfg->fS = fS;
    spectra_cfg->halfFrameSize = fS;
    spectra_cfg->nChannels = nChannels;

    msg_spectra_obj* spectra_msg = msg_spectra_construct(spectra_cfg);


    src_spectra_obj*


    // Wait for subscribers

    ROS_INFO("Waiting for subscribers........");

    ros::Publisher ros_publisher = public_handle.advertise<odas_ros::hop>(outTopic, 2000);

    while(ros_publisher.getNumSubscribers() < 1 && ros::ok()) {

        sleep(1);
    }


    // Proccess signal

    ROS_INFO("Processing signal........");

    int return_value = 0;

    ros::Rate playing_rate(fS/hopSize);

    while(return_value == 0 && ros::ok()) {

        return_value = src_hops_process(src_obj);
        odas_ros::hop ros_msg_out;

        if(return_value != 0) {

            ros_msg_out.fS = 0;
            ros_publisher.publish(ros_msg_out);

            ROS_INFO("Signal ended........");
            break;
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
        playing_rate.sleep();
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
