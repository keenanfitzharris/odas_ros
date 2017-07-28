#include "ros/ros.h"
#include "odas_ros/hop.h"
#include "odas_ros/spectra.h"

extern "C" {
    #include "odas/odas.h"
}


void process_hops_msg(const odas_ros::hop::ConstPtr& in_ros_msg, msg_hops_obj* in_msg, msg_spectra_obj* out_msg, mod_stft_obj* mod_stft, ros::Publisher &ros_publisher)
{
    odas_ros::spectra out_ros_msg;

    in_msg->fS = in_ros_msg->fS;
    in_msg->timeStamp = in_ros_msg->timeStamp;
    in_msg->hops->hopSize = in_ros_msg->hopSize;
    in_msg->hops->nSignals = in_ros_msg->nSignals;

    if(in_msg->fS == 0) {

        ROS_INFO("Signal ended........");

        out_ros_msg.fS = 0;
        ros_publisher.publish(out_ros_msg);

        ros::shutdown();
        return;
    }

    for (int iSample = 0; iSample < in_msg->hops->hopSize; iSample++) {

        for (int iChannel = 0; iChannel < in_msg->hops->nSignals; iChannel++) {

            in_msg->hops->array[iChannel][iSample] = in_ros_msg->data[iSample + iChannel*in_msg->hops->hopSize];
        }
    }


    mod_stft_process(mod_stft);


    out_ros_msg.timeStamp = out_msg->timeStamp;
    out_ros_msg.fS = out_msg->fS;
    /*
     * TODO PATCH
     * Patching from msg_spectra fS config not used!!
     * TODO PATCH
     * */
    out_ros_msg.fS = in_msg->fS;
    /*
     * TODO PATCH
     * */
    out_ros_msg.nSignals = out_msg->freqs->nSignals;
    out_ros_msg.halfFrameSize = out_msg->freqs->halfFrameSize;

    int nSamples = out_ros_msg.halfFrameSize * 2;
    int nSignals = out_ros_msg.nSignals;

    out_ros_msg.data = std::vector<float>(nSamples * nSignals);

    for(int iSamples = 0; iSamples < nSamples; iSamples++) {

        for(int iSignals = 0; iSignals < nSignals; iSignals++) {

            out_ros_msg.data[iSamples + iSignals*nSamples] = out_msg->freqs->array[iSignals][iSamples];
        }
    }


    ros_publisher.publish(out_ros_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mod_stft");

    ROS_INFO("Initializing stft module........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get parameters from server

    ROS_INFO("Retreiving parameters........");

    int nChannels, fS, in_hopSize, out_frameSize;
    std::string in_topic, out_topic;

    if(!node_handle.getParam("config/fS", fS)) {

        ROS_ERROR("Couln'd retrieve config/fS");
        return -1;
    }

    if(!node_handle.getParam("config/nChannels", nChannels)) {

        ROS_ERROR("Couln'd retrieve config/nChannels");
        return -1;
    }

    if(!node_handle.getParam("config/in/hopSize", in_hopSize)) {

        ROS_ERROR("Couln'd retrieve config/in/hopSize");
        return -1;
    }

    if(!node_handle.getParam("config/out/frameSize", out_frameSize)) {

        ROS_ERROR("Couln'd retrieve config/out/frameSize");
        return -1;
    }

    if(!node_handle.getParam("inTopic", in_topic)) {

        ROS_ERROR("Couln'd retrieve inTopic");
        return -1;
    }

    if(!node_handle.getParam("outTopic", out_topic)) {

        ROS_ERROR("Couln'd retrieve outTopic");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_hops_cfg* in_msg_cfg = msg_hops_cfg_construct();

    in_msg_cfg->fS = fS;
    in_msg_cfg->hopSize = in_hopSize;
    in_msg_cfg->nChannels = nChannels;

    msg_hops_obj* in_msg = msg_hops_construct(in_msg_cfg);


    msg_spectra_cfg* out_msg_cfg = msg_spectra_cfg_construct();

    out_msg_cfg->fS = fS;
    out_msg_cfg->halfFrameSize = out_frameSize/2;
    out_msg_cfg->nChannels = nChannels;

    msg_spectra_obj* out_msg = msg_spectra_construct(out_msg_cfg);


    mod_stft_cfg* stft_cfg = mod_stft_cfg_construct();

    mod_stft_obj* mod_stft = mod_stft_construct(stft_cfg, in_msg_cfg, out_msg_cfg);

    mod_stft_connect(mod_stft, in_msg, out_msg);


    // Wait for subscribers

    ROS_INFO("Waiting for subscribers........");

    ros::Publisher ros_publisher = public_handle.advertise<odas_ros::spectra>(out_topic, 2000);

    while(ros_publisher.getNumSubscribers() < 1 && ros::ok()) {

        sleep(1);
    }


    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::hop>(in_topic, 2000, boost::bind(process_hops_msg, _1, in_msg, out_msg, mod_stft, ros_publisher));
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");

    mod_stft_disconnect(mod_stft);

    mod_stft_destroy(mod_stft);
    mod_stft_cfg_destroy(stft_cfg);

    msg_hops_destroy(in_msg);
    msg_hops_cfg_destroy(in_msg_cfg);

    msg_spectra_destroy(out_msg);
    msg_spectra_cfg_destroy(out_msg_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
