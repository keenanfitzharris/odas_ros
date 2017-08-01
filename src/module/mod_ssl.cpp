#include "ros/ros.h"
#include "odas_ros/spectra.h"
#include "odas_ros/pot.h"
#include "../ros_parameters_parser.h"

extern "C" {
    #include "odas/odas.h"
}


void process_spectra_msg(const odas_ros::spectra::ConstPtr& in_ros_msg, msg_spectra_obj* in_msg, msg_pots_obj* out_msg, mod_ssl_obj* mod_ssl, ros::Publisher &ros_publisher)
{
    odas_ros::pot out_ros_msg;

    in_msg->fS = in_ros_msg->fS;
    in_msg->timeStamp = in_ros_msg->timeStamp;
    in_msg->freqs->nSignals = in_ros_msg->nSignals;
    in_msg->freqs->halfFrameSize = in_ros_msg->halfFrameSize;

    if(in_msg->fS == 0) {

        ROS_INFO("Signal ended........");

        out_ros_msg.fS = 0;
        ros_publisher.publish(out_ros_msg);

        ros::shutdown();
        return;
    }

    int nSamples = in_msg->freqs->halfFrameSize * 2;
    int nSignals = in_msg->freqs->nSignals;

    for(int iSignals = 0; iSignals < nSignals; iSignals++) {

        for(int iSamples = 0; iSamples < nSamples; iSamples++) {

            in_msg->freqs->array[iSignals][iSamples] = in_ros_msg->data[iSamples + iSignals*nSamples];
        }
    }


    mod_ssl_process(mod_ssl);


    out_ros_msg.timeStamp = out_msg->timeStamp;
    out_ros_msg.fS = out_msg->fS;
    out_ros_msg.nPots = out_msg->pots->nPots;

    int nPots = out_ros_msg.nPots;

    out_ros_msg.data = std::vector<float>(nPots * 4);

    for(int iPots = 0; iPots < nPots * 4; iPots++) {

        out_ros_msg.data[iPots] = out_msg->pots->array[iPots];
    }


    ros_publisher.publish(out_ros_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mod_ssl");

    ROS_INFO("Initializing ssl module........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get parameters from server

    ROS_INFO("Retreiving parameters........");

    int error_count = 0;

    XmlRpc::XmlRpcValue mics_param, mic_buffer;
    mics_obj* mics;
    int nChannels;

    if(!node_handle.getParam("general/mics", mics_param)) {

        ROS_ERROR("Couln'd retrieve general/mics");
        return -1;
    }

    nChannels = mics_param.size();
    mics = mics_construct_zero(nChannels);

    try {

        for(int iChannels = 0; iChannels < nChannels; iChannels++) {

            mic_buffer = mics_param[iChannels];

            for(int iMu = 0; iMu < 3; iMu++) {

                mics->mu[iMu + 3*iChannels] = (float)static_cast<double>(mic_buffer["mu"][iMu]);
            }

            for(int iSigma2 = 0; iSigma2 < 9; iSigma2++) {

                mics->sigma2[iSigma2 + 9*iChannels] = (float)static_cast<double>(mic_buffer["sigma2"][iSigma2]);
            }

            for(int iDirection = 0; iDirection < 3; iDirection++) {

                mics->direction[iDirection + 3*iChannels] = (float)static_cast<double>(mic_buffer["direction"][iDirection]);
            }

            mics->thetaAllPass[iChannels] = (float)static_cast<double>(mic_buffer["angle"][0]);
            mics->thetaNoPass[iChannels] = (float)static_cast<double>(mic_buffer["angle"][1]);
        }

    } catch(...) {

        ROS_ERROR("Error while parsing general/mics");
        return -1;
    }


    std::string epsilon_str;
    float epsilon;

    if(!node_handle.getParam("general/epsilon", epsilon_str)) {

        ROS_ERROR("Couln'd retrieve general/epsilon");
        return -1;
    }

    try {

        epsilon = atof(epsilon_str.c_str());

    } catch(...) {

        ROS_ERROR("Error while parsing general/epsilon");
        return -1;
    }


    int frameSize;

    if(!node_handle.getParam("general/frameSize", frameSize)) {

        ROS_ERROR("Couln'd retrieve general/frameSize");
        return -1;
    }


    samplerate_obj* sample_rate = samplerate_construct_zero();
    int sample_rate_mu;
    float sample_rate_sigma2;

    if(!node_handle.getParam("general/samplerate/mu", sample_rate_mu)) {

        ROS_ERROR("Couln'd retrieve general/samplerate/mu");
        return -1;
    }
    sample_rate->mu = sample_rate_mu;

    if(!node_handle.getParam("general/samplerate/sigma2", sample_rate_sigma2)) {

        ROS_ERROR("Couln'd retrieve general/samplerate/sigma2");
        return -1;
    }
    sample_rate->sigma2 = sample_rate_sigma2;


    soundspeed_obj* sound_speed =  soundspeed_construct_zero();
    float sound_speed_mu, sound_speed_sigma2;

    if(!node_handle.getParam("general/speedofsound/mu", sound_speed_mu)) {

        ROS_ERROR("Couln'd retrieve general/speedofsound/mu");
        return -1;
    }
    sound_speed->mu = sound_speed_mu;


    if(!node_handle.getParam("general/speedofsound/sigma2", sound_speed_sigma2)) {

        ROS_ERROR("Couln'd retrieve general/speedofsound/sigma2");
        return -1;
    }
    sound_speed->sigma2 = sound_speed_sigma2;


    std::string in_topic, out_topic;

    if(!node_handle.getParam("inTopic", in_topic)) {

        ROS_ERROR("Couln'd retrieve inTopic");
        return -1;
    }

    if(!node_handle.getParam("outTopic", out_topic)) {

        ROS_ERROR("Couln'd retrieve outTopic");
        return -1;
    }


    int nPots;

    if(!node_handle.getParam("config/nPots", nPots)) {

        ROS_ERROR("Couln'd retrieve config/nPots");
        return -1;
    }


    int nMatches;

    if(!node_handle.getParam("config/nMatches", nMatches)) {

        ROS_ERROR("Couln'd retrieve config/nMatches");
        return -1;
    }


    float probMin;

    if(!node_handle.getParam("config/probMin", probMin)) {

        ROS_ERROR("Couln'd retrieve config/probMin");
        return -1;
    }


    int nRefinedLevels;

    if(!node_handle.getParam("config/nRefinedLevels", nRefinedLevels)) {

        ROS_ERROR("Couln'd retrieve config/nRefinedLevels");
        return -1;
    }


    float gainMin;

    if(!node_handle.getParam("config/gainMin", gainMin)) {

        ROS_ERROR("Couln'd retrieve config/gainMin");
        return -1;
    }


    int nThetas;

    if(!node_handle.getParam("config/nThetas", nThetas)) {

        ROS_ERROR("Couln'd retrieve config/nThetas");
        return -1;
    }


    XmlRpc::XmlRpcValue levels_param, levels_buffer;
    int nLevels, levels_int, deltas_int;
    std::vector<int> levels_vector, deltas_vector;

    if(!node_handle.getParam("config/scans", levels_param)) {

        ROS_ERROR("Couln'd retrieve config/scans");
        return -1;
    }
    nLevels = levels_param.size();

    try {

        for(int iLevels = 0; iLevels < nLevels; iLevels++) {

            levels_buffer = levels_param[iLevels];

            levels_int = static_cast<int>(levels_buffer["level"]);
            levels_vector.push_back(levels_int);

            deltas_int = static_cast<int>(levels_buffer["delta"]);
            deltas_vector.push_back(deltas_int);

        }

    } catch(...) {

        ROS_ERROR("Error while parsing config/scans");
        return -1;
    }


    spatialfilter_obj* spatialfilter = retrieve_spatialfilter(node_handle, "general/spatialfilter", error_count);

    if(error_count > 0) {

        ROS_ERROR("Failed to retrieve parameters");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_spectra_cfg* in_msg_cfg = msg_spectra_cfg_construct();

    in_msg_cfg->fS = sample_rate->mu;
    in_msg_cfg->halfFrameSize = frameSize / 2;
    in_msg_cfg->nChannels = nChannels;

    msg_spectra_obj* in_msg = msg_spectra_construct(in_msg_cfg);


    msg_pots_cfg* out_msg_cfg = msg_pots_cfg_construct();

    out_msg_cfg->fS = sample_rate->mu;
    out_msg_cfg->nPots = nPots;

    msg_pots_obj* out_msg = msg_pots_construct(out_msg_cfg);


    mod_ssl_cfg* ssl_cfg = mod_ssl_cfg_construct();

    ssl_cfg->mics = mics;
    ssl_cfg->spatialfilter = spatialfilter;
    ssl_cfg->samplerate = sample_rate;
    ssl_cfg->soundspeed = sound_speed;
    ssl_cfg->epsilon = epsilon;
    ssl_cfg->nLevels = nLevels;

    ssl_cfg->levels = (unsigned int *) malloc(sizeof(unsigned int) * nLevels);
    ssl_cfg->deltas = (signed int *)malloc(sizeof(signed int) * nLevels);

    for(int iLevels = 0; iLevels < nLevels; iLevels++) {

        ssl_cfg->levels[iLevels] = levels_vector.at(iLevels);
        ssl_cfg->deltas[iLevels] = deltas_vector.at(iLevels);
    }

    ssl_cfg->nMatches = nMatches;
    ssl_cfg->probMin = probMin;
    ssl_cfg->nRefinedLevels = nRefinedLevels;
    ssl_cfg->nThetas = nThetas;
    ssl_cfg->gainMin = gainMin;

    mod_ssl_obj* mod_ssl = mod_ssl_construct(ssl_cfg, in_msg_cfg, out_msg_cfg);

    mod_ssl_connect(mod_ssl, in_msg, out_msg);


    // Wait for subscribers

    ROS_INFO("Waiting for subscribers........");

    ros::Publisher ros_publisher = public_handle.advertise<odas_ros::pot>(out_topic, 2000);

    while(ros_publisher.getNumSubscribers() < 1 && ros::ok()) {

        sleep(1);
    }


    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::spectra>(in_topic, 2000, boost::bind(process_spectra_msg, _1, in_msg, out_msg, mod_ssl, ros_publisher));
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");

    mod_ssl_disconnect(mod_ssl);

    mod_ssl_destroy(mod_ssl);
    mod_ssl_cfg_destroy(ssl_cfg);

    msg_spectra_destroy(in_msg);
    msg_spectra_cfg_destroy(in_msg_cfg);

    msg_pots_destroy(out_msg);
    msg_pots_cfg_destroy(out_msg_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
