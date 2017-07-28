#include "ros/ros.h"
#include "odas_ros/pot.h"
#include "odas_ros/track.h"

#include <math.h>

extern "C" {
    #include "odas/odas.h"
}


void process_pot_msg(const odas_ros::pot::ConstPtr& in_ros_msg, msg_pots_obj* in_msg, msg_tracks_obj* out_msg, mod_sst_obj* mod_sst, ros::Publisher &ros_publisher)
{
    odas_ros::track out_ros_msg;

    ros_publisher.publish(out_ros_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mod_sst");

    ROS_INFO("Initializing sst module........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get parameters from server

    ROS_INFO("Retreiving parameters........");

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

    if(!node_handle.getParam("ssl/nPots", nPots)) {

        ROS_ERROR("Couln'd retrieve ssl/nPots");
        return -1;
    }


    int nMatches;

    if(!node_handle.getParam("ssl/nMatches", nMatches)) {

        ROS_ERROR("Couln'd retrieve ssl/nMatches");
        return -1;
    }


    float probMin;

    if(!node_handle.getParam("ssl/probMin", probMin)) {

        ROS_ERROR("Couln'd retrieve ssl/probMin");
        return -1;
    }


    int nRefinedLevels;

    if(!node_handle.getParam("ssl/nRefinedLevels", nRefinedLevels)) {

        ROS_ERROR("Couln'd retrieve ssl/nRefinedLevels");
        return -1;
    }


    float gainMin;

    if(!node_handle.getParam("ssl/gainMin", gainMin)) {

        ROS_ERROR("Couln'd retrieve ssl/gainMin");
        return -1;
    }


    int nThetas;

    if(!node_handle.getParam("ssl/nThetas", nThetas)) {

        ROS_ERROR("Couln'd retrieve ssl/nThetas");
        return -1;
    }


    XmlRpc::XmlRpcValue levels_param, levels_buffer;
    int nLevels, levels_int, deltas_int;
    std::vector<int> levels_vector, deltas_vector;

    if(!node_handle.getParam("ssl/scans", levels_param)) {

        ROS_ERROR("Couln'd retrieve ssl/scans");
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

        ROS_ERROR("Error while parsing ssl/scans");
        return -1;
    }


    std::string mode_string;
    char mode_char;

    if(node_handle.getParam("sst/mode",mode_string)) {

        if(mode_string.compare("kalman") == 0)
            mode_char = 'k';

        else if(mode_string.compare("particle") == 0)
            mode_char = 'p';

        else {

            ROS_ERROR("Error while parsing sst/mode");
            return -1;
        }
    }

    else {

        ROS_ERROR("Couln'd retrieve sst/mode");
        return -1;
    }


    std::vector<int> N_inactive_vector;
    int nTracksMax;

    if(node_handle.getParam("sst/N_inactive",N_inactive_vector))
        nTracksMax = N_inactive_vector.size();

    else {

        ROS_ERROR("Couln'd retrieve sst/N_inactive");
        return -1;
    }


    std::map<std::string, float> filter_parameters;
    int parameters_asserted = 0;

    if(node_handle.getParam("sst/"+mode_string, filter_parameters)) {

        switch (mode_char) {

        case 'k':

            parameters_asserted += filter_parameters.count("sigmaQ");

            if(parameters_asserted < 1) {
                ROS_ERROR("Error while parsing sst/%s", mode_string.c_str());
                return -1;
            }
            break;

        case 'p':

            parameters_asserted += filter_parameters.count("nParticles");
            parameters_asserted += filter_parameters.count("st_alpha");
            parameters_asserted += filter_parameters.count("st_beta");
            parameters_asserted += filter_parameters.count("st_ratio");
            parameters_asserted += filter_parameters.count("ve_alpha");
            parameters_asserted += filter_parameters.count("ve_beta");
            parameters_asserted += filter_parameters.count("ve_ratio");
            parameters_asserted += filter_parameters.count("ac_alpha");
            parameters_asserted += filter_parameters.count("ac_beta");
            parameters_asserted += filter_parameters.count("ac_ratio");
            parameters_asserted += filter_parameters.count("Nmin");

            if(parameters_asserted < 11) {
                ROS_ERROR("Error while parsing sst/%s", mode_string.c_str());
                return -1;
            }
            break;

        default:
            ROS_ERROR("Error while parsing sst/%s", mode_string.c_str());
            return -1;
            break;
        }
    }

    else {

        ROS_ERROR("Couln'd retrieve sst/%s", mode_string.c_str());
        return -1;
    }


    float sigmaR2_active, sigmaR_active;

    if(node_handle.getParam("sst/sigmaR2_active",sigmaR2_active))
        sigmaR_active = sqrtf(sigmaR2_active);

    else {

        ROS_ERROR("Couln'd retrieve sst/sigmaR2_active");
        return -1;
    }


    float sigmaR2_prob, sigmaR_prob;

    if(node_handle.getParam("sst/sigmaR2_prob",sigmaR2_prob))
        sigmaR_prob = sqrtf(sigmaR2_prob);

    else {

        ROS_ERROR("Couln'd retrieve sst/sigmaR2_prob");
        return -1;
    }


    XmlRpc::XmlRpcValue active_vector, current_active;
    int nActiveGaussians;

    if(node_handle.getParam("sst/active",active_vector))
        nActiveGaussians = active_vector.size();

    else {

        ROS_ERROR("Couln'd retrieve sst/active");
        return -1;
    }

    gaussians_1d_obj* active_gaussians = gaussians_1d_construct_null(nActiveGaussians);
    float act_weight, act_mu, act_sigma;

    try {

        for(int iGaussians = 0; iGaussians < nActiveGaussians; iGaussians++) {

            current_active = active_vector[iGaussians];

            act_weight = static_cast<float>(current_active["weight"]);
            act_mu = static_cast<float>(current_active["mu"]);
            act_sigma = sqrtf(static_cast<float>(current_active["sigma2"]));

            active_gaussians->array[iGaussians] = gaussian_1d_construct_weightmusigma(act_weight, act_mu, act_sigma);
        }

    } catch(...) {

        ROS_ERROR("Error while parsing sst/active");
        return -1;
    }


    XmlRpc::XmlRpcValue inactive_vector, current_inactive;
    int nInactiveGaussians;

    if(node_handle.getParam("sst/inactive",inactive_vector))
        nInactiveGaussians = inactive_vector.size();

    else {

        ROS_ERROR("Couln'd retrieve sst/inactive");
        return -1;
    }

    gaussians_1d_obj* inactive_gaussians = gaussians_1d_construct_null(nInactiveGaussians);
    float inact_weight, inact_mu, inact_sigma;

    try {

        for(int iGaussians = 0; iGaussians < nInactiveGaussians; iGaussians++) {

            current_inactive = inactive_vector[iGaussians];

            inact_weight = static_cast<float>(current_inactive["weight"]);
            inact_mu = static_cast<float>(current_inactive["mu"]);
            inact_sigma = sqrtf(static_cast<float>(current_inactive["sigma2"]));

            inactive_gaussians->array[iGaussians] = gaussian_1d_construct_weightmusigma(inact_weight, inact_mu, inact_sigma);
        }

    } catch(...) {

        ROS_ERROR("Error while parsing sst/inactive");
        return -1;
    }


    float Pfalse;

    if(!node_handle.getParam("sst/Pfalse", Pfalse)) {

        ROS_ERROR("Couln'd retrieve sst/Pfalse");
        return -1;
    }


    float Pnew;

    if(!node_handle.getParam("sst/Pnew", Pnew)) {

        ROS_ERROR("Couln'd retrieve sst/Pnew");
        return -1;
    }


    float Ptrack;

    if(!node_handle.getParam("sst/Ptrack", Ptrack)) {

        ROS_ERROR("Couln'd retrieve sst/Ptrack");
        return -1;
    }


    float theta_new;

    if(!node_handle.getParam("sst/theta_new", theta_new)) {

        ROS_ERROR("Couln'd retrieve sst/theta_new");
        return -1;
    }


    float theta_prob;

    if(!node_handle.getParam("sst/theta_prob", theta_prob)) {

        ROS_ERROR("Couln'd retrieve sst/theta_prob");
        return -1;
    }


    float N_prob;

    if(!node_handle.getParam("sst/N_prob", N_prob)) {

        ROS_ERROR("Couln'd retrieve sst/N_prob");
        return -1;
    }


    float theta_inactive;

    if(!node_handle.getParam("sst/theta_inactive", theta_inactive)) {

        ROS_ERROR("Couln'd retrieve sst/theta_inactive");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");


    msg_pots_cfg* in_msg_cfg = msg_pots_cfg_construct();

    out_msg_cfg->fS = sample_rate->mu;
    out_msg_cfg->nPots = nPots;

    msg_pots_obj* in_msg = msg_pots_construct(in_msg_cfg);


    mod_ssl_cfg* ssl_cfg = mod_ssl_cfg_construct();

    ssl_cfg->mics = mics;
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


    mod_sst_cfg* sst_cfg = mod_sst_cfg_construct();




    // Wait for subscribers

    ROS_INFO("Waiting for subscribers........");

    ros::Publisher ros_publisher = public_handle.advertise<odas_ros::track>(out_topic, 2000);

    while(ros_publisher.getNumSubscribers() < 1 && ros::ok()) {

        sleep(1);
    }


    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::pot>(in_topic, 2000, boost::bind(process_pot_msg, _1, in_msg, out_msg, mod_sst, ros_publisher));
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");


    mod_ssl_cfg_destroy(ssl_cfg);


    msg_pots_destroy(in_msg);
    msg_pots_cfg_destroy(in_msg_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
