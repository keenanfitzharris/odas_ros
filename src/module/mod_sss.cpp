#include "ros/ros.h"
#include "odas_ros/spectra.h"
#include "odas_ros/track.h"

#include <math.h>
#include <deque>

extern "C" {
    #include "odas/odas.h"
}


class msg_synchroniser_obj
{
public:
    virtual ~msg_synchroniser_obj();

    void process_msg();
    void process_spectra_msg(const odas_ros::spectra::ConstPtr&);
    void process_tracks_msg(const odas_ros::track::ConstPtr&);

    msg_spectra_cfg* spectra_cfg;
    msg_spectra_obj* msg_in_spectra;

    msg_tracks_cfg* tracks_cfg;
    msg_tracks_obj* msg_in_tracks;

    msg_spectra_obj* msg_out;

    mod_sss_obj* mod_sss;
    ros::Publisher* ros_publisher;

private:
    std::deque<msg_spectra_obj*> spectra_vector;
    std::deque<msg_tracks_obj*> tracks_vector;
};

msg_synchroniser_obj::~msg_synchroniser_obj() {

        while(spectra_vector.size() > 0) {
            msg_spectra_destroy(*(spectra_vector.end()));
            spectra_vector.pop_back();
        }
    }

void msg_synchroniser_obj::process_msg()
{
    msg_spectra_obj* last_spectra;
    if(spectra_vector.size() > 0)
        last_spectra = spectra_vector.back();
    else
        return;

    msg_tracks_obj* last_track;
    if(tracks_vector.size() > 0)
        last_track = tracks_vector.back();
    else
        return;

    if(last_spectra->timeStamp == last_track->timeStamp) {

        msg_in_spectra = last_spectra;
        msg_in_tracks = last_track;

        mod_sss_connect(mod_sss, msg_in_spectra, msg_in_tracks, msg_out);
        mod_sss_process(mod_sss);
        mod_sss_disconnect(mod_sss);

        odas_ros::spectra msg_ros_out;

        msg_ros_out.timeStamp = msg_out->timeStamp;
        msg_ros_out.fS = msg_out->fS;
        msg_ros_out.nSignals = msg_out->freqs->nSignals;
        msg_ros_out.halfFrameSize = msg_out->freqs->halfFrameSize;

        int nSignals = msg_ros_out.nSignals;
        int nSamples = msg_ros_out.halfFrameSize * 2;

        msg_ros_out.data = std::vector<float>(nSignals * nSamples);

        for(int iSignals = 0; iSignals < nSignals; iSignals++)
            for(int iSamples = 0; iSamples < nSamples; iSamples++)
                msg_ros_out.data[iSamples + iSignals * nSamples] = msg_out->freqs->array[iSignals][iSamples];

        ros_publisher->publish(msg_ros_out);

        msg_spectra_destroy(last_spectra);
        spectra_vector.pop_back();

        msg_tracks_destroy(last_track);
        tracks_vector.pop_back();
    }

    else if(last_spectra->timeStamp < last_track->timeStamp) {

        while(last_spectra->timeStamp < last_track->timeStamp && spectra_vector.size() > 0) {

            msg_spectra_destroy(last_spectra);
            spectra_vector.pop_back();
            last_spectra = spectra_vector.back();
        }

        process_msg();
    }

    else if(last_track->timeStamp < last_spectra->timeStamp) {

        while(last_track->timeStamp < last_spectra->timeStamp && tracks_vector.size() > 0) {

            msg_tracks_destroy(last_track);
            tracks_vector.pop_back();
            last_track = tracks_vector.back();
        }

        process_msg();
    }
}

void msg_synchroniser_obj::process_spectra_msg(const odas_ros::spectra::ConstPtr &ros_msg_in_spectra)
{
    msg_spectra_obj* msg_in_spectra = msg_spectra_construct(spectra_cfg);

    msg_in_spectra->timeStamp = ros_msg_in_spectra->timeStamp;
    msg_in_spectra->fS = ros_msg_in_spectra->fS;
    msg_in_spectra->freqs->nSignals = ros_msg_in_spectra->nSignals;
    msg_in_spectra->freqs->halfFrameSize = ros_msg_in_spectra->halfFrameSize;

    if(msg_in_spectra->fS == 0) {

        odas_ros::spectra ros_msg;
        ros_msg.fS = 0;
        ros_publisher->publish(ros_msg);

        ros::shutdown();
        return;
    }

    int nSignals = msg_in_spectra->freqs->nSignals;
    int nSamples = msg_in_spectra->freqs->halfFrameSize * 2;

    for(int iSignals = 0; iSignals < nSignals; iSignals++)
        for(int iSamples = 0; iSamples < nSamples; iSamples++)
            msg_in_spectra->freqs->array[iSignals][iSamples] = ros_msg_in_spectra->data[iSamples + iSignals * nSamples];

    spectra_vector.push_front(msg_in_spectra);
    process_msg();
}


void msg_synchroniser_obj::process_tracks_msg(const odas_ros::track::ConstPtr &ros_msg_in_tracks)
{
    msg_tracks_obj* msg_in_tracks = msg_tracks_construct(tracks_cfg);

    msg_in_tracks->timeStamp = ros_msg_in_tracks->timeStamp;
    msg_in_tracks->fS = ros_msg_in_tracks->fS;
    msg_in_tracks->tracks->nTracks = ros_msg_in_tracks->nTracks;

    if(msg_in_tracks->fS == 0) {

        odas_ros::spectra ros_msg;
        ros_msg.fS = 0;
        ros_publisher->publish(ros_msg);

        ros::shutdown();
        return;
    }

    int nTracks = msg_in_tracks->tracks->nTracks;

    for(int iTracks = 0; iTracks < nTracks; iTracks++) {

        msg_in_tracks->tracks->ids[iTracks] = ros_msg_in_tracks->ids[iTracks];

        for(int iSamples = 0; iSamples < 3; iSamples++)
            msg_in_tracks->tracks->array[iSamples + iTracks * 3] = ros_msg_in_tracks->data[iSamples + iTracks * 3];
    }

    tracks_vector.push_front(msg_in_tracks);
    process_msg();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mod_sss");

    ROS_INFO("Initializing sss module........");

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


    int frameSize;

    if(!node_handle.getParam("general/frameSize", frameSize)) {

        ROS_ERROR("Couln'd retrieve general/frameSize");
        return -1;
    }


    int nTracks;

    if(!node_handle.getParam("config/nTracks", nTracks)) {

        ROS_ERROR("Couln'd retrieve config/nChannels");
        return -1;
    }


    int nThetas;

    if(!node_handle.getParam("config/nThetas", nThetas)) {

        ROS_ERROR("Couln'd retrive config/nThetas");
        return -1;
    }


    float gainMin;

    if(!node_handle.getParam("config/gainMin", gainMin)) {

        ROS_ERROR("Couln'd retrieve config/gainMin");
        return -1;
    }


    std::string spectra_topic, tracks_topic, out_topic;

    if(!node_handle.getParam("spectraTopic", spectra_topic)) {

        ROS_ERROR("Couln'd retrieve spectraTopic");
        return -1;
    }

    if(!node_handle.getParam("tracksTopic", tracks_topic)) {

        ROS_ERROR("Couln'd retrieve tracksTopic");
        return -1;
    }

    if(!node_handle.getParam("outTopic", out_topic)) {

        ROS_ERROR("Couln'd retrieve outTopic");
        return -1;
    }


    // Contruct objects

    ROS_INFO("Constructing objects........");

    msg_synchroniser_obj msg_synchroniser;

    msg_synchroniser.spectra_cfg = msg_spectra_cfg_construct();

    msg_synchroniser.spectra_cfg->fS = sample_rate_mu;
    msg_synchroniser.spectra_cfg->halfFrameSize = frameSize / 2;
    msg_synchroniser.spectra_cfg->nChannels = nChannels;


    msg_synchroniser.tracks_cfg = msg_tracks_cfg_construct();

    msg_synchroniser.tracks_cfg->fS = sample_rate_mu;
    msg_synchroniser.tracks_cfg->nTracks = nTracks;


    msg_spectra_cfg* msg_out_cfg = msg_spectra_cfg_construct();

    msg_out_cfg->fS = sample_rate_mu;
    msg_out_cfg->halfFrameSize = frameSize / 2;
    msg_out_cfg->nChannels = nTracks;

    msg_synchroniser.msg_out = msg_spectra_construct(msg_out_cfg);


    mod_sss_cfg* sss_cfg = mod_sss_cfg_construct();

    sss_cfg->gainMin = gainMin;
    sss_cfg->mics = mics;
    sss_cfg->nThetas = nThetas;
    sss_cfg->samplerate = sample_rate;
    sss_cfg->soundspeed = sound_speed;

    msg_synchroniser.mod_sss = mod_sss_construct(sss_cfg, msg_out_cfg, msg_synchroniser.tracks_cfg);


    // Wait for subscribers

    ROS_INFO("Waiting for subscribers........");

    ros::Publisher ros_publiser = public_handle.advertise<odas_ros::spectra>(out_topic, 2000);
    msg_synchroniser.ros_publisher = &ros_publiser;

    while(msg_synchroniser.ros_publisher->getNumSubscribers() < 1 && ros::ok()) {

        sleep(1);
    }

    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_spectra_subscriber = public_handle.subscribe<odas_ros::spectra>(spectra_topic, 2000, &msg_synchroniser_obj::process_spectra_msg, &msg_synchroniser);
    ros::Subscriber ros_tracks_subscriber = public_handle.subscribe<odas_ros::track>(tracks_topic, 2000, &msg_synchroniser_obj::process_tracks_msg, &msg_synchroniser);
    ros::spin();


    // Destroy objects

    ROS_INFO("Destroying objects........");

    mod_sss_destroy(msg_synchroniser.mod_sss);
    mod_sss_cfg_destroy(sss_cfg);

    msg_spectra_destroy(msg_synchroniser.msg_out);
    msg_spectra_cfg_destroy(msg_out_cfg);

    msg_spectra_cfg_destroy(msg_synchroniser.spectra_cfg);
    msg_tracks_cfg_destroy(msg_synchroniser.tracks_cfg);


    ROS_INFO("Quitting node........");

    return 0;
}
