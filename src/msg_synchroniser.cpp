#include "msg_synchroniser.h"

sspf_synchroniser_obj::sspf_synchroniser_obj()
{

    stft_msg_cfg = msg_spectra_cfg_construct();
    sss_msg_cfg = msg_spectra_cfg_construct();
    tracks_msg_cfg = msg_tracks_cfg_construct();

    sspf_cfg = mod_sspf_cfg_construct();

    sspf_msg_cfg = msg_spectra_cfg_construct();

    ros_publisher = nullptr;

    mod_sspf = nullptr;
    sspf_msg = nullptr;
}


sspf_synchroniser_obj::~sspf_synchroniser_obj()
{
    msg_spectra_cfg_destroy(stft_msg_cfg);
    msg_spectra_cfg_destroy(sss_msg_cfg);
    msg_tracks_cfg_destroy(tracks_msg_cfg);

    mod_sspf_destroy(mod_sspf);

    msg_spectra_cfg_destroy(sspf_msg_cfg);
    msg_spectra_destroy(sspf_msg);

    ros_publisher = nullptr;

    while(stft_deque.size() > 0) {

        msg_spectra_destroy(stft_deque.back());
        stft_deque.pop_back();
    }

    while(sss_deque.size() > 0) {

        msg_spectra_destroy(sss_deque.back());
        sss_deque.pop_back();
    }

    while(tracks_deque.size() > 0) {

        msg_tracks_destroy(tracks_deque.back());
        tracks_deque.pop_back();
    }
}


void sspf_synchroniser_obj::construct_module()
{
    mod_sspf = mod_sspf_construct(sspf_cfg, sspf_msg_cfg, tracks_msg_cfg);
    sspf_msg = msg_spectra_construct(sspf_msg_cfg);
}


void sspf_synchroniser_obj::process_stft_msg(const odas_ros::spectra::ConstPtr &stft_ros_msg)
{
    msg_spectra_obj* stft_msg = msg_spectra_construct(stft_msg_cfg);

    stft_msg->timeStamp = stft_ros_msg->timeStamp;
    stft_msg->fS = stft_ros_msg->fS;
    stft_msg->freqs->nSignals = stft_ros_msg->nSignals;
    stft_msg->freqs->halfFrameSize = stft_ros_msg->halfFrameSize;

    if(stft_msg->fS == 0) {

        odas_ros::spectra sspf_ros_msg;
        sspf_ros_msg.fS = 0;
        ros_publisher->publish(sspf_ros_msg);

        ros::shutdown();
        return;
    }

    int nSignals = stft_msg->freqs->nSignals;
    int nSamples = stft_msg->freqs->halfFrameSize * 2;

    for(int iSignals = 0; iSignals < nSignals; iSignals++)
        for(int iSamples = 0; iSamples < nSamples; iSamples++)
            stft_msg->freqs->array[iSignals][iSamples] = stft_ros_msg->data[iSamples + iSignals * nSamples];

    stft_deque.push_front(stft_msg);
    process_msg();
}


void sspf_synchroniser_obj::process_sss_msg(const odas_ros::spectra::ConstPtr &sss_ros_msg)
{
    msg_spectra_obj* sss_msg = msg_spectra_construct(sss_msg_cfg);

    sss_msg->timeStamp = sss_ros_msg->timeStamp;
    sss_msg->fS = sss_ros_msg->fS;
    sss_msg->freqs->nSignals = sss_ros_msg->nSignals;
    sss_msg->freqs->halfFrameSize = sss_ros_msg->halfFrameSize;

    if(sss_msg->fS == 0) {

        odas_ros::spectra sspf_ros_msg;
        sspf_ros_msg.fS = 0;
        ros_publisher->publish(sspf_ros_msg);

        ros::shutdown();
        return;
    }

    int nSignals = sss_msg->freqs->nSignals;
    int nSamples = sss_msg->freqs->halfFrameSize * 2;

    for(int iSignals = 0; iSignals < nSignals; iSignals++)
        for(int iSamples = 0; iSamples < nSamples; iSamples++)
            sss_msg->freqs->array[iSignals][iSamples] = sss_ros_msg->data[iSamples + iSignals * nSamples];

    sss_deque.push_front(sss_msg);
    process_msg();
}


void sspf_synchroniser_obj::process_tracks_msg(const odas_ros::track::ConstPtr &tracks_ros_msg)
{
    msg_tracks_obj* tracks_msg = msg_tracks_construct(tracks_msg_cfg);

    tracks_msg->timeStamp = tracks_ros_msg->timeStamp;
    tracks_msg->fS = tracks_ros_msg->fS;
    tracks_msg->tracks->nTracks = tracks_ros_msg->nTracks;

    if(tracks_msg->fS == 0) {

        odas_ros::spectra sspf_ros_msg;
        sspf_ros_msg.fS = 0;
        ros_publisher->publish(sspf_ros_msg);

        ros::shutdown();
        return;
    }

    int nTracks = tracks_msg->tracks->nTracks;

    for(int iTracks = 0; iTracks < nTracks; iTracks++) {

        tracks_msg->tracks->ids[iTracks] = tracks_ros_msg->ids[iTracks];

        for(int iSamples = 0; iSamples < 3; iSamples++)
            tracks_msg->tracks->array[iSamples + iTracks * 3] = tracks_ros_msg->data[iSamples + iTracks * 3];
    }

    tracks_deque.push_front(tracks_msg);
    process_msg();
}


bool sspf_synchroniser_obj::sync_deques()
{
    long long unsigned int newest_timestamp = 0;

    msg_spectra_obj* last_stft;
    msg_spectra_obj* last_sss;
    msg_tracks_obj* last_tracks;

    if(stft_deque.size() > 0) {

        last_stft = stft_deque.back();

        if(last_stft->timeStamp > newest_timestamp)
            newest_timestamp = last_stft->timeStamp;
    }
    else
        return false;

    if(sss_deque.size() > 0) {

        last_sss = sss_deque.back();

        if(last_sss->timeStamp > newest_timestamp)
            newest_timestamp = last_sss->timeStamp;
    }
    else
        return false;

    if(tracks_deque.size() > 0) {

        last_tracks = tracks_deque.back();

        if(last_tracks->timeStamp > newest_timestamp)
            newest_timestamp = last_tracks->timeStamp;
    }
    else
        return false;

    bool is_newest = true;

    while(stft_deque.size() > 0 && last_stft->timeStamp < newest_timestamp) {

        msg_spectra_destroy(last_stft);
        stft_deque.pop_back();
        last_stft = stft_deque.back();
    }

    is_newest = is_newest && last_stft->timeStamp == newest_timestamp;

    while(sss_deque.size() > 0 && last_sss->timeStamp < newest_timestamp) {

        msg_spectra_destroy(last_sss);
        sss_deque.pop_back();
        last_sss = sss_deque.back();
    }

    is_newest = is_newest && last_sss->timeStamp == newest_timestamp;

    while(tracks_deque.size() > 0 && last_tracks->timeStamp < newest_timestamp) {

        msg_tracks_destroy(last_tracks);
        tracks_deque.pop_back();
        last_tracks = tracks_deque.back();
    }

    is_newest = is_newest && last_tracks->timeStamp == newest_timestamp;
}


void sspf_synchroniser_obj::process_msg()
{
    if(!sync_deques())
        return;

    mod_sspf_connect(mod_sspf, stft_deque.back(), sss_deque.back(), tracks_deque.back(), sspf_msg);
    mod_sspf_process(mod_sspf);
    mod_sspf_disconnect(mod_sspf);

    msg_spectra_destroy(stft_deque.back());
    stft_deque.pop_back();

    msg_spectra_destroy(sss_deque.back());
    sss_deque.pop_back();

    msg_tracks_destroy(tracks_deque.back());
    tracks_deque.pop_back();

    odas_ros::spectra sspf_ros_msg;

    sspf_ros_msg.timeStamp = sspf_msg->timeStamp;
    sspf_ros_msg.fS = sspf_msg->fS;
    sspf_ros_msg.halfFrameSize = sspf_msg->freqs->halfFrameSize;
    sspf_ros_msg.nSignals = sspf_msg->freqs->nSignals;

    int nSignals = sspf_ros_msg.nSignals;
    int nSamples = sspf_ros_msg.halfFrameSize * 2;

    sspf_ros_msg.data = std::vector<float>(nSignals * nSamples);

    for(int iSignals = 0; iSignals < nSignals; iSignals++)
        for(int iSamples = 0; iSamples < nSamples; iSamples++)
            sspf_ros_msg.data[iSamples + iSignals * nSamples] = sspf_msg->freqs->array[iSignals][iSamples];

    ros_publisher->publish(sspf_ros_msg);
}
