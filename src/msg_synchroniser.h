#ifndef msg_synchroniser_h
#define msg_synchroniser_h

#include "ros/ros.h"
#include "odas_ros/spectra.h"
#include "odas_ros/track.h"

#include <deque>

extern "C" {
    #include "odas/odas.h"
}


class sspf_synchroniser_obj {
public:
    sspf_synchroniser_obj();
    virtual ~sspf_synchroniser_obj();

    void construct_module();

    void process_stft_msg(const odas_ros::spectra::ConstPtr &stft_ros_msg);
    void process_sss_msg(const odas_ros::spectra::ConstPtr &sss_ros_msg);
    void process_tracks_msg(const odas_ros::track::ConstPtr &tracks_ros_msg);

    msg_spectra_cfg* stft_msg_cfg;
    msg_spectra_cfg* sss_msg_cfg;
    msg_tracks_cfg* tracks_msg_cfg;

    mod_sspf_cfg* sspf_cfg;

    msg_spectra_cfg* sspf_msg_cfg;

    ros::Publisher* ros_publisher;

private:
    void process_msg();
    bool sync_deques();

    mod_sspf_obj* mod_sspf;
    msg_spectra_obj* sspf_msg;

    std::deque<msg_spectra_obj*> stft_deque;
    std::deque<msg_spectra_obj*> sss_deque;
    std::deque<msg_tracks_obj*> tracks_deque;
};

#endif
