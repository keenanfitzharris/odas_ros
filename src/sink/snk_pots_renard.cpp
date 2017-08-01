#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include "odas_ros/pot.h"
#include "../ros_parameters_parser.h"


void process_pots_msg(const odas_ros::pot::ConstPtr &pot_msg, float balise_no, ros::Publisher &ros_publisher)
{
    std_msgs::Float32MultiArray array_msg;

    std::vector<float> msg_data = pot_msg->data;
    msg_data.push_back(balise_no);

    array_msg.data = msg_data;
    ros_publisher.publish(array_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "snk_pots_renard");

    ROS_INFO("Initializing sink pots renard........");

    ros::NodeHandle node_handle("~");
    ros::NodeHandle public_handle;


    // Get parameters from server

    ROS_INFO("Retrieving parameters........");

    int error_count = 0;

    float balise_no = retrieve_float(node_handle, "config/baliseNo", error_count);

    std::string inTopic = retrieve_string(node_handle, "inTopic", error_count);
    std::string outTopic = retrieve_string(node_handle, "outTopic", error_count);

    if(error_count > 0) {
        ROS_ERROR("Failed to retrieve parameters");
        return -1;
    }

    ros::Publisher ros_publiser = public_handle.advertise<std_msgs::Float32MultiArray>(outTopic, 2000);

    // Waiting for subscribers

    while(ros_publiser.getNumSubscribers() < 1 && ros::ok())
        sleep(1);


    // Proccess signal

    ROS_INFO("Processing signal........");

    ros::Subscriber ros_subscriber = public_handle.subscribe<odas_ros::pot>(inTopic, 2000, boost::bind(process_pots_msg, _1, balise_no, ros_publiser));
    ros::spin();


    ROS_INFO("Quitting node........");

    return 0;
}
