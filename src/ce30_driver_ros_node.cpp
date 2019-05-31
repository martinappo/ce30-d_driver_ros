#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <ce30_driver/ce30_driver.h>

using namespace std;
using namespace ce30_driver;

ros::Publisher gPub;
std::string gFrameID = "ce30d";

void DataReceiveCB(shared_ptr<PointCloud> cloud) {
    sensor_msgs::PointCloud pointcloud;
    pointcloud.header.stamp = ros::Time::now();
    pointcloud.header.frame_id = gFrameID;
    static int point_num = 320 * 20;
    pointcloud.points.reserve(point_num);
    for (auto &point : cloud->points) {
        static geometry_msgs::Point32 ros_point;
        ros_point.x = point.x;
        ros_point.y = point.y;
        ros_point.z = point.z;
        pointcloud.points.push_back(ros_point);
    }
    if (gPub.getNumSubscribers() > 0) {
        gPub.publish(pointcloud);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ce30d_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(30);
    std::string ip, newIp;
    nh.getParam("ip", ip);
    nh.getParam("newIp", newIp);
    nh.param<std::string>("frameId", gFrameID, gFrameID);
    gPub = nh.advertise<sensor_msgs::PointCloud>("ce30_points", 1);
    UDPServer server;

    if (ip.empty()) {
        ROS_ERROR("No ip parameter provided, exiting!");
        return -1;
    }
    server.SetIP(ip);

    if (!newIp.empty()) {
        ROS_INFO("newIp parameter provided, changing device ip...");
        if (server.ChangeDeviceIp(newIp)) {
            ROS_INFO("new ip applied successfully! Please restart the node without newIp parameter.");
            return 0;
        } else {
            ROS_ERROR("Device ip change failed! Inspect logs above.");
            return -1;
        }
    }

    server.RegisterCallback(DataReceiveCB);
    if (!server.Start()) {
        ROS_ERROR("Could not connect to the device. Inspect logs above.");
        return -1;
    } else {
        ROS_INFO("CE30D with frame id %s and ip %s started.", gFrameID.c_str(), ip.c_str());
    }
    while (ros::ok()) {
        server.SpinOnce();
        rate.sleep();
    }
}
