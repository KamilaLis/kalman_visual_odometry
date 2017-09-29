
#include <ros/ros.h>
#include "monocamera_visual_odometry_kalman_filter/component_monocamera_visual_odometry_kalman_filter.h"

bool got_lost_;
ros::Publisher filtered_vel;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg,
                  boost::shared_ptr<ComponentFilter> filter_lin_ptr,
                  boost::shared_ptr<ComponentFilter> filter_ang_ptr)
{
    double lin_vel = msg->twist.twist.linear.z;
    double ang_vel = -(msg->twist.twist.angular.y);

    filter_lin_ptr->computeKalman(lin_vel, got_lost_);
    filter_ang_ptr->computeKalman(ang_vel, got_lost_);

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = filter_lin_ptr->getVelocity();
    twist_msg.angular.z = filter_ang_ptr->getVelocity();
    filtered_vel.publish(twist_msg);
}

void infoCallback(const viso2_ros::VisoInfo::ConstPtr& msg)
{
    got_lost_ = msg->got_lost;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "monocamera_visual_odometry_kalman_filter");

  boost::shared_ptr<ComponentFilter> filter_lin_ptr (new ComponentFilter());
  boost::shared_ptr<ComponentFilter> filter_ang_ptr (new ComponentFilter());

  ros::NodeHandle n;

  ros::Subscriber sub_odom_ = n.subscribe<nav_msgs::Odometry>("/mono_odometer/odometry", 1000,
                           boost::bind(odomCallback, _1, filter_lin_ptr, filter_ang_ptr));
  ros::Subscriber sub_info_ = n.subscribe("/mono_odometer/info", 1000, infoCallback);

  filtered_vel = n.advertise<geometry_msgs::Twist>("filtered_vel",1);

  ROS_INFO("monocamera_visual_odometry_kalman_filter_node STARTED :(");

  ros::spin();
  return 0;
}
