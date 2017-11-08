
#include <ros/ros.h>
#include "monocamera_visual_odometry_kalman_filter/component_filter.h"
//
double lin_vel_;
double ang_vel_;
//
bool got_lost_;
bool first_found = false;
ros::Publisher filtered_vel;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg,
                  boost::shared_ptr<ComponentFilter> filter_lin_ptr,
                  boost::shared_ptr<ComponentFilter> filter_ang_ptr)
{
    lin_vel_= msg->twist.twist.linear.z;
    ang_vel_= -(msg->twist.twist.angular.y);

    /*
    double lin_vel = msg->twist.twist.linear.z;
    double ang_vel = -(msg->twist.twist.angular.y);

    filter_lin_ptr->computeKalman(lin_vel, got_lost_);
    filter_ang_ptr->computeKalman(ang_vel, got_lost_);

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = filter_lin_ptr->getVelocity();
    twist_msg.angular.z = filter_ang_ptr->getVelocity();
    filtered_vel.publish(twist_msg);
    */
}

void infoCallback(const viso2_ros::VisoInfo::ConstPtr& msg,
                  boost::shared_ptr<ComponentFilter> filter_lin_ptr,
                  boost::shared_ptr<ComponentFilter> filter_ang_ptr)
{
    got_lost_ = msg->got_lost;
    if (!first_found && !got_lost_) first_found = true;
    if (first_found)
    {   /*
        filter_lin_ptr->computeKalman(lin_vel_, got_lost_);
        filter_ang_ptr->computeKalman(ang_vel_, got_lost_);
        */
        filter_lin_ptr->computeMean(lin_vel_, got_lost_);
        filter_ang_ptr->computeMean(ang_vel_, got_lost_);

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = filter_lin_ptr->getVelocity();
        twist_msg.angular.z = filter_ang_ptr->getVelocity();
        filtered_vel.publish(twist_msg);

        filter_lin_ptr->last_vel_ = filter_lin_ptr->getVelocity();
        filter_ang_ptr->last_vel_ = filter_ang_ptr->getVelocity();
        filter_lin_ptr->old_vel_ = filter_lin_ptr->last_vel_;
        filter_ang_ptr->old_vel_ = filter_ang_ptr->last_vel_;
    }
    else ROS_INFO("Waiting for data from viso2...");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "monocamera_visual_odometry_kalman_filter");
  ROS_INFO("MONOODOMETER KALMAN FILTER STARTED !!");

  boost::shared_ptr<ComponentFilter> filter_lin_ptr (new ComponentFilter());
  boost::shared_ptr<ComponentFilter> filter_ang_ptr (new ComponentFilter());

  ros::NodeHandle n;
  ros::Subscriber sub_odom_ = n.subscribe<nav_msgs::Odometry>("/mono_odometer/odometry", 1000,
                           boost::bind(odomCallback, _1, filter_lin_ptr, filter_ang_ptr));
  ros::Subscriber sub_info_ = n.subscribe<viso2_ros::VisoInfo>("/mono_odometer/info", 1000, 
                           boost::bind(infoCallback, _1, filter_lin_ptr, filter_ang_ptr));

  filtered_vel = n.advertise<geometry_msgs::Twist>("filtered_vel",1);

  ros::spin();
  return 0;
}
