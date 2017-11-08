#include "monocamera_visual_odometry_kalman_filter/component_filter.h"

ComponentFilter::ComponentFilter()
{
    // Read local parameters
    ros::NodeHandle local_nh("~");
    local_nh.getParam("kal_Q", kal_Q_);
    local_nh.getParam("kal_R", kal_R_);
    //ROS_INFO("kal_Q: %f, kal_R: %f", kal_Q_, kal_R_);
    //kal_Q_=10.0;
    //kal_R_=0.01;

    // Initialize matrices
    A_ = Matrix::eye(2);
    G_ = Matrix(2,1);
    double C_data[2] = {1,0};
    C_ = Matrix(1,2, C_data);
}

double ComponentFilter::getVelocity()
{
    return x_k_.val[0][0];
}

void ComponentFilter::computeKalman(double odom_vel, bool got_lost)
{
    ros::Time current_rostime = ros::Time::now();
    double dt = current_rostime.toSec() - last_rostime_;
    updateMatrices(dt);
    countEstimation();
    if (!got_lost)
    {
        updateMeasurements(odom_vel, dt);
    }
    last_rostime_= current_rostime.toSec();
}



void ComponentFilter::updateMeasurements(double odom_vel, double dt)
{
    double eps = odom_vel - (C_*x_k_).val[0][0];
    double S = (C_*P_k_*~C_).val[0][0] + kal_R_;
    Matrix K = P_k_*~C_/S;
    x_k_ = x_k_ + K*eps;
    P_k_ = P_k_-K*S*~K;
}

void ComponentFilter::countEstimation()
{
    x_k_ = A_*x_k_;
    P_k_ = A_*P_k_*~A_ + G_*kal_Q_*~G_;
}

void ComponentFilter::updateMatrices(double dt)
{   
    A_.val[0][1]=dt;
    G_.val[0][0]=(dt*dt)/2;
    G_.val[1][0]=dt;
}

void ComponentFilter::computeMean(double odom_vel, bool got_lost)
{
    if(!got_lost)
    {
        mean_ = (last_vel_+old_vel_+odom_vel)/3;
        x_k_.val[0][0] = mean_;
    }
}



