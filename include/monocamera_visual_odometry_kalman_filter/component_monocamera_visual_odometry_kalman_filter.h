#include <iostream>
#include <stdlib.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <viso2_ros/VisoInfo.h>

#include <matrix.h>

class ComponentFilter
{
public:

    ComponentFilter();
    void computeKalman(double odom_vel, bool got_lost);
    double getVelocity();

protected:

    void updateMatrices(double dt);
    void countEstimation();
    void updateMeasurements(double odom_vel, double dt);

private:

    double last_rostime_;

    Matrix P_k_ = Matrix::eye(2);
    Matrix x_k_ = Matrix(2,1);
    double kal_Q_;
    double kal_R_;

    Matrix A_;
    Matrix G_;
    Matrix C_;

};


