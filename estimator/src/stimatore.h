#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf/tf.h"
#include <Eigen/Geometry>


class STIMATORE{

    public:
        STIMATORE();
        void momentum_dot();
        void momentum();
        void Odomcallback(const nav_msgs::Odometry odometry_msg);
        void ForceThrustcallback(const std_msgs::Float32MultiArray force_thrust);
        void stima();       
	void run();

    private:
        Eigen::Matrix3d _Ib;
        Eigen::Vector3d _eta;
        Eigen::Vector3d _eta_dot;
        Eigen::Vector3d _tau_b;
        Eigen::Vector3d _vel;
        Eigen::Matrix3d _Jacobian;
        Eigen::Matrix<double,6,6> K1;
        Eigen::Matrix<double,6,6> K2;
        Eigen::Matrix<double,6,1> _momentum_dot;
        Eigen::Matrix<double,6,1> _momentum;
        Eigen::Matrix<double,6,1> _stima;
        double _mass;
        double _gravity;
        double _force;

        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _force_thrust_sub;
        ros::Publisher _stima_pub;
        nav_msgs::Odometry _odometr;
};
