/*
    File: utils.h
    -----------------
    pose definition and miscs. 
*/ 

#ifndef AUTONOMOUSFLIGHTUTILS_H
#define AUTONOMOUSFLIGHTUTILS_H
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <random>
#include <Eigen/Dense>
    
namespace AutoFlight{
    const double PI_const = 3.1415926;
    struct pose{
        double x;
        double y;
        double z;
        double yaw;
        pose(){
            x = 0; y = 0; z = 0; yaw = 0;
        }
        pose(double _x, double _y, double _z){
            x = _x; y = _y; z = _z; yaw = 0;
        }   

        pose(double _x, double _y, double _z, double _yaw){
            x = _x; y = _y; z = _z; yaw = _yaw;
        }
    };

    struct velocity{
        double vx;
        double vy;
        double vz;
        velocity(){
            vx = 0; vy = 0; vz = 0;
        }
        velocity(double _vx, double _vy, double _vz){
            vx = _vx; vy = _vy; vz = _vz;
        }
    };

    struct state{
        pose p;
        velocity v;
        state(){
            p = pose(0, 0, 0, 0);
            v = velocity(0, 0, 0);
        }
        state(const pose& _p, const velocity& _v){
            p = _p; v = _v;
        }
        state(double _px, double _py, double _pz, double _yaw, double _vx, double _vy, double _vz){
            p = pose(_px, _py, _pz, _yaw);
            v = velocity(_vx, _vy, _vz);
        }
    };

    inline std::ostream &operator<<(std::ostream &os, pose& pose){
        os << "pose: (" << pose.x << " " << pose.y << " " << pose.z << " " << pose.yaw << ")";
        return os;
    }


    inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
        if (yaw > PI_const){
            yaw = yaw - 2*PI_const;
        }
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    inline double rpy_from_quaternion(const geometry_msgs::Quaternion& quat){
        // return is [0, 2pi]
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    inline void rpy_from_quaternion(const geometry_msgs::Quaternion& quat, double &roll, double &pitch, double &yaw){
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }

    inline double getPoseDistance(const pose& p1, const pose& p2){
        return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));    

    }

    inline double getPoseDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2){
        return sqrt(pow((p1.position.x - p2.position.x),2) + pow((p1.position.y - p2.position.y),2) + pow((p1.position.z - p2.position.z),2));    
    }

    inline double getPoseDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
        return sqrt(pow((p1.pose.position.x - p2.pose.position.x),2) + pow((p1.pose.position.y - p2.pose.position.y),2) + pow((p1.pose.position.z - p2.pose.position.z),2));    
    }

    inline double getAngleDiff(double angle1, double angle2){
        double delta = std::abs(angle2 - angle1);
        if (delta > PI_const){
            delta = 2 * PI_const - delta;
        }
        return delta;
    }

    inline double getYawDistance(const pose& pStart, const pose& pTarget){
        double yaw1 = pStart.yaw;
        double yaw2 = pTarget.yaw;
        double delta = std::abs(yaw2 - yaw1);
        if (delta > PI_const){
            delta = 2 * PI_const - delta;
        }
        return delta;
    }
    
    // Helper Function: Random Number
    inline double randomNumber(double min, double max){
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<double> distribution(min, max);
        return distribution(mt);
    }

    inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
        Eigen::Matrix3d rotmat;
        rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
            2 * q(0) * q(2) + 2 * q(1) * q(3),

            2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
            2 * q(2) * q(3) - 2 * q(0) * q(1),

            2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
            q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
        return rotmat;
    }

    inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
        Eigen::Vector4d quat;
        double tr = R.trace();
        if (tr > 0.0){
            double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
            quat(0) = 0.25 * S;
            quat(1) = (R(2, 1) - R(1, 2)) / S;
            quat(2) = (R(0, 2) - R(2, 0)) / S;
            quat(3) = (R(1, 0) - R(0, 1)) / S;
        } 
        else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))){
            double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
            quat(0) = (R(2, 1) - R(1, 2)) / S;
            quat(1) = 0.25 * S;
            quat(2) = (R(0, 1) + R(1, 0)) / S;
            quat(3) = (R(0, 2) + R(2, 0)) / S;
        } 
        else if (R(1, 1) > R(2, 2)){
            double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
            quat(0) = (R(0, 2) - R(2, 0)) / S;
            quat(1) = (R(0, 1) + R(1, 0)) / S;
            quat(2) = 0.25 * S;
            quat(3) = (R(1, 2) + R(2, 1)) / S;
        } 
        else{
            double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
            quat(0) = (R(1, 0) - R(0, 1)) / S;
            quat(1) = (R(0, 2) + R(2, 0)) / S;
            quat(2) = (R(1, 2) + R(2, 1)) / S;
            quat(3) = 0.25 * S;
        }
        return quat;
    }
}
#endif