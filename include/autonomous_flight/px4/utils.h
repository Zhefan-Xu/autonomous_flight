/*
    File: utils.h
    -----------------
    Autonomous Flight utils and miscs. 
*/ 

#ifndef AUTOFLIGHTUTILS_H
#define AUTOFLIGHTUTILS_H
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <random>


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
}

#endif