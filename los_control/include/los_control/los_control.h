#ifndef LOS_CONTROL_H
#define LOS_CONTROL_H

#endif // LOS_CONTROL_H
#include <tf/tf.h>          //tf转换
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <mutex>
#include <defineMessage/task_reseive.h>
#include <defineMessage/los_control.h>

using namespace std;
#define PI 3.14159265

typedef pcl::PointXYZI PointType;

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;


enum taskState{
    TASKNORMAL = 0,
    TASKSTOP,
    TASKCONTINUE

};
enum motionState{
    MOTION_AHEAD = 0,
    MOTION_BACK,
};
enum goalState{
    HAVE_ARRIVED_GOAL = 0,
    HAVE_NOT_ARRIVED_GOAL

};
enum rotateState{
    HAVE_ROTATE_ANGLE = 0,
    HAVE_NOT_ROTATE_ANGLE

};
