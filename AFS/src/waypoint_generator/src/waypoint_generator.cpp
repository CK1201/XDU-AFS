#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Eigen>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>
// #include <quadrotor_msgs/PolynomialTrajectory.h>
// #include <sensor_msgs/Joy.h>
// #include <algorithm>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;
using namespace Eigen;

int _trajectory_type = 0;
nav_msgs::Path _waypoints;

ros::Publisher _waypoints_pub;

void trajectory_type_cb(const std_msgs::Int32::ConstPtr &msg);
nav_msgs::Path circle();
nav_msgs::Path eight();

int main(int argc, char **argv){
    ros::init(argc, argv, "waypoint_generator_node");
    ros::NodeHandle nh("~");

    // nh.param("/waypoints/test", _test, true);

    ros::Subscriber trajectory_type_sub = nh.subscribe<std_msgs::Int32>("/AFS/trajectory_type", 10, trajectory_type_cb);
    _waypoints_pub = nh.advertise<nav_msgs::Path>("/AFS/waypoints", 10);

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();  
        rate.sleep();
    }

    return 0;
    // nav_msgs::Path waypoints;
    // geometry_msgs::PoseStamped pt;
    // pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

void trajectory_type_cb(const std_msgs::Int32::ConstPtr &msg){
    _trajectory_type = (int)(msg->data);
    switch (_trajectory_type)
    {
    case 1:
        _waypoints = circle();
        _waypoints_pub.publish(_waypoints);
        break;
    case 2:
        _waypoints = eight();
        _waypoints_pub.publish(_waypoints);
        break;
    default:
        break;
    }
}

// Circle
nav_msgs::Path circle()
{
    double h = 0.0;
    double scale = 1.0;
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);         

    return waypoints;
}

// Figure 8 trajectory
nav_msgs::Path eight()
{
    // Circle parameters
    double offset_x = 0.0;
    double offset_y = 0.0;
    double r = 1.0;
    double h = 1;
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);    

    for(int i=0; i< 1; ++i)
    {
        // First loop
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r + offset_y;
        pt.pose.position.z =  h/2+0.5;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r ;
        pt.pose.position.z =  h/2+0.5;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r + offset_x * 2;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0  + offset_x;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);
        // Second loop
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3+0.5;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r;
        pt.pose.position.z =  h / 2 * 3+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3+0.5;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y =  r + offset_y;
        pt.pose.position.z =  h / 2 * 3+0.5;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h+0.5;
        waypoints.poses.push_back(pt);  
    }
    return waypoints;   
}  