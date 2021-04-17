#include <string>
#include <iostream>
#include <cstdio>
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
#include <geometry_msgs/Polygon.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
// #include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>

using namespace std;
using namespace Eigen;

int _order;
double _dt = 0.01;
bool flag = false;
VectorXd _time;
MatrixXd _poly_coeff_matrix;
mavros_msgs::WaypointList _trajectoryList;
geometry_msgs::Polygon _velList;

ros::Publisher _trajectory_type_pub, _trajectoryList_pub, _trajectory_vel_pub;

void poly_coeff_cb(const nav_msgs::Path::ConstPtr &poly_coeff);
void waypointTraj(MatrixXd polyCoeff, VectorXd time);
Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t);
Vector3d getVelPoly(MatrixXd polyCoeff, int k, double t);

int main(int argc, char **argv){
    ros::init(argc, argv, "main_contorl_node");
    ros::NodeHandle nh("~");

    // nh.param("/trajectory/order",     _order, 4);

    ros::Subscriber poly_coeff_sub = nh.subscribe<nav_msgs::Path>("/AFS/poly_coeff", 10, poly_coeff_cb);
    _trajectory_type_pub = nh.advertise<std_msgs::Int32>("/AFS/trajectory_type", 10);
    _trajectoryList_pub = nh.advertise<mavros_msgs::WaypointList>("/mavros/mission/waypoints", 10);
    _trajectory_vel_pub= nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    int num = 0;
    geometry_msgs::Twist vel_curr;
    ros::Rate rate(1 / _dt);
    while(ros::ok()){
        if(flag){
            if(num<_velList.points.size()){
                vel_curr.linear.x = _velList.points[num].x;
                vel_curr.linear.y = _velList.points[num].y;
                vel_curr.linear.z = _velList.points[num].z;
                _trajectory_vel_pub.publish(vel_curr);
                num++;
            }
            else
            {
                flag = false;
            }
        }else{
            vel_curr.linear.x = 0;
            vel_curr.linear.y = 0;
            vel_curr.linear.z = 0;
            _trajectory_vel_pub.publish(vel_curr);
            num = 0;
        }
        ros::spinOnce();  
        rate.sleep();
    }

    return 0;
}

void poly_coeff_cb(const nav_msgs::Path::ConstPtr &poly_coeff){
    int row_size = poly_coeff->poses[0].pose.position.x, col_size = poly_coeff->poses[0].pose.position.y;
    _order = col_size / 2;
    MatrixXd poly_coeff_matrix(row_size, col_size * 3);
    int k = 1;
    for (int i = 0; i < row_size; i++){
        for (int j = 0; j < col_size;j++){
            poly_coeff_matrix(i, j) = poly_coeff->poses[k].pose.position.x;
            poly_coeff_matrix(i, j + col_size) = poly_coeff->poses[k].pose.position.y;
            poly_coeff_matrix(i, j + 2 * col_size) = poly_coeff->poses[k].pose.position.z;
            k++;
        }

    }
    VectorXd time(row_size);
    for (int i = 0; i < row_size; i++){
        time(i)=poly_coeff->poses[k].pose.position.x;
        k++;
    }
    _poly_coeff_matrix = poly_coeff_matrix;
    _time = time;
    // cout << "processed coeff:" << endl
    //      << _poly_coeff_matrix << endl;
    // cout << "processed time:" << endl
    //      << _time << endl;
    waypointTraj(_poly_coeff_matrix, _time);
    flag = true;
}

void waypointTraj( MatrixXd polyCoeff, VectorXd time)
{        
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "/map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    // _traj_vis.scale.x = _vis_traj_width;
    // _traj_vis.scale.y = _vis_traj_width;
    // _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    mavros_msgs::Waypoint trajectory;
    

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    // _traj_vis.points.clear();
    Vector3d pos, vel_3d;
    geometry_msgs::Point32 vel;
    geometry_msgs::Point pt;

    _trajectoryList.waypoints.clear();
    _velList.points.clear();
    for (int i = 0; i < time.size(); i++)
    {
        for (double t = 0.0; t < time(i); t += _dt, count += 1){
            pos = getPosPoly(polyCoeff, i, t);
            cur(0) = trajectory.x_lat = pt.x = pos(0);
            cur(1) = trajectory.y_long = pt.y = pos(1);
            cur(2) = trajectory.z_alt = pt.z = pos(2);
            _trajectoryList.waypoints.push_back(trajectory);
            _traj_vis.points.push_back(pt);
            if (count) traj_len += (pre - cur).norm();
            pre = cur;

            vel_3d = getVelPoly(polyCoeff, i, t);
            vel.x = vel_3d(0);
            vel.y = vel_3d(1);
            vel.z = vel_3d(2);
            _velList.points.push_back(vel);
        }
    }
    // cout << _velList.points << endl;
    _trajectoryList_pub.publish(_trajectoryList);

    // _wp_traj_vis_pub.publish(_traj_vis);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * 2*_order, 2*_order );
        VectorXd time  = VectorXd::Zero( 2*_order );
        
        for(int j = 0; j < 2*_order; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        // cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

Vector3d getVelPoly(MatrixXd polyCoeff, int k, double t){
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * 2*_order, 2*_order );
        VectorXd time  = VectorXd::Zero( 2*_order );
        
        for(int j = 1; j < 2*_order; j ++){
            time(j) = pow(t, j - 1) * j;
        }
        // cout << time << endl;
        // if(j==1)
        //     time(j) = 1.0;
        // else
        //     time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        // cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    // cout << ret << endl;

    return ret;
}