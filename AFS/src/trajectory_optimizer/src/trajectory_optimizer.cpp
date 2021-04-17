#include <string>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <math.h>
#include <cmath>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
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

mavros_msgs::State _current_state;
geometry_msgs::PoseStamped _local_pos;
geometry_msgs::TwistStamped _local_vel;
sensor_msgs::Imu _imu_data;
nav_msgs::Path _waypoints;
double _max_vel, _max_acc;
int _order;
// std_msgs::Float64MultiArray _time;

void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
void imu_data_cb(const sensor_msgs::Imu::ConstPtr &msg);
void waypoint_cb(const nav_msgs::Path::ConstPtr &wp);
VectorXd allocateTime(MatrixXd path);
int Factorial(int x);
MatrixXd PolyQPGeneration(const int d_order, const MatrixXd &Path, const MatrixXd &Vel, const MatrixXd &Acc, const VectorXd &Time);

ros::Publisher _poly_coeff_pub;

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_optimizer_node");
    ros::NodeHandle nh("~");

    nh.param("/trajectory/max_vel",   _max_vel,   1.0 );
    nh.param("/trajectory/max_acc",   _max_acc,   1.0 );
    nh.param("/trajectory/order",     _order, 4);


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 10, local_vel_cb);
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_data_cb);
    ros::Subscriber waypoints_sub = nh.subscribe<nav_msgs::Path>("/AFS/waypoints", 10, waypoint_cb);

    _poly_coeff_pub = nh.advertise<nav_msgs::Path>("/AFS/poly_coeff", 10);
    // _time_pub = nh.advertise<std_msgs::Float64MultiArray>("/AFS/time", 10);

    ros::Rate rate(100);
    // while(ros::ok() && !_current_state.connected){
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    while(ros::ok()){
        ros::spinOnce();  
        rate.sleep();
    }

    return 0;
}

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    _current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    _local_pos = *msg;
    // printf("local position:(%lf,%lf,%lf)\n", local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z);
}

void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    _local_vel = *msg;
}

void imu_data_cb(const sensor_msgs::Imu::ConstPtr &msg){
    _imu_data = *msg;
}

void waypoint_cb(const nav_msgs::Path::ConstPtr &wp_temp){
    nav_msgs::Path wp = *wp_temp;
    vector<Vector3d> wp_list;
    wp_list.clear();
    Vector3d start_pose(_local_pos.pose.position.x, _local_pos.pose.position.y, _local_pos.pose.position.z);
    for (int k = 0; k < (int)wp.poses.size(); k++){
        Vector3d pt(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        pt = pt + start_pose;
        wp_list.push_back(pt);
        // if(wp.poses[k].pose.position.z < 0.0)
        //     break;
    }
    MatrixXd path(wp_list.size(), 3);
    for(int k = 0; k < (int)wp_list.size(); k++)
        path.row(k) = wp_list[k];

    MatrixXd vel = MatrixXd::Zero(2, 3);
    MatrixXd acc = MatrixXd::Zero(2, 3);
    vel(0, 0) = _local_vel.twist.linear.x;
    vel(0, 1) = _local_vel.twist.linear.y;
    vel(0, 2) = _local_vel.twist.linear.z;

    if(true){
        VectorXd time = allocateTime(path);
        // cout << "origin time:" << endl
        //      << time << endl;
        MatrixXd poly_coeff_matrix = PolyQPGeneration(_order, path, vel, acc, time);
        // cout << "origin coeff:" << endl
        //      << poly_coeff_matrix << endl;
        int poly_coeff_num = _order * 2;
        nav_msgs::Path poly_coeff;
        geometry_msgs::PoseStamped poly;
        poly.pose.position.x = poly_coeff_matrix.rows();
        poly.pose.position.y = poly_coeff_num;
        poly.pose.position.z = 0;
        poly_coeff.poses.push_back(poly);
        // printf("size of poly_coeff_matrix:(%d,%d)\n", poly_coeff_matrix.rows(), poly_coeff_matrix.cols());
        for (int i = 0; i < poly_coeff_matrix.rows(); i++)
        {
            for (int j = 0; j < poly_coeff_num;j++){
                // printf("(%d,%d)\n", i, j);
                poly.pose.position.x = poly_coeff_matrix(i, j);
                poly.pose.position.y = poly_coeff_matrix(i, j+poly_coeff_num);
                poly.pose.position.z = poly_coeff_matrix(i, j + 2 * poly_coeff_num);
                poly_coeff.poses.push_back(poly);
            }
        }
        poly.pose.position.y = 0;
        poly.pose.position.z = 0;
        for (int i = 0; i < time.size(); i++)
        {
            poly.pose.position.x = time(i);
            poly_coeff.poses.push_back(poly);
        }
        // poly_coeff.header.seq = poly_coeff_matrix.rows();
        // poly_coeff.header.stamp = poly_coeff_num;
        _poly_coeff_pub.publish(poly_coeff);
    }
}

VectorXd allocateTime(MatrixXd path){
    VectorXd time(path.rows() - 1);
    double t_acc = _max_vel / _max_acc;
    double s_acc = _max_acc * t_acc * t_acc / 2;
    double distance = 1;
    for (int i = 0; i < time.size(); i++)
    {
        distance = sqrt(
            (path(i, 0) - path(i + 1, 0)) * (path(i, 0) - path(i + 1, 0)) +
            (path(i, 1) - path(i + 1, 1)) * (path(i, 1) - path(i + 1, 1)) +
            (path(i, 2) - path(i + 1, 2)) * (path(i, 2) - path(i + 1, 2)));
        if (distance < (s_acc * 2))
        {
            time(i) = sqrt(distance / _max_acc);
        }
        else{
            time(i) = (distance - s_acc * 2) / _max_vel + 2 * t_acc;
        }
    }
    return time;
}

int Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

//minimum snap
MatrixXd PolyQPGeneration(
            const int d_order,
            const MatrixXd &Path,
            const MatrixXd &Vel,
            const MatrixXd &Acc,
            const VectorXd &Time)
{
    int p_order   = 2 * d_order - 1;
    int p_num1d   = p_order + 1;

    int m = Time.size();
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    MatrixXd M = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    MatrixXd Mi;

    for (int k = 0; k < m; k++)
    {
        Mi = MatrixXd::Zero(p_num1d, p_num1d);
        //time=0
        Mi(0, 0) = 1;
        Mi(1, 1) = 1;
        Mi(2, 2) = 2;
        Mi(3, 3) = 6;
        //p
        for (int i = 0; i < p_num1d;i++){
            Mi(4, i) = pow(Time(k), i);
        }
        //v
        for (int i = 1; i < p_num1d;i++){
            Mi(5, i) = i * pow(Time(k), i - 1);
        }
        //a
        for (int i = 2; i < p_num1d;i++){
            Mi(6, i) = i * (i - 1) * pow(Time(k), i - 2);
        }
        //j
        for (int i = 3; i < p_num1d;i++){
            Mi(7, i) = i * (i - 1) * (i - 2) * pow(Time(k), i - 3);
        }
        M.block(k * p_num1d, k * p_num1d, p_num1d, p_num1d) = Mi;
    }
    // cout << "M:" << endl
    //      << M << endl;

    MatrixXd Ct = MatrixXd::Zero(2 * d_order * m, d_order * (m + 1));
    Ct(0, 0) = 1;
    Ct(1, 1) = 1;
    Ct(2, 2) = 1;
    Ct(3, 3) = 1;

    Ct(2 * d_order * m - 4, d_order + m - 1) = 1;
    Ct(2 * d_order * m - 3, d_order + m + 0) = 1;
    Ct(2 * d_order * m - 2, d_order + m + 1) = 1;
    Ct(2 * d_order * m - 1, d_order + m + 2) = 1;

    for (int i = 0; i < m-1; i++){
        
        Ct(d_order+2*i*d_order,d_order+i) = 1;
        Ct(d_order+2*i*d_order+d_order,d_order+i) = 1;

        Ct(d_order + 2 * i * d_order + 1, 2 * d_order + m + i * (d_order - 1) - 1) = 1; // v_end
        Ct(d_order+2*i*d_order+2,2*d_order+m+i*(d_order-1)+0)=1;// a_end
        Ct(d_order+2*i*d_order+3,2*d_order+m+i*(d_order-1)+1)=1;// j_end

        Ct(d_order+2*i*d_order+1+d_order,2*d_order+m+i*(d_order-1)-1)=1;// v_start
        Ct(d_order+2*i*d_order+2+d_order,2*d_order+m+i*(d_order-1)+0)=1;// a_start
        Ct(d_order+2*i*d_order+3+d_order,2*d_order+m+i*(d_order-1)+1)=1;// j_start
    }
    // cout << "Ct:" << endl
    //      << Ct << endl;
    auto C = Ct.transpose();

    MatrixXd Q = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    MatrixXd Qi;
    for (int k = 0; k < m; k++)
    {
        Qi = MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = 3; i < p_num1d; i++){
            for (int l = 3; l < p_num1d ; l++){
                Qi(i, l) = (i + 1) * i * (i - 1) * (i - 2) * (l + 1) * l * (l - 1) * (l - 2) / (i + l + 2 - 7) * pow(Time(k), i + l + 2 - 7);
            }
        }
        Q.block(k * p_num1d, k * p_num1d, p_num1d, p_num1d) = Qi;
    }
    // cout << "Q:" << endl
    //      << Q << endl;

    auto R = C * M.inverse().transpose() * Q * M.inverse() * Ct;

    int Fsize = Path.size() / 3 - 2 + 2 * 4;
    int Psize = R.rows() - Fsize;
    auto R_pp = R.block(Fsize, Fsize, Psize, Psize);
    auto R_fp = R.block(0, Fsize, Fsize, Psize);
    cout << "Path:" << endl
            << Path << endl;
    for (int i = 0; i < 3;i++){
        
        Vector4d start_cond(Path(0, i),Vel(0,i),Acc(1,i),0), end_cond(Path(Path.size() / 3-1, i),Vel(1,i),Acc(1,i),0);
        VectorXd dF(Fsize);
        dF << start_cond, Path.block(1,i,Path.size() / 3 - 2, 1), end_cond;
        auto dP = -R_pp.inverse() * R_fp.transpose() * dF;
        VectorXd d(Fsize + Psize);
        d << dF, dP;
        
        MatrixXd P2 = M.inverse() * Ct * d;
        MatrixXd P1 = P2.transpose();
        // cout << "111111111111" << endl;
        MatrixXd P(m, p_num1d);
        // cout << "222222222222" << endl;
        for (int j = 0; j < m; j++)
        {
            P.block(j,0,1,p_num1d) = P1.block(0, j * p_num1d, 1, p_num1d);
        }
        // cout << "333333333333" << endl;
        PolyCoeff.block(0, i * p_num1d, m, p_num1d) = P;
    }
    // cout << "PolyCoeff:" << endl
            // << PolyCoeff << endl;
    return PolyCoeff;
}
