//
// Created by zain on 5/8/17.
//

#ifndef G2O_PLAYGROUND_GRAPHSLAM_H
#define G2O_PLAYGROUND_GRAPHSLAM_H

#endif //G2O_PLAYGROUND_GRAPHSLAM_H

#include <rosbag/bag.h>
#include <iostream>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <boost/foreach.hpp>
#include <map>
#include<cmath>
#include<geometry_msgs/Pose2D.h>
#include<graph_slam_visualizer/VertexSE2.h>
#include<graph_slam_visualizer/EdgeSE2.h>
#include <graph_slam_visualizer/GraphSE2.h>
#include <ros/publisher.h>
#include <geometry_msgs/Vector3.h>
#include<ros/ros.h>
#include <g2o/core/factory.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/block_solver.h>
#include <tf/transform_datatypes.h>
#include <g2o_playground/distance.h>
#include <g2o_playground/distance_vector.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/subscriber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace g2o;
using namespace std;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        i=1;
        j=1;
        pub_=nodeHandle.advertise<graph_slam_visualizer::GraphSE2>("bablu",10);     // Publisher that publishes on bablu topic the resultant graphslam visualizer messages
        sub_=nodeHandle.subscribe("/pose2D",1,&SubscribeAndPublish::mega,this);     // It subcribes to pose2D from scan matcher
        k[0]=2;k[1]=2;k[2]=1;
        m.diagonal()= k;
        g2o_initialize();
    }

private:

    //*****ros

    ros::NodeHandle nodeHandle;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    // *****g2o
    g2o::SparseOptimizer g2o_graph;
    g2o::SE2 new_pose;
    g2o::SE2 pose_current;
    g2o::SE2 pose_previous;
    g2o::VertexSE2 *prev_pose;
    g2o::VertexSE2 *current_pose;
    g2o::EdgeSE2 * odom_edge;

    //**** graph slam visualizer
    graph_slam_visualizer::VertexSE2 vert;
    graph_slam_visualizer::EdgeSE2 edges;
    graph_slam_visualizer::GraphSE2 graph_pub;

    // **** Custom variables
    int i;  // Counter for node index
    int j;  // Counter for sampling
    Eigen::DiagonalMatrix<double,3> m;
    Eigen::Vector3d k;
    double gsv[3];

    // **** Custom messages
    g2o_playground::distance temp_distance_store;
    g2o_playground::distance_vector distances_bw_vertices;

    //**** methods
    void closest_vertex(g2o::VertexSE2 *latest);
    void optimize();
    void adding_vertices_edges();
    void g2o_initialize();
    void mega(const geometry_msgs::Pose2D::ConstPtr &msg);
    void make_g2o_file();
};
