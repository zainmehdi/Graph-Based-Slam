//
// Created by zain on 4/13/17.
//


// This file contains the core code that handles data from scan matcher and builds a graph as sequential poses of robot in 2D.
// It also contains loop clsoing functions that handles contraint consistency check.

/* brief 1: => Loop closer
     Whenever there is a new scan from scan matcher it is compared with previous poses skipping a specified number to ensure
     loop is not closed with nearby poses as graph is built. First we check for the closes pose in our graph using closest
     vertex checker function. It gives us the most probable candidate for loop closing. It is then checked for consistency
     with the current pose using pcl point cloud matching. If the match is below a speific threshold we have a close match
     and the two poses match very closely. Hence we add a loop closing edge(This approach however has been found to be
     least robust as wrong loop closures have made our graph diverted and faulty).To cover up this deficiency we have two
     options

     1) we can opt for very robust place recognition system
     2) we can improve our loop closing algorithm

     We have opted for the second approach and work is being done on it.
*/

#include <rosbag/bag.h>
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
//#include <pcl/io/pcd_io.h>
//#include <pcl/conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/PCLPointCloud2.h>
//#include <pcl_ros/transforms.h>
//#include <pcl/registration/gicp.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <iostream>







#define foreach BOOST_FOREACH

using namespace g2o;
using namespace std;
G2O_USE_TYPE_GROUP(slam2d);


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

    void mega(const geometry_msgs::Pose2D::ConstPtr &msg)
    {

     //   cerr << "Calculating relative measurements..."<<endl;
       if(j%10 == 0 ) {     // Sampling every 10th measurement from published pose2D data by scan matcher

           if (i == 1) {

               Vector3D poze(msg->x, msg->y, msg->theta);
               vert.x = msg->x;
               vert.y = msg->y;
               vert.theta = msg->theta;

               edges.vi_idx = i - 1;
               edges.vj_idx = i;
               pose_previous.fromVector(poze);
               adding_vertices_edges();
               i++;

           }
           if (i > 1) {
               Vector3D poze(msg->x, msg->y, msg->theta);
               new_pose.fromVector(poze);
               pose_current = pose_previous.inverse().operator*(new_pose);    // relative measurement
               pose_previous = new_pose;

            //   vert.x = pose_current.toVector()[0];
             //  vert.y = pose_current.toVector()[1];
             //  vert.theta = pose_current.toVector()[2];




               adding_vertices_edges();

               optimize();                            // When we have robust loop closing this function will only be called if loop closing edge is detected

               /////////////////////////////////////////////////////////////////////
               VertexSE2* RobotPose = dynamic_cast<VertexSE2*>(g2o_graph.vertex(i));
               RobotPose->getEstimateData(gsv);
               /////////////////////////////////////////////////////////////////////
               edges.vi_idx = i - 1;
               edges.vj_idx = i;
               vert.x=gsv[0];
               vert.y=gsv[1];
               vert.theta=gsv[2];

               graph_pub.header.frame_id = "laser";      ///////                                                                                                                 //////
               graph_pub.vertices.push_back(vert);      ///  adding nodes and edges to graph_slam_visualzier https://github.com/zainmehdi/graph_slam_visualizer by jaejun lee      ///
               graph_pub.edges.push_back(edges);       //////                                                                                                                  //////
               pub_.publish(graph_pub);

               closest_vertex(current_pose);
          //     make_g2o_file();
               i++;

           }

              cerr<<"//////////call fin //////////////" <<endl;

       }
           j++;
    }

    void g2o_initialize()
    {
        cerr << "Initializing g2o..."<<endl;

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1,-1> > SlamBlockSolver;
        typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
        ROS_INFO("Initializing solver");
        SlamLinearSolver* linearSolver = new SlamLinearSolver();
        linearSolver->setBlockOrdering(false);
        SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
        g2o::OptimizationAlgorithmGaussNewton* solverGauss =
                new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
        g2o_graph.setAlgorithm(solverGauss);

        cerr << "INITIALIZATION dONE..."<<endl;



    }

    void adding_vertices_edges() {
    //    cerr << "Adding vertices..." << endl;

        if (i == 1) {

            current_pose = new VertexSE2;
            current_pose->setEstimate(pose_current);
            current_pose->setId(0);
            g2o_graph.addVertex(current_pose);
            prev_pose = current_pose;


                VertexSE2 *firstRobotPose = dynamic_cast<VertexSE2 *>(g2o_graph.vertex(0));

                firstRobotPose->setFixed(true);

               // g2o_graph.setVerbose(true);
              //  cerr << "Fixing done..." << endl;
             //   cerr << "/////////////////////////////" << endl;


        }

        if (i > 1) {
            current_pose = new VertexSE2;
            current_pose->setId(i);
            current_pose->setEstimate(pose_current);
            g2o_graph.addVertex(current_pose);

            odom_edge=new EdgeSE2;
            odom_edge->setVertex(0,prev_pose);
            odom_edge->setVertex(1,current_pose);
            prev_pose = current_pose;
            odom_edge->setMeasurement(pose_current);
            odom_edge->setInformation(m);
            g2o_graph.addEdge(odom_edge);

        }





    }


    void make_g2o_file()          // Function that creates output g2o file
    {
      //  if (i>40)
        {
             cerr<<"Making file"<<endl;
            cerr<<"/////////////////////////////////";
             g2o_graph.save("my_first.g2o");
        }
    }

    void optimize()             // Optimization
    {

        cerr << "Optimizing" << endl;
     //   cerr << "///////////////////////////////////" << endl;
        g2o_graph.initializeOptimization();
        g2o_graph.optimize(10);
     //   cerr << "done." << endl;



    }

    void closest_vertex(VertexSE2 *latest)               // this function returns closest vertex wrt to the current vertex skipping "n" vertices
    {
        double buffer_latest[3];
        latest->getEstimateData(buffer_latest);
        double x1=buffer_latest[0];
        double y1=buffer_latest[1];

        if(g2o_graph.vertices().size() >5) {             // raw form value of n has been set to 5


            distances_bw_vertices.distance_all.resize(g2o_graph.vertices().size());

            for (auto it = g2o_graph.vertices().begin(); it != g2o_graph.vertices().end(); it++) {
                int gh=it->first;
               if(it->first<g2o_graph.vertices().size()-5 && it->first!=latest->id()) {
                   VertexSE2 *temp = static_cast<g2o::VertexSE2 *>(it->second);
                   double buffer[3];
                   temp->getEstimateData(buffer);
                   double x2 = buffer[0];
                   double y2 = buffer[1];

                   temp_distance_store.distance=sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
                   temp_distance_store.from=latest->id();
                   temp_distance_store.to=it->first;


                   distances_bw_vertices.distance_all[it->first]=temp_distance_store;

                 //  ROS_INFO("Vertex ID %d", distances_bw_vertices.distance_all);
                //  ROS_INFO("All ids ID=%d %d SERVICE FINISHED ", it->first, l++);
               }
            }



            int minimum_keyframe_index = 0;
            for (int i = 0; i < distances_bw_vertices.distance_all.size()-5; i++) {

                    if (distances_bw_vertices.distance_all[i].distance <
                        distances_bw_vertices.distance_all[minimum_keyframe_index].distance) {
                        minimum_keyframe_index = distances_bw_vertices.distance_all[i].to;

                    }

              //  ROS_INFO("vertex ID %d",distances_bw_vertices.distance_all[i].to);

           }

           ROS_INFO("CLOSEST KEYFRAME ID=%d ", distances_bw_vertices.distance_all[minimum_keyframe_index].to);
           ROS_INFO("BASE KEYFRAME ID=%d ", latest->id());


        }


    }





private:
    ros::NodeHandle nodeHandle;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    SparseOptimizer g2o_graph;
    graph_slam_visualizer::VertexSE2 vert;
    graph_slam_visualizer::EdgeSE2 edges;
    graph_slam_visualizer::GraphSE2 graph_pub;
    g2o::SE2 new_pose;
    g2o::SE2 pose_current;
    g2o::SE2 pose_previous;
    VertexSE2 *prev_pose;
    VertexSE2 *current_pose;
    EdgeSE2 * odom_edge;
    int i;  // Counter for node index
    int j;  // Counter for sampling
    double gsv[3];
    Eigen::DiagonalMatrix<double,3> m;
    Eigen::Vector3d k;
    g2o_playground::distance temp_distance_store;
    g2o_playground::distance_vector distances_bw_vertices;




};





int main(int argc, char **argv)
{
    ros::init(argc,argv,"SubscribeAndPublish");

    SubscribeAndPublish subpub;

    ros::spin();


}