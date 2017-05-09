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
    #include <g2o/types/slam2d/se2.h>
    #include <g2o/types/slam2d/types_slam2d.h>
    #include <tf/transform_datatypes.h>
    #include <geometry_msgs/Quaternion.h>
    #include <ros/subscriber.h>
    #include "pose_cov_ops/pose_cov_ops.h"






    #define foreach BOOST_FOREACH

    using namespace g2o;
    using namespace std;
    using namespace mrpt::poses;
    using namespace mrpt::math;

    ros::Publisher pub_graph_vis;
    ros::Subscriber sub_;

    int i;
    g2o::SE2 increment;
    g2o::SE2 vertex_current;
    g2o::SE2 vertex_last;

    geometry_msgs::Quaternion quat;
    tf::Quaternion quat_tf;
    double roll, pitch, yaw;
    graph_slam_visualizer::GraphSE2 graph_pub;
    graph_slam_visualizer::VertexSE2 vert;
    graph_slam_visualizer::EdgeSE2 edges;
    geometry_msgs::Pose p1,p2,p;




    void callback(const geometry_msgs::Pose2D::ConstPtr &msg)
    //  foreach(rosbag::MessageInstance const m, view)
    {
        // nav_msgs::Odometry::ConstPtr s=m.instantiate<nav_msgs::Odometry>();
        //   geometry_msgs::Pose2D::ConstPtr s=m.instantiate<geometry_msgs::Pose2D>();
        //   if (s != NULL)

        // std::cout <<"position from intel bag"<< s->pose.pose.position << std::endl;

        if (i == 1) {



            //   quat=s->pose.pose.orientation;
            //   tf::quaternionMsgToTF(quat,quat_tf);
            //  tf::Matrix3x3(quat_tf).getRPY(roll,pitch,yaw);
            //  Vector3D poze(s->pose.pose.position.x,s->pose.pose.position.y,yaw);
            Vector3D poze(msg->x, msg->y, msg->theta);

            //  std::cout <<"x: "<< msg->x <<" y: "<< msg->y<<" th: "<< msg->theta<<std::endl;


            increment.fromVector(poze);
            vertex_current = increment;


            vert.x = poze[0];
            vert.y = poze[1];
            vert.theta = poze[2];


            edges.vi_idx = i - 1;
            edges.vj_idx = i;
            vertex_last = vertex_current;


        }

        if (i > 1) {



            //   quat=s->pose.pose.orientation;
            //   tf::quaternionMsgToTF(quat,quat_tf);
            //   tf::Matrix3x3(quat_tf).getRPY(roll,pitch,yaw);
            //  Vector3D poze(s->pose.pose.position.x,s->pose.pose.position.y,yaw);

            Vector3D poze(msg->x, msg->y, msg->theta);


            increment.fromVector(poze);         //converted to se3 form               //new pose
            // vertex_current=increment;        //current vertex in se3 form

            vertex_current = vertex_last.inverse().operator*(increment);          //relative measurement

            //vertex_current = vertex_last.operator*=(increment);

            vertex_current = vertex_current.operator*=(vertex_last);             //absolute measurement

            vertex_last = vertex_current;

            vert.x = vertex_current.toVector()[0];
            vert.y = vertex_current.toVector()[1];
            vert.theta = vertex_current.toVector()[2];

            edges.vi_idx = i - 1;
            edges.vj_idx = i;
            //  std::cout << "x: " << vert.x << "y: " << vert.y << "th: " << vert.theta << std::endl;
            p1 = p2;
            i++;

        }

        graph_pub.header.frame_id = "laser";
        graph_pub.vertices.push_back(vert);
        graph_pub.edges.push_back(edges);
        pub_graph_vis.publish(graph_pub);
        //   std::cout <<"position from node "<< nodes[i] << std::endl;






        //  bag.close();

      //  }
    }





  //  }




    int main (int argc, char ** argv) {

        i = 1;
        ros::init(argc, argv, "bag_reader_panga");
        ros::NodeHandle nh;
        pub_graph_vis = nh.advertise<graph_slam_visualizer::GraphSE2>("zann_graph_se2", 10);
        sub_ = nh.subscribe("/pose2D", 1000, callback);
    //    ros::Rate rate(1);
        //  while (ros::ok()) {

     //   rate.sleep();
      //  rosbag::Bag bag;

      //  std::map<int, graph_slam_visualizer::VertexSE2> nodes;
        // std::map<int,graph_slam_visualizer::EdgeSE2> edges;
        ros::spin();
      //  return 0;
    }

  //  bag.open("/home/zain/test_workspace/bag/myfile_2017-04-21-02-37-10.bag", rosbag::bagmode::Read);

  //  std::vector<std::string> topics;
  //  topics.push_back(std::string("/raw_odom"));
   // topics.push_back(std::string("/pose2D"));


   // rosbag::View view(bag);







