//
// Created by zain on 5/8/17.
//

#include "../include/g2o_playground/graphslam.h"




void SubscribeAndPublish::closest_vertex(g2o::VertexSE2 *latest)               // this function returns closest vertex wrt to the current vertex skipping "n" vertices
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

void SubscribeAndPublish::optimize()             // Optimization
{

    cerr << "Optimizing" << endl;
    //   cerr << "///////////////////////////////////" << endl;
    g2o_graph.initializeOptimization();
    g2o_graph.optimize(10);
    //   cerr << "done." << endl;



}

void SubscribeAndPublish::make_g2o_file()          // Function that creates output g2o file
{
    //  if (i>40)
    {
        cerr<<"Making file"<<endl;
        cerr<<"/////////////////////////////////";
        g2o_graph.save("my_first.g2o");
    }
}


void SubscribeAndPublish::adding_vertices_edges() {
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


void SubscribeAndPublish::g2o_initialize()
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

void SubscribeAndPublish::mega(const geometry_msgs::Pose2D::ConstPtr &msg)
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
