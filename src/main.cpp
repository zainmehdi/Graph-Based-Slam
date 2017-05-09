//
// Created by zain on 3/23/17.
//

#include <ros/ros.h>
#include <graph_slam_visualizer/GraphSE2.h>
#include <g2o/core/factory.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/block_solver.h>
#include<iostream>

#include <vector>

#include <ros/publisher.h>
ros::Publisher pub_graph_vis;
G2O_USE_TYPE_GROUP(slam2d);
using namespace std;

void publishGraph(boost::shared_ptr<g2o::SparseOptimizer> g2o_graph);

int main(int argc, char** argv){
    ros::init(argc, argv, "graph_visualize_test");

    if(argc!=2){
        printf("give me g2o file name\n");
        return 0;
    }
    ros::NodeHandle nh;

    pub_graph_vis = nh.advertise<graph_slam_visualizer::GraphSE2>("zann_graph_se2", 10);

    boost::shared_ptr<g2o::SparseOptimizer> g2o_graph(new g2o::SparseOptimizer());

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1,-1> > SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    ROS_INFO("Initializing solver");
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmGaussNewton* solverGauss =
            new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
    g2o_graph->setAlgorithm(solverGauss);

    ROS_INFO("Loading g2o file");
    std::cout<<argv[1]<<std::endl;
    if(!g2o_graph->load(argv[1])){
        ros::shutdown();
        return 0;
    }
    if(!g2o_graph->initializeOptimization()){
        ROS_ERROR("Failed initializing optimization");
        ros::shutdown();
        return 0;
    }

  //  g2o_graph->optimize(10);
    publishGraph(g2o_graph);
    ros::Rate rate(1);
    rate.sleep();
    ROS_INFO("Start publishing");
    while(ros::ok()){
    //   g2o_graph->optimize(10);
        publishGraph(g2o_graph);
        rate.sleep();
    }

    return 0;
}

void publishGraph(boost::shared_ptr<g2o::SparseOptimizer> g2o_graph){
    if(!pub_graph_vis.getNumSubscribers())
        return;
    graph_slam_visualizer::GraphSE2Ptr graph_msg(new graph_slam_visualizer::GraphSE2());
    graph_msg->header.frame_id = "world";
    graph_msg->header.stamp = ros::Time::now();
    std::map<int, int> id_to_idx;
    double buffer[3];
    int cnt = 0;

    size_t sz = g2o_graph->vertices().size();
    /////////////////////////////////////////////
    cout<<"number of vertices "<<sz;
    ////////////////////////////////////////////
    graph_msg->vertices.reserve(g2o_graph->vertices().size());
    graph_slam_visualizer::VertexSE2 v;
    graph_slam_visualizer::EdgeSE2 e;

    for(auto it = g2o_graph->vertices().begin();it!=g2o_graph->vertices().end();it++){
        g2o::VertexSE2* pv_g2o = static_cast<g2o::VertexSE2*>(it->second);
        id_to_idx[it->first] = cnt;
        ROS_INFO("id to x test = %d",it->first);
        ROS_INFO("count = %d",cnt);
        pv_g2o->getEstimateData(buffer);
        v.x = buffer[0];
        v.y = buffer[1];
        v.theta = buffer[2];
        graph_msg->vertices.push_back(v);
        cnt++;
    }

    graph_msg->edges.reserve(g2o_graph->edges().size());
    graph_msg->edge_weights.reserve(g2o_graph->edges().size());
    Eigen::Vector3d scale;
    double chi2;
    for(const auto& p_edge:g2o_graph->edges()){
        g2o::EdgeSE2 * e_se2 = dynamic_cast<g2o::EdgeSE2*>(p_edge);
        assert(e_se2 != nullptr);

        e.vi_idx = id_to_idx[e_se2->vertex(0)->id()];
        e.vj_idx = id_to_idx[e_se2->vertex(1)->id()];
        ROS_INFO("printing value of vertex ids i = %d",e_se2->vertex(0)->id());
        ROS_INFO("printing value of vertex ids j = %d",e_se2->vertex(1)->id());
        ROS_INFO("id_to_idx vi %d",id_to_idx[e_se2->vertex(0)->id()]);
        ROS_INFO("id_to_idx vj %d",id_to_idx[e_se2->vertex(1)->id()]);
        if(e_se2->robustKernel()){
            chi2 = e_se2->chi2();
            e_se2->robustKernel()->robustify(chi2, scale);
            graph_msg->edge_weights.push_back(scale[0] / chi2);
        }else{
            graph_msg->edge_weights.push_back(1.0f);
        }

        graph_msg->edges.push_back(e);
    }

    pub_graph_vis.publish(graph_msg);

}
