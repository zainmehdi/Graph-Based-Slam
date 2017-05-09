//
// Created by zain on 5/9/17.
//

#include "../include/g2o_playground/graphslam.h"


int main(int argc, char **argv)
{
    ros::init(argc,argv,"SubscribeAndPublish");

    SubscribeAndPublish subpub;

    ros::spin();

    return 0;
}