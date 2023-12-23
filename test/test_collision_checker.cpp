#include <iostream>
#include <gtest/gtest.h>
#include "polygonal_obstacle.h"


TEST(CollisionTests, CollideChecks) {

    std::vector<fcl::CollisionObjectd*> otherCollisionObject;
    Polygon obstacle("../input/obstacles/allObs.obj");


    // Create a robot
    Polygon robot("../input/robot.obj");
    Eigen::VectorXd x(5), g(2);
//    x << 171.3, 28.7, M_PI_2, 0, 0;
//    g << 174, 87;

    x << 171.1,29, M_PI_2, 0, 0;
    g << 203.4,76.9;

    auto env_obj_ = obstacle.getCollisionObj();
    auto traj = robot.genTraj(x, g, 0.5);
    bool collide = false;
    for(int t = 0; t < traj.rows(); ++t)
    {
        auto state = traj.row(t);
        auto robot_obj_ = robot.getCollisionObj(state);
        // set the collision request structure, here we just use the default setting
        fcl::CollisionRequestd request;
        // result will be returned via the collision result structure
        fcl::CollisionResultd result;
        // perform collision test
        fcl::collide(env_obj_, robot_obj_, request, result);
        collide = collide || result.isCollision();

    }

    ASSERT_TRUE(collide);

}



TEST(CollisionTests, FreeChecks) {

    std::vector<fcl::CollisionObjectd*> otherCollisionObject;
    Polygon obstacle("../input/obstacles/allObs.obj");


    // Create a robot
    Polygon robot("../input/robot.obj");
    Eigen::VectorXd x(5), g(2);
    x << 172.6, 85, M_PI_2, 0, 0;
    g << 202, 75;
//    double angle = atan2(117 - 140, 233 - 62);
//    x << 62, 140, angle, 0, 0;
//    g << 233, 117;
    auto env_obj_ = obstacle.getCollisionObj();
    auto traj = robot.genTraj(x, g, 0.5);
    bool collide = false;
    for(int t = 0; t < traj.rows(); ++t)
    {
        auto state = traj.row(t);
        auto robot_obj_ = robot.getCollisionObj(state);
        // set the collision request structure, here we just use the default setting
        fcl::CollisionRequestd request;
        // result will be returned via the collision result structure
        fcl::CollisionResultd result;
        // perform collision test
        fcl::collide(env_obj_, robot_obj_, request, result);
        collide = collide || result.isCollision();
    }

    ASSERT_FALSE(collide);

}

TEST(CollisionTests, DistChecks)
{
    std::vector<fcl::CollisionObjectd*> otherCollisionObject;
    Polygon obstacle("../input/obstacles/allObs.obj");


    // Create a robot
    Polygon robot("../input/robot.obj");


    Eigen::VectorXd x(5), g(2);
    x << 171.3, 28.7, M_PI_2, 0, 0;
    g << 174, 87;

    auto env_obj_ = obstacle.getCollisionObj();
    auto traj = robot.genTraj(x, g, 0.5);
    double minDist = std::numeric_limits<double>::max();
    for(int t = 0; t < traj.rows(); ++t)
    {
        auto state = traj.row(t);
//        std::cout << state;
        auto robot_obj_ = robot.getCollisionObj(state);
        // set the distance request structure, here we just use the default setting
        fcl::DistanceRequestd request_;
        // result will be returned via the collision result structure
        fcl::DistanceResultd result_;
        distance(env_obj_, robot_obj_, request_, result_);
        minDist = std::min(result_.min_distance, minDist);
//        std::cout << " " << result_.min_distance << std::endl;
    }

    ASSERT_TRUE(minDist < 0.5);
}