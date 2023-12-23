//
// Created by airlab on 11/26/23.
//

#ifndef GOAL_FREE_PLANNER_COLLISION_CHECKER_H
#define GOAL_FREE_PLANNER_COLLISION_CHECKER_H
//#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/continuous_collision.h"
#include "polygonal_obstacle.h"

class DynamicCollisionChecker
{
public:
    DynamicCollisionChecker()
    {
        geom_ = std::make_shared<fcl::Boxd>(0.5, 0.5, 0.1);
    }

    void setGeom(const std::array<double, 3>& param)
    {
        geom_ = std::make_shared<fcl::Boxd>(param[0], param[1], param[2]);
    }

    fcl::ContinuousCollisionObjectd *getCCO(const std::array<double, 3>& start, std::array<double, 3>& goal)
    {
        std::vector<fcl::Vector3d> t(4), r(4);
        populate_translation(start, goal, t);
        populate_rotation(start[2], goal[2], r);

        auto motion_a = fcl::make_aligned_shared<fcl::SplineMotion<double>>(
                t[0], t[1], t[2], t[3],
                        r[0], r[1], r[2], r[3]);
        return new fcl::ContinuousCollisionObjectd(geom_, motion_a);
    }

    bool collide(const fcl::ContinuousCollisionObjectd * selfCCO, const fcl::ContinuousCollisionObjectd * otherCCO) const
    {

        // result will be returned via the continuous collision result structure
        fcl::ContinuousCollisionResultd result;

        fcl::ContinuousCollisionRequestd request;
        // perform continuous collision test
        request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
        request.gjk_solver_type = fcl::GST_LIBCCD;
        fcl::collide(selfCCO, otherCCO, request, result);
        return result.is_collide;
    }

protected:
    static void populate_translation(const std::array<double, 3>& start1, const std::array<double, 3> &goal1, std::vector<fcl::Vector3d>& t)
    {
        double dx1 = goal1[0] - start1[0];
        double dy1 = goal1[1] - start1[1];
        t[0] = fcl::Vector3d(start1[0], start1[1], 0);
        t[1] = fcl::Vector3d(start1[0] + 0.33 * dx1, start1[1] + 0.33 * dy1, 0);
        t[2] = fcl::Vector3d(start1[0] + 0.66 * dx1, start1[1] + 0.66 * dy1, 0);
        t[3] = fcl::Vector3d(goal1[0], goal1[1], 0);
    }

    static void populate_rotation(double start, double goal, std::vector<fcl::Vector3d>& r)
    {
        double delta = goal - start;
        r[0] = fcl::Vector3d(0, 0, start);
        r[1] = fcl::Vector3d(0, 0, start + 0.33 * delta);
        r[2] = fcl::Vector3d(0, 0, start + 0.66 * delta);
        r[3] = fcl::Vector3d(0, 0, goal);
    }
private:
    std::shared_ptr<fcl::Boxd> geom_;

};

class CollisionChecker
{
public:
    CollisionChecker()
    {
        dcc = new DynamicCollisionChecker();
        scc = new Polygon("../input/obstacles/allObs.obj");
        robot = new Polygon("../input/robot.obj");
        env_obj_ = scc->getCollisionObj();
    }
    virtual ~CollisionChecker()
    {

        delete env_obj_;
        delete scc;
        delete dcc;
        delete robot;
    }
    bool isStaticCollision(const std::array<double, 3>& start_i, std::array<double, 3>& goal_i) const
    {
        Eigen::VectorXd x1(5), g1(2);
        x1 << start_i[0], start_i[1], start_i[2], 0, 0;
        g1 << goal_i[0], goal_i[1];

        return checkStaticCollision(x1, g1);
    }
    bool isDynamicCollision(const std::array<double, 3>& start_i, std::array<double, 3>& goal_i, const std::array<double, 3>& start_j, std::array<double, 3>& goal_j)
    {
        auto ri = dcc->getCCO(start_i, goal_i);
        auto rj = dcc->getCCO(start_j, goal_j);
        return dcc->collide(ri, rj);
    }
private:
    DynamicCollisionChecker *dcc;
    Polygon *scc;
    Polygon *robot;
    fcl::CollisionObjectd *env_obj_;
protected:
    bool checkStaticCollision(const Eigen::VectorXd& x, const Eigen::Vector2d& g) const
    {
        Eigen::Vector2d x0;

        x0 << x(0), x(1);
        auto dx = g - x0;
        const double robotSize = 0.5;
        if(dx.norm() < robotSize)
            return false;

        auto traj = robot->genTraj(x, g, robotSize);
        for(int t = 0; t < traj.rows(); ++t)
        {
            auto state = traj.row(t);
            auto robot_obj_ = robot->getCollisionObj(state);
            // set the collision request structure, here we just use the default setting
            fcl::CollisionRequestd request;
            // result will be returned via the collision result structure
            fcl::CollisionResultd result;
            // perform collision test
            fcl::collide(env_obj_, robot_obj_, request, result);
            if(result.isCollision())
                return true;
        }

        return false;
    }
};

#endif //GOAL_FREE_PLANNER_COLLISION_CHECKER_H
