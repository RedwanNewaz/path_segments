//
// Created by redwan on 12/21/23.
//

#ifndef VISIBILITY_TASK_PLANNING_POLYGONAL_OBSTACLE_H
#define VISIBILITY_TASK_PLANNING_POLYGONAL_OBSTACLE_H
#include <iostream>
#include <utility>
#include <vector>
#include <string>
#include "fcl/fcl.h"
#include "fcl/config.h"
#include "motion_model.h"

class Polygon {
    using Model = fcl::BVHModel<fcl::OBBRSSd>;

public:
    Polygon(std::string  filePath):fileName_(std::move(filePath))
    {
        geom_ = getGeom();
    }

    Eigen::MatrixXd genTraj(const Eigen::VectorXd & x, const Eigen::Vector2d & g, double robotSize) const
    {
        Eigen::Vector2d x0, uu;

        x0 << x(0), x(1);
        auto dx = g - x0;
        auto dq = atan2(dx(1), dx(0)) - x(2);
        dq = fmod(dq + M_PI, 2 * M_PI) - M_PI;
        double dist = dx.norm();
        double predTime = dist / robotSize;
        double w = (dq / predTime);
        uu << robotSize, w;
        double dt = 1.0;

        MotionModel motion(5, dt, predTime);
        size_t numSamples = predTime;

        return motion.sample(x, uu, numSamples);
//        return lerp_samples(x, g, numSamples, w);

    }



    fcl::CollisionObjectd *getCollisionObj() const
    {

        // Create FCL collision object
        Eigen::VectorXd ori(5);
        ori << 0.0, 0.0, 0.0, 0.0, 0.0;
        return getCollisionObj(ori);
    }

    fcl::CollisionObjectd* getCollisionObj(const Eigen::VectorXd& x) const
    {
        auto obj_collision = new fcl::CollisionObjectd(
                geom_, Eigen::AngleAxisd(x(2), Eigen::Vector3d::UnitZ()).matrix(),
                Eigen::Vector3d(x(0), x(1), 0.000000));
        return obj_collision;
    }

private:
    Eigen::MatrixXd lerp_samples(const Eigen::VectorXd& x, const Eigen::VectorXd& g, size_t numSamples, double w) const
    {

        Point start(x(0), x(1));
        Point end(g(0), g(1));
        double heading = x(2);
        Eigen::MatrixXd traj (numSamples + 1, 5);
        // Perform interpolation and print the interpolated points
        for (int i = 0; i <= numSamples; ++i) {
            // Calculate the parameter t based on the current step
            double t = static_cast<double>(i) / numSamples;

            // Interpolate between start and end points
            Point interpolatedPoint = lerp(start, end, t);
            Eigen::VectorXd nX(5);
            nX << interpolatedPoint.x, interpolatedPoint.y, heading, 0, 0;
            traj.row(i) = nX;


            // Print the interpolated point coordinates
//            std::cout << "Interpolated Point " << i << ": (" << interpolatedPoint.x << ", " << interpolatedPoint.y << ")\n";
            heading += w;
        }

        return traj;
    }
    std::shared_ptr<Model> getGeom() const
    {
        // Extract mesh data from the loaded scene
        std::vector<fcl::Vector3d> vertices;
        std::vector<fcl::Triangle> triangles;

        // Create FCL BVHModel

        std::shared_ptr<Model> geom = std::make_shared<Model>();
        loadOBJFile(fileName_.c_str(), vertices, triangles);

        // Add the mesh data into the BVHModel structure
        geom->beginModel();
        geom->addSubModel(vertices, triangles);
        geom->endModel();

        return geom;
    }

    std::shared_ptr<Model> geom_;
    const std::string fileName_;
    template <typename S>
    void loadOBJFile(const char* filename, std::vector<fcl::Vector3<S>>& points, std::vector<fcl::Triangle>& triangles) const
    {

        FILE* file = fopen(filename, "rb");
        if(!file)
        {
            std::cerr << "file not exist" << std::endl;
            return;
        }

        bool has_normal = false;
        bool has_texture = false;
        char line_buffer[2000];
        while(fgets(line_buffer, 2000, file))
        {
            char* first_token = strtok(line_buffer, "\r\n\t ");
            if(!first_token || first_token[0] == '#' || first_token[0] == 0)
                continue;

            switch(first_token[0])
            {
                case 'v':
                {
                    if(first_token[1] == 'n')
                    {
                        strtok(nullptr, "\t ");
                        strtok(nullptr, "\t ");
                        strtok(nullptr, "\t ");
                        has_normal = true;
                    }
                    else if(first_token[1] == 't')
                    {
                        strtok(nullptr, "\t ");
                        strtok(nullptr, "\t ");
                        has_texture = true;
                    }
                    else
                    {
                        S x = (S)atof(strtok(nullptr, "\t "));
                        S y = (S)atof(strtok(nullptr, "\t "));
                        S z = (S)atof(strtok(nullptr, "\t "));
                        points.emplace_back(x, y, z);
                    }
                }
                    break;
                case 'f':
                {
                    fcl::Triangle tri;
                    char* data[30];
                    int n = 0;
                    while((data[n] = strtok(nullptr, "\t \r\n")) != nullptr)
                    {
                        if(strlen(data[n]))
                            n++;
                    }

                    for(int t = 0; t < (n - 2); ++t)
                    {
                        if((!has_texture) && (!has_normal))
                        {
                            tri[0] = atoi(data[0]) - 1;
                            tri[1] = atoi(data[1]) - 1;
                            tri[2] = atoi(data[2]) - 1;
                        }
                        else
                        {
                            const char *v1;
                            for(int i = 0; i < 3; i++)
                            {
                                // vertex ID
                                if(i == 0)
                                    v1 = data[0];
                                else
                                    v1 = data[t + i];

                                tri[i] = atoi(v1) - 1;
                            }
                        }
                        triangles.push_back(tri);
                    }
                }
            }
        }
    }

    struct Point {
        double x, y;

        Point(double _x, double _y) : x(_x), y(_y) {}
    };

// Linear interpolation function
    Point lerp(const Point& p1, const Point& p2, double t) const
    {
        return Point(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
    }

// Function to calculate Euclidean distance between two points
    double distance(const Point& p1, const Point& p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }




};
#endif //VISIBILITY_TASK_PLANNING_POLYGONAL_OBSTACLE_H
