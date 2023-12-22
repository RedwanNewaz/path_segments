//
// Created by redwan on 11/19/23.
//

#ifndef INFOTHEOPLANNER_PATH_DECOMPOSITION_H
#define INFOTHEOPLANNER_PATH_DECOMPOSITION_H
#include <vector>
#include <iostream>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <map>
#include <unordered_set>
#define MAX_ITERATION (30000)


class PathDecomposition{

public:
    using COORD = std::pair<double, double>;
    using PATH = std::map<int, COORD>;
    PathDecomposition(int numAgents,  PATH& path):
            numAgents_(numAgents), path_(path)
    {
        chunks_.resize(numAgents);
        for (int i = 0; i < numAgents; ++i) {
            headingAngles_.push_back(M_PI_2);
        }
        prevCoord_.resize(numAgents);
        newCoord_.resize(numAgents);
        parentCost_.resize(numAgents);
        std::fill(parentCost_.begin(), parentCost_.end(), 0);
    }

    std::vector<COORD> getAgentPath(int index) const
    {
        return chunks_.at(index);
    }

//    size_t getPathLength(const PATH& path) const
//    {
//        size_t totalLen = 0;
//        for (int i = 1; i < path.size(); ++i) {
//            double d = distance(path[i-1].first, path[i-1].second, path[i].first, path[i].second);
//            totalLen += d;
//        }
//        return totalLen;
//    }


    void computeCost(const PATH& initPoses)
    {

        for(const auto& agent: initPoses)
        {
            int i = agent.first;
            current_["a" + std::to_string(i)] = agent.second;
            chunks_[i].emplace_back(agent.second);
        }
        findAssignment();
    }

protected:

    void update(std::map<std::string, std::string>& solution)
    {
        int k = 0;
        for (const auto& s : solution)
        {
            int index = std::stoi(s.second);
            chunks_[k++].emplace_back(current_[s.first]);
            if(verbose_)
            {
                std::cout << "      " << s.first  << ": t(" ;
                std::cout  <<index<< ")\n" ;
            }
        }
        // update parent cost and heading angle
        for(int i = 0; i < prevCoord_.size(); ++i)
        {
            double d = distance(prevCoord_[i].first, prevCoord_[i].second, newCoord_[i].first, newCoord_[i].second);
            parentCost_[i] += d;
            headingAngles_[i] = relAngle(prevCoord_[i], newCoord_[i]);
        }
    }

    double relAngle(const COORD& a, const COORD& b) const
    {
        double dx = b.first - a.first;
        double dy = b.second - a.second;
        return atan2(dy, dx);
    }

    double distance(double x1, double y1, double x2, double y2) const
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
//        return std::abs(x2 - x1) + std::abs(y2 - y1);
        return sqrt(dx * dx + dy * dy);
    }



    // Function to erase elements from a vector by indices
    void eraseByIndices(const std::vector<int>& indices) {
        // Sort the indices in descending order to avoid invalidation issues
//        std::vector<int> sortedIndices = indices;
//        std::sort(sortedIndices.rbegin(), sortedIndices.rend());

        // Erase elements by indices
        for (const auto& index : indices) {
            if (path_.find(index) != path_.end()) {
//                vec.erase(vec.begin() + index );
//                auto it = path_.find(index);
                path_.erase(index);
            } else {
                std::cerr << "Index " << index << " is out of range." << std::endl;
            }
        }
    }

    virtual void findAssignment() = 0;

    bool isValidAssignment() const
    {
        bool validAssignment = true;
        for(auto check: {prevCoord_, newCoord_})
            for (int i = 0; i <= numAgents_; ++i) {
                auto a = check[(i +1) % numAgents_];
                auto b = check[i % numAgents_];
                if(distance(a.first, a.second, b.first, b.second) < 1)
                {
                    validAssignment = false;
                    break;
                }
            }

        if(!validAssignment)
        {
            std::cerr << "  + invalid assignment: " << path_.size() << std::endl;
        }
        return validAssignment;
    }


protected:
    PATH path_;
    int numAgents_;
    std::map<std::string, COORD> current_;
    std::vector<std::vector<COORD>> chunks_;
    bool verbose_;

    std::vector<double>headingAngles_;
    std::vector<COORD> prevCoord_, newCoord_;
    std::vector<double>parentCost_;
    std::unordered_set<int> visited_;
};


#endif //INFOTHEOPLANNER_PATH_DECOMPOSITION_H
