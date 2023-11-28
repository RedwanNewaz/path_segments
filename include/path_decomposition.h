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
#define MAX_ITERATION (3000)


class PathDecomposition{

public:
    using COORD = std::pair<int, int>;
    using PATH = std::vector<COORD>;
    PathDecomposition(int numAgents, const PATH& path):
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

    PATH getAgentPath(int index) const
    {
        return chunks_.at(index);
    }

    size_t getPathLength(const PATH& path) const
    {
        size_t totalLen = 0;
        for (int i = 1; i < path.size(); ++i) {
            double d = distance(path[i-1].first, path[i-1].second, path[i].first, path[i].second);
            totalLen += d;
        }
        return totalLen;
    }


    void computeCost(const std::vector<std::pair<int, int>>& initPoses)
    {
        int i = 0;
        for(const auto& agent: initPoses)
        {
            current_["a" + std::to_string(i)] = agent;
            chunks_[i].emplace_back(agent);
            ++i;
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
                std::cout << "      " << s.first  << ": (" ;
                std::cout  <<path_[index].first  << ", " << path_[index].second << ")\n" ;
            }
        }
        // update parent cost and heading angle
        for(int i = 0; i < prevCoord_.size(); ++i)
        {
            int d = distance(prevCoord_[i].first, prevCoord_[i].second, newCoord_[i].first, newCoord_[i].second);
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

    int distance(int x1, int y1, int x2, int y2) const
    {
        return std::abs(x2 - x1) + std::abs(y2 - y1);
    }



    // Function to erase elements from a vector by indices
    template <typename T>
    void eraseByIndices(std::vector<T>& vec, const std::vector<size_t>& indices) {
        // Sort the indices in descending order to avoid invalidation issues
        std::vector<size_t> sortedIndices = indices;
        std::sort(sortedIndices.rbegin(), sortedIndices.rend());

        // Erase elements by indices
        for (const auto& index : sortedIndices) {
            if (index < vec.size()) {
                vec.erase(vec.begin() + index);
            } else {
                std::cerr << "Index " << index << " is out of range." << std::endl;
            }
        }
    }

    virtual void findAssignment() = 0;


protected:
    PATH path_;
    int numAgents_;
    std::map<std::string, COORD> current_;
    std::vector<PATH> chunks_;
    bool verbose_;

    std::vector<double>headingAngles_;
    std::vector<COORD> prevCoord_, newCoord_;
    std::vector<int>parentCost_;
};


#endif //INFOTHEOPLANNER_PATH_DECOMPOSITION_H
