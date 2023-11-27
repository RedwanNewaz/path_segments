//
// Created by redwan on 11/27/23.
//

#ifndef PATH_SEGMENTS_CONFLICT_BASED_DECOMPOSER_H
#define PATH_SEGMENTS_CONFLICT_BASED_DECOMPOSER_H
#include "path_decomposition.h"
#include "collision_checker.h"
#include "libMultiRobotPlanning/next_best_assignment.hpp"
using libMultiRobotPlanning::NextBestAssignment;

class ConflictBasedDecomposer: public PathDecomposition
{
public:
    ConflictBasedDecomposer(int numAgents, const PATH &path);

protected:
    void findAssignment() override;
private:
    CollisionChecker collision_;

    std::vector<std::vector<int>> getCostMatrix() const;
    std::map<std::string, std::string> resolveConflicts(NextBestAssignment<std::string, std::string>& assignment);
    bool moveToTargets(const std::vector<COORD>& prevCoord, const std::vector<COORD>& newCoord);

};
#endif //PATH_SEGMENTS_CONFLICT_BASED_DECOMPOSER_H
