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
    using COSTMAT=std::unordered_map<int, std::vector<std::pair<int, double>>>;
public:
    ConflictBasedDecomposer(int numAgents,  PATH &path, bool verbose=true);
    void setGeom(const std::array<double, 3>&param);

protected:
    void findAssignment() override;
private:
    CollisionChecker collision_;


    COSTMAT getCostMatrix() const;
    std::map<std::string, std::string> resolveConflicts(NextBestAssignment<std::string, std::string>& assignment);
    bool moveToTargets(const std::vector<COORD>& prevCoord, const std::vector<COORD>& newCoord);

};
#endif //PATH_SEGMENTS_CONFLICT_BASED_DECOMPOSER_H
