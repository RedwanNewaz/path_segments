//
// Created by redwan on 11/27/23.
//
#include "conflict_based_decomposer.h"

ConflictBasedDecomposer::ConflictBasedDecomposer(int numAgents,  PATH &path, bool verbose) :
PathDecomposition(numAgents, path){
    verbose_ = verbose;
}


void ConflictBasedDecomposer::findAssignment()
{
    do{
        auto costMatrix = getCostMatrix();
        NextBestAssignment<std::string, std::string> assignment;
        for(auto& ii: costMatrix)
        {
            for(auto& jj : costMatrix[ii.first])
            {
                assignment.setCost("a" + std::to_string(ii.first),  std::to_string(jj.first), jj.second);
            }
        }
        assignment.solve();

        try {
            auto solution = resolveConflicts(assignment);
            update(solution);
        } catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
            break;
        }

    }while((path_.size() - visited_.size()) >  0);

    std::cout << "[Unassigned] - locations: " << path_.size() - visited_.size() << std::endl;

    for (int a = 0; a < numAgents_; ++a) {
        std::cout << "[Assigned] - pathSize: " << chunks_[a].size() << std::endl;
    }
}


ConflictBasedDecomposer::COSTMAT ConflictBasedDecomposer::getCostMatrix() const
{
    int i = 0;
    std::unordered_map<int, std::vector<std::pair<int, double>>> costMatrix;
    for(const auto& agent: current_)
    {
        int j = 0;
        for (const auto& p: path_)
        {
            if(visited_.count(p.first) == 0)
            {
                double d  = parentCost_[i] + distance(agent.second.first, agent.second.second, p.second.first, p.second.second);
                costMatrix[i].emplace_back(j, d);
            }
            ++j;
        }
        ++i;
    }
    return costMatrix;
}

std::map<std::string, std::string>  ConflictBasedDecomposer::resolveConflicts(NextBestAssignment<std::string, std::string>& assignment)
{
    std::vector<int>removeIndexes;
    std::map<std::string, std::string> solution;

    double c;
    size_t loop = 0;

    do
    {
        removeIndexes.clear();
        c = assignment.nextSolution(solution);
        int k = 0;
        for (const auto& s : solution) {
            int index = std::stoi(s.second);
            prevCoord_[k] = current_[s.first];
            current_[s.first] = path_[index];
            newCoord_[k] = path_[index];
            removeIndexes.push_back(index);
            ++k;
        }
        // Check a condition to throw an exception
        if (++loop > MAX_ITERATION) {
            throw std::runtime_error("Maximum number of iterations reached");
        }

    }while(!moveToTargets(prevCoord_, newCoord_));

    for(const auto& index:removeIndexes)
        visited_.insert(index);

//    eraseByIndices(removeIndexes);

    if(verbose_)
    {
        std::cout << "  + pathSize: " << path_.size() - visited_.size() << std::endl;
        std::cout << "  - cost: " << c << std::endl;
        std::cout << "    assignment:" << std::endl;
    }

    return solution;
}


bool ConflictBasedDecomposer::moveToTargets(const std::vector<COORD>& prevCoord, const std::vector<COORD>& newCoord)
{
    auto toState = [&](const COORD& coord)
    {
        return std::array<double, 3>{(double)coord.first, (double)coord.second, M_PI_2};
    };

    int N = prevCoord.size();
    // check same final goals


    for(int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j) {
            if(i != j)
            {
                auto ri_start = toState(prevCoord.at(i));
                ri_start[2] = headingAngles_[i];
                auto ri_goal = toState(newCoord.at(i));
                ri_goal[2] = relAngle(prevCoord.at(i), newCoord.at(i));
                auto ri = collision_.getCCO(ri_start, ri_goal);

                auto rj_start = toState(prevCoord.at(j));
                ri_start[2] = headingAngles_[j];
                auto rj_goal = toState(newCoord.at(j));
                rj_goal[2] = relAngle(prevCoord.at(j), newCoord.at(j));
                auto rj = collision_.getCCO(rj_start, rj_goal);
                if(collision_.collide(ri, rj))
                {
                    return false;
                }
            }
        }
    }
    return true;

}

void ConflictBasedDecomposer::setGeom(const std::array<double, 3> &param) {
    collision_.setGeom(param);
}
