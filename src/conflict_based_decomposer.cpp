//
// Created by redwan on 11/27/23.
//
#include "conflict_based_decomposer.h"

ConflictBasedDecomposer::ConflictBasedDecomposer(int numAgents,  PATH &path, bool verbose) :
PathDecomposition(numAgents, path){
    verbose_ = verbose;
    std::string inp_file = "../input/adjmatrix2.csv";
    adj_ = new rapidcsv::Document(inp_file, rapidcsv::LabelParams(-1, -1));
}

ConflictBasedDecomposer::~ConflictBasedDecomposer()
{
    delete adj_;
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
            if(solution.size() == 0)
                break;
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


ConflictBasedDecomposer::COSTMAT ConflictBasedDecomposer::getCostMatrix()
{
    int i = 0;
    std::unordered_map<int, std::vector<std::pair<int, double>>> costMatrix;
    for(const auto& agent: current_)
    {
        int j = 0;
        for (const auto& p: path_)
        {
            double d = 1000.0;
            if(visited_.count(p.first) == 0 )
            {
                double d  =  parentCost_[i] + distance(agent.second.first, agent.second.second, p.second.first, p.second.second);
                costMatrix[i].emplace_back(j, 1000.0 * d/(visibility_[i] + visibility_[j]));
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
    int solSize;
    do
    {
        removeIndexes.clear();
        c = assignment.nextSolution(solution);
        int k = 0;
        for (const auto& s : solution) {
            int index = std::stoi(s.second);
            prevCoord_[k] = current_[s.first];
            newCoord_[k] = path_[index];
            removeIndexes.push_back(index);
            ++k;
        }
        // Check a condition to throw an exception
        if (++loop > MAX_ITERATION) {
            throw std::runtime_error("Maximum number of iterations reached");
        }

    }while(!moveToTargets(prevCoord_, newCoord_));
    std::cout << "sol size " << solSize << " loop count " << loop << std::endl;
    int rid = 0;
    for(const auto& index:removeIndexes) {
        visited_.insert(index);
        current_["a" + std::to_string(rid++)] = path_[index];
    }

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
        auto ri_start = toState(prevCoord.at(i));
        ri_start[2] = headingAngles_[i];
        auto ri_goal = toState(newCoord.at(i));
        ri_goal[2] = relAngle(prevCoord.at(i), newCoord.at(i));

        if(collision_.isStaticCollision(ri_start, ri_goal))
        {
            // back track
            auto path = getAgentPath(i);
            auto agentID = "a" + std::to_string(i);
            auto coord = current_[agentID];
            auto it = std::find(path.begin(), path.end(), coord);
            --it;
            current_[agentID] = *it;
            return false;
        }
    }
    return true;

}

void ConflictBasedDecomposer::setGeom(const std::array<double, 3> &param) {

}
