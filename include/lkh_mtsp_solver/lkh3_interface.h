#ifndef _LKH3_INTERFACE_H
#define _LKH3_INTERFACE_H

#include <algorithm>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <unordered_map>

extern "C" {
#include "LKH.h"
#include "Genetic.h"
#include "BIT.h"
}




class MTSP
{
public:
    MTSP(const char* input_file);
    bool solve();
    std::vector<int> getAssignment(int agent_id);
    int getNumAgents() const;

private:
    const char* input_file_;
    std::unordered_map<int, std::vector<int>> solution_;
    int solveMTSPWithLKH3(const char* input_file);
    std::vector<int> parseSolution(const std::string& output_file) const;

};


#endif