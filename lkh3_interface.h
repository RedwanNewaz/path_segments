#ifndef _LKH3_INTERFACE_H
#define _LKH3_INTERFACE_H
#include <algorithm>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>

extern "C" {
  #include "LKH.h"
  #include "Genetic.h"
  #include "BIT.h"
}
class MTSP
{
public:
    MTSP(const char* input_file);
    bool success() const;
private:
    bool sucess_;
    std::string outputFile_;
    int solveMTSPWithLKH3(const char* input_file);
    std::vector<int> parseSolution() const;
};


#endif