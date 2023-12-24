#pragma once 
#include <yaml-cpp/yaml.h>
#include <fmt/format.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include "rapidcsv.h"

class InputGenerator
{
public:
    InputGenerator(const char * inpFile);
    std::string get_parameter_file();

private:
    int DIM;
    YAML::Node config;
    std::string atsp_file;
    std::string par_file;

protected:
    std::string create_atsp_file();
    std::string create_par_file();
    double get_distance(double x1, double y1, double x2, double y2);
    void save_file(const std::string& filename, const std::string& data);

};