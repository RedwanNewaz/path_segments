#include <iostream>
#include <vector>
#include <random>
#include "conflict_based_decomposer.h"
#include "rapidcsv.h"
#include <sstream>
#include <iostream>
#include <fstream>
//#define RANDOM_INIT

std::pair<int, int> sampleCoord(int lower, int upper)
{
    // Seed for the random number generator
    std::random_device rd;
    std::mt19937 generator(rd());

    // Define the range for x and y coordinates
    int minX = lower;
    int maxX = upper;
    int minY = lower;
    int maxY = upper;

    // Generate random x and y coordinates
    std::uniform_int_distribution<int> distributionX(minX, maxX);
    std::uniform_int_distribution<int> distributionY(minY, maxY);

    int randomX = distributionX(generator);
    int randomY = distributionY(generator);
    return std::make_pair(randomX, randomY);
}

void save_results(const ConflictBasedDecomposer& decomposer, int numAgents)
{
    for (int i = 0; i < numAgents; ++i) {
        auto path = decomposer.getAgentPath(i);
        std::stringstream ss;
        for(const auto&p: path)
        {
            ss << p.first << "," << p.second << "\n";
        }
        std::ofstream myfile;
        myfile.open ("../results/" + std::to_string(i + 1) + ".csv");
        myfile << ss.str();
        myfile.close();

    }
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    int numAgents = 4;
    int lower = 0;
    int upper = 50;

    rapidcsv::Document pathData("../test/xy.csv", rapidcsv::LabelParams(-1, -1));

    // create path variable
    PathDecomposition::PATH path;
    std::vector<double> X, Y;
    X = pathData.GetColumn<double>(0);
    Y = pathData.GetColumn<double>(1);
    for (int i = 0; i < X.size(); ++i) {
        path[i] = std::make_pair(X[i], Y[i]);
    }

    // decompose path into num agents
    ConflictBasedDecomposer decomposer(numAgents, path);
    PathDecomposition::PATH agents;
#ifndef RANDOM_INIT
    std::vector<std::pair<double, double>>cand{
            {27, 46},
            {28, 34},
            {11, 26},
            {49, 25}
    };
#else
    std::vector<std::pair<double, double>>cand;
#endif
    for (int i = 0; i < numAgents; ++i) {
#ifdef RANDOM_INIT
        cand.push_back(sampleCoord(lower, upper));
#endif
        printf("[Init]: a%d (%lf, %lf) \n", i, cand[i].first, cand[i].second);
        agents[i] = cand[i];
    }

    decomposer.computeCost(agents);


    return 0;
}
