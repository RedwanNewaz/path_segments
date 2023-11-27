#include <iostream>
#include <vector>
#include <random>
#include "conflict_based_decomposer.h"
#include "rapidcsv.h"
#define RANDOM_INIT

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

int main() {
    std::cout << "Hello, World!" << std::endl;
    int numAgents = 4;
    int lower = 0;
    int upper = 50;

    rapidcsv::Document pathData("../test/coords.csv", rapidcsv::LabelParams(-1, -1));

    // create path variable
    PathDecomposition::PATH path;
    std::vector<int> X, Y;
    X = pathData.GetColumn<int>(0);
    Y = pathData.GetColumn<int>(1);
    for (int i = 0; i < X.size(); ++i) {
        path.emplace_back(std::make_pair(X[i], Y[i]));
    }

    // decompose path into num agents
    ConflictBasedDecomposer decomposer(numAgents, path);
    PathDecomposition::PATH agents;
#ifndef RANDOM_INIT
    std::vector<std::pair<int, int>>cand{
            {27, 46},
            {28, 34},
            {11, 26},
            {49, 25}
    };
#else
    std::vector<std::pair<int, int>>cand;
#endif
    for (int i = 0; i < numAgents; ++i) {
#ifdef RANDOM_INIT
        cand.push_back(sampleCoord(lower, upper));
#endif
        printf("[Init]: a%d (%d, %d) \n", i, cand[i].first, cand[i].second);
        agents.emplace_back(cand[i]);
    }

    decomposer.computeCost(agents);
    return 0;
}
