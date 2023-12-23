#include <iostream>
#include <boost/program_options.hpp>
#include "conflict_based_decomposer.h"
#include "rapidcsv.h"
#include <fstream>
#include <sstream>
#define debug(x) std::cout << x << std::endl

void save_results(const ConflictBasedDecomposer& CBD, int numAgents, const std::string& outdir)
{
    for (int i = 0; i < numAgents; ++i) {
        auto path = CBD.getAgentPath(i);
        std::stringstream ss;
        for(const auto&p:path)
            ss << p.first << "," << p.second << "\n";
        std::ofstream  myfile;
        myfile.open(outdir + std::to_string(i + 1) + ".csv");
        myfile << ss.str();
        myfile.close();
    }
}

int main(int argc, char *argv[])
{
    int numAgents = 4;
    PathDecomposition::PATH coords, agents;
    std::string inp_file = "../input/pvis.csv";
    std::string out_file = "../results/";

    rapidcsv::Document doc(inp_file, rapidcsv::LabelParams(-1, -1));
    for(int i = 0; i < doc.GetRowCount(); ++i)
        coords[i] = std::make_pair(doc.GetCell<double>(0, i), doc.GetCell<double>(1, i));
    ConflictBasedDecomposer CBD(numAgents, coords);
    std::vector<double> visibility = doc.GetColumn<double>(2);


    // load init points
     std::vector<std::pair<double, double>>cand{
            {29.8047, 23.8687},
            {38.1579, 33.8386},
            {29.7511, 41.9475},
            {42.284, 40.4879}
    };

     for(int j = 0; j < cand.size(); ++j)
         agents[j] = cand[j];
    CBD.computeCost(agents, visibility);
    save_results(CBD, numAgents, out_file);
    return 0;
}
