#include <iostream>
#include <vector>
#include <iterator>
#include <fstream>
#include "include/lkh_mtsp_solver/lkh3_interface.h"
#include "include/lkh_mtsp_solver/input_generator.h"



int main(int argc, char *argv[])
{
    assert(argc > 0 && "No parameter file found!");
    InputGenerator ig(argv[1]);
    auto file = ig.get_parameter_file();
    fmt::print("\n\n {} \n", file);
    const char *par = file.c_str();
    MTSP mtsp(par);

    YAML::Node node;
    

    if(mtsp.solve())
    {
        for (int i = 0; i < mtsp.getNumAgents(); ++i) {
            auto assignment = mtsp.getAssignment(i);
            for (size_t j = 1; j < assignment.size(); j++)
            {
                if(!ig.isValidEdge(assignment[j], assignment[j-1]))
                {
                    std::cerr << "not a valid edge (" << j-1 << " -> " << j <<" )" << std::endl;
                    assignment.erase(assignment.begin() + j-1);
                }
            }
            
            
            node["robot" + std::to_string(i+1)] = assignment;
            node["robot" + std::to_string(i+1)].SetStyle(YAML::EmitterStyle::Flow);
    
            // std::cout <<  i + 1 << ": [";
            // std::copy(assignment.begin(), assignment.end(), std::ostream_iterator<int>(std::cout, ", "));
            // std::cout << "]" << std::endl;
        }
    }

    node["NUM_ROBOTS"] = mtsp.getNumAgents();
   

    std::cout << node << std::endl;

    YAML::Emitter out;
    out << node;
    out << YAML::EndSeq;
    std::ofstream myfile;
    myfile.open ("result.yaml");
    myfile << out.c_str();
    myfile.close();

    return 0;
}
