#include <iostream>
#include <iterator>
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
    if(mtsp.solve())
    {
        for (int i = 0; i < mtsp.getNumAgents(); ++i) {
            auto assingment = mtsp.getAssignment(i);
            std::cout <<  i + 1 << ": [";
            std::copy(assingment.begin(), assingment.end(), std::ostream_iterator<int>(std::cout, ", "));
            std::cout << "]" << std::endl;
        }
    }

    return 0;
}
