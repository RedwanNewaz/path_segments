#include <iostream>
#include <iterator>
#include "include/lkh_mtsp_solver/lkh3_interface.h"


int main(int argc, char *argv[])
{
    MTSP mtsp(argv[1]);
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
