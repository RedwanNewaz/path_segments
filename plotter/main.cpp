#include <yaml-cpp/yaml.h>
#include "rapidcsv.h"
#include <iostream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "usage: " << argv[0] << " <path to yaml file>" << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(argv[1]);
  std::cout << "Parsed YAML:\n" << config << std::endl;
  rapidcsv::Document pvisc(config["PVIS_COORD"].as<std::string>(), rapidcsv::LabelParams(-1, -1));
  
  for (size_t i = 1; i <= 4; i++)
  {
    std::vector<float> x;  
    std::vector<float> y;
    auto assignment  = config["robot" + std::to_string(i)].as<std::vector<int>>();
    for (size_t j = 0; j < assignment.size(); j++)
    {
        x.emplace_back(pvisc.GetCell<float>(0, assignment[j]));
        y.emplace_back(pvisc.GetCell<float>(1, assignment[j]));
    }
    plt::scatter(x, y);
    plt::plot(x, y);

  }
  
 

//   // Set the size of output image to 1200x780 pixels
//     plt::figure_size(1200, 780);
//     // Plot line from given x and y data. Color is selected automatically.
//     plt::plot(x, y);
//     // Plot a red dashed line from given x and y data.
//     plt::plot(x, w,"r--");
//     // Plot a line whose name will show up as "log(x)" in the legend.
//     plt::named_plot("log(x)", x, z);
//     // Set x-axis to interval [0,1000000]
//     plt::xlim(0, 1000*1000);
//     // Add graph title
//     plt::title("Sample figure");
//     // Enable legend.
//     plt::legend();
    // Save the image (file format is determined by the extension)
    plt::save("./result.png");

  return 0;
}