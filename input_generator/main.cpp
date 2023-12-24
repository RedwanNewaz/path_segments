#include "lkh_mtsp_solver/input_generator.h"

// double get_distance(double x1, double y1, double x2, double y2)
// {
//     double dx = x2 - x1;
//     double dy = y2 - y1;
//     return sqrt(dx * dx + dy * dy);
// }

// void save_file(const std::string& filename, const std::string& data)
// {
//   std::ofstream myfile;
//   myfile.open (filename);
//   myfile << data;
//   myfile.close();
// }

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "usage: " << argv[0] << " <path to yaml file>" << std::endl;
    return 1;
  }

  InputGenerator ig(argv[1]);
  fmt::print(ig.get_parameter_file());

 

  // YAML::Node config = YAML::LoadFile(argv[1]);
  // std::cout << "Parsed YAML:\n" << config << std::endl;

  // std::stringstream ss;
  // rapidcsv::Document pvisc(config["PVIS_COORD"].as<std::string>(), rapidcsv::LabelParams(-1, -1));
  // rapidcsv::Document adj(config["ADJ_MAT"].as<std::string>(), rapidcsv::LabelParams(-1, -1));
  // for(int i = 0; i < adj.GetRowCount(); ++i)
  // {
  //   for(int j = 0; j < adj.GetColumnCount(); ++j)
  //   {
  //       int cost = 9999;
  //       if(adj.GetCell<int>(j, i) == 1)
  //       {   
  //           // get distance 
  //           double x1 = pvisc.GetCell<double>(0, i);
  //           double y1 = pvisc.GetCell<double>(1, i);
  //           double x2 = pvisc.GetCell<double>(0, j);
  //           double y2 = pvisc.GetCell<double>(1, j);
  //           double d = get_distance(x1, y1, x2, y2);

  //           // get visibility
  //           double total_vis = pvisc.GetCell<double>(2, i) + pvisc.GetCell<double>(2, j);
  //           // compute cost matrix
  //           cost = 100000 * d / total_vis;
  //       }
  //       ss << cost << " ";
  //   }
  //   ss << "\n";
  // }
  // int DIM = adj.GetRowCount();
  // std::string ATSP =fmt::format(config["ATSP_TEMPLATE"].as<std::string>(), DIM, ss.str());
  // std::string atsp_file = config["PROBLEM_FILE"].as<std::string>(); 
  // save_file(atsp_file, ATSP);
  
  // // fmt::print(ATSP);

  // // gen par file
  // int NUM_ROBOTS = config["NUM_ROBOTS"].as<int>();
  // int MTSP_MIN_SIZE = DIM / NUM_ROBOTS - 1;

  // std::string par_file = config["TOUR_FILE"].as<std::string>(); 
  // std::string PAR =fmt::format(config["PAR_TEMPLATE"].as<std::string>(), atsp_file, NUM_ROBOTS, MTSP_MIN_SIZE, par_file);
  // save_file(par_file, PAR);
 
  // fmt::print(PAR);
  // fmt::print("\n");

  return 0;
}