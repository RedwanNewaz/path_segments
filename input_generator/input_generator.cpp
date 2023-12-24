#include "lkh_mtsp_solver/input_generator.h"

InputGenerator::InputGenerator(const char * inpFile)
{
    config = YAML::LoadFile(inpFile);
    std::cout << "Parsed YAML:\n" << config << std::endl;
}

std::string InputGenerator::get_parameter_file() 
{
    create_atsp_file();
    create_par_file();
    return par_file;
}

std::string InputGenerator::create_atsp_file()
{
    std::stringstream ss;
    rapidcsv::Document pvisc(config["PVIS_COORD"].as<std::string>(), rapidcsv::LabelParams(-1, -1));
    rapidcsv::Document adj(config["ADJ_MAT"].as<std::string>(), rapidcsv::LabelParams(-1, -1));
    for(int i = 0; i < adj.GetRowCount(); ++i)
    {
        for(int j = 0; j < adj.GetColumnCount(); ++j)
        {
            int cost = 9999;
            if(adj.GetCell<int>(j, i) == 1)
            {   
                // get distance 
                double x1 = pvisc.GetCell<double>(0, i);
                double y1 = pvisc.GetCell<double>(1, i);
                double x2 = pvisc.GetCell<double>(0, j);
                double y2 = pvisc.GetCell<double>(1, j);
                double d = get_distance(x1, y1, x2, y2);

                // get visibility
                double total_vis = pvisc.GetCell<double>(2, i) + pvisc.GetCell<double>(2, j);
                // compute cost matrix
                cost = 100000 * d / total_vis;
            }
            ss << cost << " ";
        }
        ss << "\n";
    }
    DIM = adj.GetRowCount();
    std::string ATSP =fmt::format(config["ATSP_TEMPLATE"].as<std::string>(), DIM, ss.str());
    atsp_file = config["PROBLEM_FILE"].as<std::string>(); 
    save_file(atsp_file, ATSP);
    return atsp_file;
}

std::string InputGenerator::create_par_file()
{
    int NUM_ROBOTS = config["NUM_ROBOTS"].as<int>();
    int MTSP_MIN_SIZE = DIM / NUM_ROBOTS - 1;

    par_file = config["TOUR_FILE"].as<std::string>(); 
    std::string PAR =fmt::format(config["PAR_TEMPLATE"].as<std::string>(), atsp_file, NUM_ROBOTS, MTSP_MIN_SIZE, par_file);
    save_file(par_file, PAR);
    return par_file;
}
double InputGenerator::get_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}
void InputGenerator::save_file(const std::string& filename, const std::string& data)
{
    std::ofstream myfile;
    myfile.open (filename);
    myfile << data;
    myfile.close();

}
