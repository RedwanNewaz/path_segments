//
// Created by redwan on 11/27/23.
//
// mylibrary.cpp
#include <boost/python.hpp>
#include "conflict_based_decomposer.h"
using namespace boost::python;


class cbd_ta
{
public:

    cbd_ta(int numAgents, const boost::python::list& X, const boost::python::list& Y)
    {
        PathDecomposition::PATH path;
        for (int i = 0; i < len(X); ++i) {
            int x = boost::python::extract<int>(X[i]);
            int y = boost::python::extract<int>(Y[i]);
            std::pair<int, int> coord = std::make_pair(x, y);
            path.push_back(coord);
        }
        decomposer_ = new ConflictBasedDecomposer(numAgents, path);
    } // added constructor
    ~cbd_ta()
    {
        delete decomposer_;
    }
    void setAgents(const boost::python::list& X, const boost::python::list& Y)
    {

        PathDecomposition::PATH agents;
        for (int i = 0; i < len(X); ++i) {
            int x = boost::python::extract<int>(X[i]);
            int y = boost::python::extract<int>(Y[i]);
            std::pair<int, int> coord = std::make_pair(x, y);
            agents.push_back(coord);
        }
        decomposer_->computeCost(agents);
    }
    boost::python::list getAgentPath(int agentId, int index){
        auto path = decomposer_->getAgentPath(agentId);
        boost::python::list result;
        for (const auto& p: path) {
            if(index == 0)
                result.append(p.first);
            else
                result.append(p.second);
        }
        return result;

    }

private:

    ConflictBasedDecomposer *decomposer_;

};


BOOST_PYTHON_MODULE(cbd_ta)
{
    Py_Initialize();
    class_<cbd_ta>("cbd_ta", init<int, const boost::python::list&, const boost::python::list&>())
            .def(init<int, const boost::python::list&, const boost::python::list&>())
            .def("setAgents", &cbd_ta::setAgents)
            .def("getAgentPath", &cbd_ta::getAgentPath)
            ;
}