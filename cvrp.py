import numpy as np
import matplotlib.pyplot as plt
import pathlib
from pprint import pprint
from collections import defaultdict
import os
import subprocess
def read_csv(filename):
    return np.loadtxt(filename, delimiter=",")

def read_csv_from_dir(dirname):
    path = pathlib.Path(dirname)
    for filename in path.glob("*.csv"):
        yield read_csv(filename)

# http://akira.ruc.dk/~keld/research/LKH-3/
class mtsp_solver:
    ATSP_TEMPLATE = '''NAME : A-G-90-3
TYPE : ACVRP
DIMENSION : {}
EDGE_WEIGHT_TYPE : EXPLICIT
EDGE_WEIGHT_FORMAT : FULL_MATRIX
DISPLAY_DATA_TYPE : NO_DISPLAY
CAPACITY : {}
VEHICLES : {}
EDGE_WEIGHT_SECTION
{}
DEMAND_SECTION
{}
DEPOT_SECTION
9
EOF
'''

    PAR_TEMPLATE = '''SPECIAL
PROBLEM_FILE = {}
SALESMEN = {}
MTSP_OBJECTIVE = MINSUM
MTSP_MIN_SIZE = 20
RUNS = 3
TRACE_LEVEL = 1
TOUR_FILE = {}
    '''
    EXE="lib/mtsp_node"
    def __init__(self, num_agents, adjFile, visibilityFile,  prob_dir, result_dir):
        self.num_agents = num_agents
        self.prob_dir =    prob_dir

        self.prob_name =   'vmtsp_%d.atsp' % self.num_agents
        self.result_dir =  result_dir
        self.result_name = 'vmtsp_%d.tour' % self.num_agents

        self.adjFile = adjFile
        self.visibilityFile = visibilityFile

    def solve(self):
        pdf = self.__gen_problem()
        args = self.__gen_args()
        self.__solver(pdf, args)

    def plot(self):

        xy_coord = read_csv(self.visibilityFile )
        plt.scatter(xy_coord[:, 0], xy_coord[:, 1], c='k')

        tours = self.__get_result()


        for res in tours.values():
            indexes = list(res)
            plt.plot(xy_coord[indexes, 0], xy_coord[indexes, 1])
            plt.scatter(xy_coord[indexes, 0], xy_coord[indexes, 1])

        for obs in read_csv_from_dir('input/obstacles'):
            plt.fill(obs[:, 0], obs[:, 1], 'k')

        plt.show()

    @staticmethod
    def genEdgeWeights(adjFile, visibilityFile):
        adj = read_csv(adjFile)
        pvs = read_csv(visibilityFile)
        visibility = 100 * pvs[:, 2] / min(pvs[:, 2])
        xy = pvs[:, :2]

        N, M = len(adj), len(adj[0])
        assert N == M

        edge_weights = np.zeros((N, M), dtype=int)
        for i in range(N):
            # print(i, end=": ")
            for j in range(M):
                cost = 0
                if adj[i, j]:
                    dist = np.linalg.norm(xy[i] - xy[j])
                    cost = dist
                edge_weights[i, j] = cost * 1000
        return edge_weights, visibility

    @staticmethod
    def extract_tour_selection_indexes(file_path):
        with open(file_path, 'r') as file:
            content = file.read()

        # Split the content into lines
        lines = content.split('\n')

        # Find the index where TOUR_SECTION starts
        tour_section_index = lines.index("TOUR_SECTION")

        # Extract TOUR_SELECTION indexes
        tour_selection_indexes = []
        for line in lines[tour_section_index + 1:]:
            if line == "-1" or line == "EOF":
                break
            tour_selection_indexes.append(int(line))

        return tour_selection_indexes
    def __get_result(self):
        file_path = os.path.join(self.result_dir, self.result_name)
        tour_selection_indexes = self.extract_tour_selection_indexes(file_path)
        # print("TOUR_SELECTION Indexes:", tour_selection_indexes)

        tours = defaultdict(list)
        currentID = 0
        for index in tour_selection_indexes:
            nodeId = index - self.num_agents
            if nodeId <= 0:
                currentID = abs(nodeId)
            else:
                tours[currentID].append(nodeId)
        return tours
    def __solver(self, pdf, args):
        # write problem
        pdfFile = os.path.join(self.prob_dir, 'vmtsp_%d.atsp' % self.num_agents)
        inpFile = os.path.join(self.prob_dir, 'vmtsp_%d.par' % self.num_agents)
        with open(pdfFile, 'w+') as file:
            file.write(pdf)
        with open(inpFile, 'w+') as file:
            file.write(args)
        subprocess.call([self.EXE, inpFile])

    def __gen_problem(self):
        edges, vis = self.genEdgeWeights(self.adjFile, self.visibilityFile)
        capacity = 1000
        N = len(edges)
        formatted_string = '\n'.join([' '.join(map(str, row)) for row in edges])
        demand_string = '\n'.join([' '.join(map(str, [i + 1, int(v - min(vis)) ])) for i, v in enumerate(vis)])
        atsp = self.ATSP_TEMPLATE.format(N, capacity, self.num_agents,
                                                     formatted_string, demand_string)
        return atsp

    def __gen_args(self):
        problem = os.path.join(self.prob_dir, self.prob_name)
        outdir = os.path.join(self.result_dir, self.result_name)
        args = self.PAR_TEMPLATE.format(problem, self.num_agents, outdir)
        return args



if __name__ == '__main__':
    num_agents = 3
    prob_dir = os.path.join(os.getcwd(), 'input/mtsp')
    result_dir = os.path.join(os.getcwd(), 'results/amtsp')
    adjFile = 'input/adjmatrix2.csv'
    visibilityFile = 'input/pvis.csv'

    mtsp = mtsp_solver(num_agents, adjFile, visibilityFile, prob_dir, result_dir)
    mtsp.solve()
    mtsp.plot()