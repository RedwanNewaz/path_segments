import numpy as np
import matplotlib.pyplot as plt
import yaml
import pathlib
from ObstaclePolygon import ObstaclePolygon
def read_csv(filename):
    return np.loadtxt(filename, delimiter=",")

def read_csv_from_dir(dirname):
    path = pathlib.Path(dirname)
    for filename in path.glob("*.csv"):
        xy = read_csv(filename)
        yield ObstaclePolygon(xy[:, 0],xy[:, 1])

if __name__ == "__main__":
    result = "../result.yaml"
    xy_coord = read_csv('../input/pvis.csv')

    with open(result) as file:
        data = yaml.safe_load(file)


    for i in range(data['NUM_ROBOTS']):
        indexes = data[f'robot{i+1}']
        plt.plot(xy_coord[indexes, 0], xy_coord[indexes, 1])
        plt.scatter(xy_coord[indexes, 0], xy_coord[indexes, 1])

    obstacles = list(read_csv_from_dir('../input/obstacles'))

    plt.show()
