import numpy as np 
import matplotlib.pyplot as plt 
import yaml
import pathlib

def read_csv(filename):
    return np.loadtxt(filename, delimiter=",")

def read_csv_from_dir(dirname):
    path = pathlib.Path(dirname)
    for filename in path.glob("*.csv"):
        yield read_csv(filename)

if __name__ == "__main__":
    result = "result.yaml"
    
    with open(result) as file:
        data = yaml.safe_load(file)
        xy_coord = np.loadtxt('input/pvis.csv', delimiter=",")

    for i in range(data['NUM_ROBOTS']):
        indexes = data[f'robot{i+1}']
        plt.plot(xy_coord[indexes, 0], xy_coord[indexes, 1])
        plt.scatter(xy_coord[indexes, 0], xy_coord[indexes, 1])
    
    for obs in read_csv_from_dir('input/obstacles'):
        plt.fill(obs[:, 0], obs[:, 1], 'k')
    plt.savefig('result.png')
    print('result.png updated')
