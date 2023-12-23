import numpy as np
import matplotlib.pyplot as plt
import pathlib
def read_csv(filename):
    return np.loadtxt(filename, delimiter=",")

def read_csv_from_dir(dirname):
    path = pathlib.Path(dirname)
    for filename in path.glob("*.csv"):
        yield read_csv(filename)

if __name__ == '__main__':
    inp_file = 'input/xy.csv'
    xy_coord = read_csv(inp_file)

    # plt.plot(xy_coord[:, 0], xy_coord[:, 1], 'ko')


    for res in read_csv_from_dir('results'):
        plt.plot(res[:, 0], res[:, 1])
        plt.scatter(res[:, 0], res[:, 1])

    for obs in read_csv_from_dir('input/obstacles'):
        plt.fill(obs[:, 0], obs[:, 1], 'k')

    plt.show()