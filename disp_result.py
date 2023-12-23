import numpy as np
import matplotlib.pyplot as plt
import pathlib
def read_csv(filename):
    return np.loadtxt(filename, delimiter=",")

def read_csv_from_dir(dirname):
    path = pathlib.Path(dirname)
    for filename in path.glob("*.csv"):
        yield read_csv(filename)

def main():
    inp_file = 'input/xy.csv'
    xy_coord = read_csv(inp_file)

    plt.scatter(xy_coord[:, 0], xy_coord[:, 1], c='k')


    for res in read_csv_from_dir('results'):
        plt.plot(res[:, 0], res[:, 1])
        plt.scatter(res[:, 0], res[:, 1])

    for obs in read_csv_from_dir('input/obstacles'):
        plt.fill(obs[:, 0], obs[:, 1], 'k')

    plt.show()

def convert():
    inp = "input/pointsvisibility.csv"
    data = read_csv(inp)
    xy = data[:, :2]
    z = data[:, 3:]
    xyz = np.column_stack((xy, np.max(z, axis=1)))
    print(xy.shape, z.shape, xyz.shape)
    np.savetxt("input/pvis.csv", xyz, delimiter=",")

if __name__ == '__main__':
    # main()
    adj = read_csv('input/adjmatrix2.csv')
    visibility = read_csv('input/pvis.csv')
    for obs in read_csv_from_dir('input/obstacles'):
        plt.fill(obs[:, 0], obs[:, 1], 'k')

    N = len(adj)
    for i in range(N):
        for j in range(N):
            if adj[i, j]:
                plt.plot([visibility[i, 0], visibility[j, 0]], [visibility[i, 1], visibility[j, 1]], 'k')

    plt.show()