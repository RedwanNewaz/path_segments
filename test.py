import numpy as np
from build.cbd_ta import cbd_ta
# from cbd_ta import cbd_ta
path = np.loadtxt('test/coords.csv', delimiter=',')
X = path[:, 0].astype(int).tolist()
Y = path[:, 1].astype(int).tolist()

NumAgents = 4

cbd = cbd_ta(NumAgents, X, Y, False)
agentX = [27, 28, 11, 49]
agentY = [46, 34, 26, 25]
cbd.setAgents(agentX, agentY)

for i in range(NumAgents):
    path = list(zip(cbd.getAgentPath(i, 0), cbd.getAgentPath(i, 0)))
    print(path)