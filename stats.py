import plotly
import plotly.plotly as py
import json
from itertools import *

from plotly.graph_objs import *

print plotly.__version__                # At least 1.8.6 is required. Upgrade with: $ pip install plotly --upgrade
from os import listdir

names = []
steps = []
avg = []
data = []
for file in sorted(listdir("output2")):
    with open("output2/" + file) as data_file: 
        x = json.load(data_file)
        names.append(file)
        data.append(x)
        print(file)
        
def getParam(key):
    return lambda x:x[key]

def averageByKey(key, xs):
    if(len(xs) == 0):
        return 0
    return sum(map(lambda x : x[key], xs)) /  float(len(xs))

for mapName, m in groupby(sorted(data, key = getParam("map")), getParam("map")):   
    cout = []
    nbNode = []
    algoNames = []
    for algoName, n in groupby(sorted(m, key = getParam("algo")), getParam("algo")):
        xs = filter(lambda x: "firstFind" in x and x["firstFind"] != 0, n)
        cout.append(averageByKey("cost", xs))
        if algoName in ["RRT","RRT*"]:
            nbNode.append(averageByKey("firstFind", xs))
        else:
            nbNode.append(0)
        algoNames.append(algoName + '-' + str(len(xs)))
    t0 = Bar(x = algoNames, y = cout, name="Cout moyen")
    t1 = Bar(x = algoNames, y = nbNode, name="Nombre de noeud dans le graphe")
    py.plot(Figure(data=[t0, t1], layout=Layout(barmode='group', title=mapName)), filename = mapName.replace("/", "_") + "_step")

