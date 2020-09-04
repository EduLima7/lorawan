import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import sys


fontSize = 18
plt.rc('font', size=fontSize)  


# Nodes
splitSymb = ' '
file = open("nodes.txt", "r")
data = []
for line in file:
	data.append(line.split(splitSymb))
file.close()
# data.append([0,0,5,])
nlines = len(data); ncolls = len(data[0])
nodes = np.zeros([nlines,ncolls])
for i in range(nlines):
	nodes[i,:] = np.array(data[i])

# Gateways
splitSymb = ' '
file = open("gws.txt", "r")
data = []
for line in file:
	data.append(line.split(splitSymb))
file.close()
nlines = len(data); ncolls = len(data[0])
gws = np.zeros([nlines,ncolls])
for i in range(nlines):
	gws[i,:] = np.array(data[i])

colors = ['brown','black', 'blue', 'green', 'cyan', 'yellow', 'magenta']
x_nodes = nodes[:,0]
y_nodes = nodes[:,1]
SFs = nodes[:,2]
x_gws = gws[:,0]
y_gws = gws[:,1]
SFcolor = [colors[int(i)-6] for i in SFs]
fig, ax = plt.subplots(figsize=(7, 6))
plt.tight_layout(rect=(0.022, 0, 1.05, .9))

scatter = ax.scatter(x_nodes, y_nodes, c=SFcolor, s=40)
scatter = ax.scatter(x_gws, y_gws, c='red', s=80)

Handles = []
for c in range(1,len(colors)):
	Handles.append(mpatches.Patch(color=colors[c], label='SF'+str(c+6)))
Handles.append(mpatches.Patch(color='brown', label='Out of range devices'))
Handles.append(mpatches.Patch(color='red', label='Gateway'))

plt.legend(handles=Handles, bbox_to_anchor=(0., 1.02, 1., .102), mode='expand', loc='lower center', ncol=4,columnspacing=0.4,handletextpad=0.2, handlelength=1., borderaxespad=0.)

plt.savefig('mapPlot')
# plt.show()