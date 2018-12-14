import numpy as np
import matplotlib.pyplot as plt
import csv

from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D


#path = '/home/nctrl/disIntSystems/dis_project/webots/Optimisation results'
#scenario = '/crossing'
#run = '/run01_2'
#resultFile = '/Reynolds_performance.csv'
#filename = path + scenario + run  + resultFile




USE_AVG   = 1
USE_FINAL = 2

METRIC = USE_AVG
#METRIC = USE_FINAL

OBSTACLES = 1
CROSSING  = 2

SCENARIO = CROSSING
#SCENARIO = OBSTACLES

RATIO = '1_2'


path = '/home/nctrl/disIntSystems/dis_project/webots/Optimisation results'
if(SCENARIO == OBSTACLES):
	path += '/obstacles'
if(SCENARIO == CROSSING):
	path += '/crossing'
path = path + '/run0' + RATIO + '/Reynolds_performance.csv'



final  = 10 



with open(path) as csvfile:
	reader = csv.reader(csvfile) # change contents to floats
	data = list(reader)


def get_avg_perf(db):
	perf = []
	single_perf = []
	start = False

	for i, row in enumerate(db):
		if(row[0] == str(1) and not(start)):
			start = True
			continue

		if(row[0] == str(1) and start):
			perf.append(np.mean(single_perf))
			single_perf = []
			continue

		if(i == len(db)-1):
			single_perf.append(float(db[i][6]))
			perf.append(np.mean(single_perf))

		if(start):
			single_perf.append(float(db[i][6]))
			continue

	return perf


def get_final_perf(db):
	perf = []
	start = False

	for i, row in enumerate(db):
		#for j, col in enumerate(row):
		if(not(start) and row[0] == str(1)):
			start = True
			continue

		if(i == len(db)-1):
			perf.append(float(db[i][6]))

		if(start and row[0] == str(1)):
			perf.append(float(db[i-1][6]))

	return perf

perf = get_final_perf(data)
avg_perf = get_avg_perf(data)

def center_of_mass(db):
	sum_x = 0
	sum_y = 0
	sum_z = 0

	M = 0

	for i in range(0, final):
		for j in range(0, final):
			for k in range(0, final):
				sum_x += db[i,j,k] * (i+1)
				sum_y += db[i,j,k] * (j+1)
				sum_z += db[i,j,k] * (k+1)
				M += db[i,j,k]

	return [sum_x/M, sum_y/M, sum_z/M]



def data4d(db):
	mesh = np.zeros([final,final,final])

	ctr = 0
	for i in range(0, final):
		for j in range(0, final):
			for k in range(0, final):
				mesh[i,j,k] = db[ctr]
				ctr = ctr +1
	return mesh

if(METRIC == USE_AVG):
	print('Using AVG performance')
	mesh = data4d(avg_perf)
elif(METRIC == USE_FINAL):
	print('Using FINAL performance')
	mesh = data4d(perf)
else:
	print('Unknown metric')


print('Center of mass of the 4d data:', center_of_mass(mesh))

def flatten4d(db):
	ix = []
	vals = []

	for i in range(0, final):
		for j in range(0, final):
			for k in range(0, final):
				vals.append(db[i,j,k])
				ix.append([i+1,j+1,k+1])


	return {'ix':ix, 'vals':vals}

def sorter4d(nb, db):
	ff = flatten4d(db)
	LI = np.argsort(ff['vals'])
	LI = np.flip(LI)

	for i in range(nb):
		print('  ',ff['ix'][LI[i]],ff['vals'][LI[i]])

print('Best 10 results:')
sorter4d(10, mesh)


mmin = np.amin(mesh);
mmax = np.amax(mesh);

print('min max: ', mmin, ' / ', mmax)


tic = list(range(final))
lab = [str(i+1) for i in tic]

fig = plt.figure(1, figsize=(6,6))
main_ax = fig.add_axes([0.1,0.25,0.8,0.7], xticks=tic, yticks=tic, xlabel='x', ylabel='y', xticklabels=lab, yticklabels=lab)
slider_ax  = fig.add_axes([0.1,0.1,0.8,0.05])

main_ax.imshow(mesh[:,:,0], aspect='auto', vmin=mmin, vmax=mmax)


my_slider = Slider(slider_ax, 'z', 1, final, valinit=1, valfmt='%d')

def update(val):
    main_ax.imshow(mesh[:,:,int(val-1)], aspect='auto', vmin=mmin, vmax=mmax)
    plt.draw()

my_slider.on_changed(update)
plt.show()


