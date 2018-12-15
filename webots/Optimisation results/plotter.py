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

#OBSTACLES = 1
#CROSSING  = 2

#SCENARIO = CROSSING
#SCENARIO = OBSTACLES

RATIO = '1_2'

print('USING RATIO ', RATIO)

path_obs = '/home/nctrl/disIntSystems/dis_project/webots/Optimisation results/obstacles/ratio0'+RATIO+'/Reynolds_performance.csv'
path_cross = '/home/nctrl/disIntSystems/dis_project/webots/Optimisation results/crossing/ratio0'+RATIO+'/Reynolds_performance.csv'
final  = 10 

def getdata(path):
	with open(path) as csvfile:
		reader = csv.reader(csvfile) # change contents to floats
		data = list(reader)
	
	return data

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


data_obs   = getdata(path_obs)
data_cross = getdata(path_cross)

obs_perf = get_final_perf(data_obs)
obs_avg_perf = get_avg_perf(data_obs)

cross_perf = get_final_perf(data_cross)
cross_avg_perf = get_avg_perf(data_cross)


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

def combined4d(obs,cross):
	mesh = np.zeros([final,final,final])

	ctr = 0
	for i in range(0, final):
		for j in range(0, final):
			for k in range(0, final):
				mesh[i,j,k] = obs[i,j,k]+cross[i,j,k]	
	return mesh

if(METRIC == USE_AVG):
	print('Using AVG performance')
	obs_mesh   = data4d(obs_avg_perf)
	cross_mesh = data4d(cross_avg_perf)
	comb_mesh  = combined4d(obs_mesh, cross_mesh)
elif(METRIC == USE_FINAL):
	print('Using FINAL performance')
	obs_mesh = data4d(perf)
	cross_mesh = data4d(cross_perf)
	comb_mesh  = combined4d(obs_mesh, cross_mesh)
else:
	print('Unknown metric')


print('OBS  : CoM of the 4d data:', center_of_mass(obs_mesh))
print('CROSS: CoM of mass of the 4d data:', center_of_mass(cross_mesh))
print('COMB : CoM of mass of the 4d data:', center_of_mass(comb_mesh))

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

print('Best OBS results:')
sorter4d(10, obs_mesh)
print('Best CROSS results:')
sorter4d(10, cross_mesh)
print('Best COMB results:')
sorter4d(10, comb_mesh)

o_min = np.amin(obs_mesh);
o_max = np.amax(obs_mesh);

c_min = np.amin(cross_mesh);
c_max = np.amax(cross_mesh);

b_min = np.amin(comb_mesh);
b_max = np.amax(comb_mesh);

print('OBS  : min max : ', o_min, ' / ', o_max)
print('CROSS: min max : ', c_min, ' / ', c_max)
print('COMB : min max : ', b_min, ' / ', b_max)


def plot_fig(mesh, mmin, mmax):
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

	#heatmap = main_ax.pcolor(mesh, cmap=plt.cm.Blues)
	#plt.colorbar(heatmap)

	my_slider.on_changed(update)
	plt.show()

#plot_fig(obs_mesh, o_min, o_max)
#plot_fig(cross_mesh, c_min, c_max)
plot_fig(comb_mesh, b_min, b_max)
