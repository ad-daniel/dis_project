import numpy as np
import matplotlib.pyplot as plt
import csv

from matplotlib.widgets import Slider


#nx = 100
#ny = 100
#nz = 100

#data = np.random.rand(nx,ny,nz)

filename = '/home/nctrl/Desktop/results/crossing/run01/' + 'Reynolds_performance.csv'

with open(filename, 'rb') as csvfile:
	csvfile = csv.reader(csvfile, delimiter=' ', quotechar='|')
	for row in csvfile:
		print(', ').join(row)


fig = plt.figure(1, figsize=(6,6))
main_ax = fig.add_axes([0.1,0.2,0.8,0.7])
slider_ax  = fig.add_axes([0.1,0.1,0.8,0.05])

main_ax.imshow(data[:,:,0], aspect='auto')


my_slider = Slider(slider_ax, 'layer', 0, nz, valinit=0, valfmt='%d')

def update(val):
    main_ax.imshow(data[:,:,int(val)], aspect='auto')
    plt.draw()

my_slider.on_changed(update)
plt.show()