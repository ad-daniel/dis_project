# Recap of this commit

Implementation of csv file with the parameters to be tested, parameters are written on the file name
Implementation of csv file that update the line that need to be read for the next simulation
Read these file at every reload
Implementation of function for : controlling action to be executed on a file

Improvments to be done : 
- Add a line with the parameters so we can directly plot wich param is more optimal in Matlab
- When test for 1 param done, Matlab send this optimal param, automatically used for next tests
- choice of range automatically done, link with Matlab results and Webots code

# How to implement parameters optimization on your supervisor

## On the supervisor controller flock_super2.c

l.11 : #define _GNU_SOURCE (need to be before including <stdio.h>
l.32 : WbDeviceTag emitter;
l.46/47: declare global pointer data_glob and data_line
l.118 : function read_csv 
l.146 : function handle_line_csv
l.174 : function test_param(params)
l.280 to 286 : main, param to declare and fuction calling
l.307 to 325 : calculation of performance and update of the line to be read, 
	+ writting onto param file + gestion of simulation reloading.
	
	Do a search, "Mathilde", to find more easily.
	
## On the controller controller reynolds2.c

l.86, 91, 113, 124 to 136.