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

## On the supervisor controller

l.11 : #define _GNU_SOURCE (need to be before including <stdio.h>
l.46 : declare global pointer data
l.121 : function read_csv 
l.151 : function handle_line_csv
l.181 : function test_param(params)
l.273 to 280 : main, param to declare and fuction calling
l.300 to 325 : calculation of performance and update of the line to be read, 
	+ writting onto param file + gestion of simulation reloading.