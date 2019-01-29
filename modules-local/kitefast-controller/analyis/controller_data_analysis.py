#! /usr/bin/env python
""" controller_data_analysis.py

This script is used to plot variables from the contoller output file (.csv).
The .csv contains a header line with all available variables. The user must specify
which variables to plot below using the 'plot_vars' variable. 
This script will cycle through the .csv, look for the variables that are speicified
by the user, and then save the values to a final array. This script also creates
a time vector and inserts it as the first column in the final array. 
This time vector is calculated by 'dT' and the number of data entries within the .csv
(essentially the number of rows). 
Once the final array is assembled, each column (chosen variable) is plotted 
against the time vector to produce a plot for each chosen variable

The row_count and header_count are the number of rows found within .csv 
and are used in the cycling feature:
	row_count - The total number of rows found in the .csv
	header_count - the total number of rows that are headers (should always be 4)   

TODO - have controller save the generated .csv to this directory
"""

__author__ = 'jmiller@systemstech.com (Justin Miller)'

print "running contorller analysis"

import csv
import numpy as np
import matplotlib.pyplot as plt

# user inputs
plot_vars = ["sph_f.v","sph_f.alpha","flaps[0]", "rotors[7]"] #variables to be plotted
file_name = "controller_save_data.csv" #file name of data file
time_val = 0 #inital time of data/plots
dT = 0.02 #sec (time step)
row_count = 9 # total rows found in .csv
header_count = 4 # total number of rows that are headers

# initialize variables
plot_ind = []
plot_vals = np.zeros([row_count-header_count,len(plot_vars)+1])
temp_vals = np.zeros([len(plot_vars)+1])

# TODO - have controller save the .csv to this directory
with open(file_name) as file:
#with open("modules-local/kitefast-controller/analyis/controller_save_data.csv") as file:
	reader = csv.reader(file,delimiter=',')
	line_count = 0
	for row in reader:
		# first header
		if line_count == 0:
			print(row)
			line_count += 1
		# print date when file created
		elif line_count == 1:	
			print(row)			
			line_count += 1
		# print controller version
		elif line_count == 2:
			print(row)				
			line_count += 1
		# sort through data header
		elif line_count == 3:
			i = 0
			# grab header and make list
			header = ', '.join(row)
			header_list = [x.strip() for x in header.split(',')]
			# find indicies of user inputted variables to plot	
			while i < len(plot_vars):
				if plot_vars[i] in header_list:
					plot_ind.append(header_list.index(plot_vars[i]))
				else:
					msg = "Input " + plot_vars[i] + " is not a valid variable"
					exit(msg)
				i += 1
			line_count += 1
		else:
			# assemble data of chosen variables
			j = 0
			while j < len(plot_vars)+1:
				if j == 0:
					temp_vals[j] = time_val
					time_val += dT
				else:
					temp_vals[j] = row[plot_ind[j-1]]
				j += 1
			# assign to final structure  
			plot_vals[line_count-header_count] = temp_vals 
			line_count += 1
# final data struct
# print header
print "Variables to be Plotted:"
print(', '.join(plot_vars)) 
print "Values:"
print plot_vals
print "Processed " ,line_count, " lines"

#begin plotting
k = 0
while k < len(plot_vars):
	plt.figure(k+1)
	plt.title(plot_vars[k])
	plt.plot(plot_vals[:,0], plot_vals[:,k+1], '-')
	plt.axis([ np.min(plot_vals[:,0]), np.max(plot_vals[:,0]), np.min(plot_vals[:,k+1]), np.max(plot_vals[:,k+1]) ])
	plt.show(block=True)
	k += 1
