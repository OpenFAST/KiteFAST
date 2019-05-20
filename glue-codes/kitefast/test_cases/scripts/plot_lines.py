# -*- coding: utf-8 -*-
"""
Created on Fri Jul 13 14:04:04 2018
Reading and plotting mooring line out files
@author: fwendt
"""

import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.set_xlim(-20,20)
#ax.set_ylim(-20,20)
#fig.ylim(-20,20)
#plt.zlim(0,100)



name_list=list()
max_row=100
row_step=100
row_cur=1
line1new=1
line2new=1
line3new=1
line4new=1
line5new=1
line6new=1

#find all mooring files (assuming there are less than 6 and at least 1 file)
while row_cur<max_row:
    for lineID in range(6):
        for file in os.listdir("."):
            search_string="Line"+str(lineID)+".out"
            if file.endswith(search_string):
                #read the header line
                with open(file) as f:
                    first_line = f.readline()
                    header=first_line.split()
                    #print(file)
                    name_list.append(file)
                    #read the values from the file
                    if (lineID == 1 and line1new == 1):
                        line_data=np.loadtxt(file,skiprows=2)
                        #reduce the line_data to unique time rows
                        u, u_indices = np.unique(line_data[:,0], return_index=True)
                        line_data=line_data[u_indices,:]
                        
                        line_data1=line_data
                        line1new=0
                    elif (lineID == 1 and line1new == 0):
                        line_data=line_data1
                        
                    if (lineID == 2 and line2new == 1):
                        line_data=np.loadtxt(file,skiprows=2)
                        #reduce the line_data to unique time rows
                        u, u_indices = np.unique(line_data[:,0], return_index=True)
                        line_data=line_data[u_indices,:]
                        
                        line_data2=line_data
                        line2new=0
                    elif (lineID == 2 and line2new == 0):
                        line_data=line_data2
                        
                    if (lineID == 3 and line3new == 1):
                        line_data=np.loadtxt(file,skiprows=2)
                        #reduce the line_data to unique time rows
                        u, u_indices = np.unique(line_data[:,0], return_index=True)
                        line_data=line_data[u_indices,:]
                        
                        line_data3=line_data
                        line3new=0
                    elif (lineID == 3 and line3new == 0):
                        line_data=line_data3
                    
                    #plot the line geometry data
                    num_of_points=np.int((line_data.shape[1]))
                    line_crds=np.empty([num_of_points, 3])
                    max_row=line_data.shape[0]
                pt=0 
                stop_here=0
                col=1
                while stop_here==0:
                    xcol_val=line_data[row_cur,col]
                    ycol_val=line_data[row_cur,col+1]
                    zcol_val=line_data[row_cur,col+2]
                    line_crds[pt,:]=[xcol_val,ycol_val,zcol_val]
                    pt=pt+1
                    col=col+3
                    if "Seg" in header[col]:
                        stop_here=1
                        for del_row in range(pt,int(line_crds.shape[0])): 
                            line_crds=np.delete(line_crds, pt, 0)
                


                ax.plot(line_crds[:,0], line_crds[:,1], line_crds[:,2])
                plt.title(line_data[row_cur,0])
        ax.set_xlim(-400,400)
        ax.set_ylim(-400,400)
    if row_cur>=max_row-row_step:
    	row_cur=row_cur+row_step
    	plt.draw()
    	plt.pause(5.0)
    	plt.cla()
    else:
        row_cur=row_cur+row_step
        plt.draw()
        plt.pause(0.000001)
        plt.cla()
