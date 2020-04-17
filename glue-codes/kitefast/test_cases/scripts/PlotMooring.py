import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import argparse
from pathlib import Path
import matplotlib.animation as animation
import sys, datetime

def PlotMooring(path2kfile,anifile="",decimation=10):
    """
    This function reads mooring and aircraft attitude and plots mooring line positions and aircraft triad.
    path2kfile: Pathlib object, path+filename to KFAST out file
    anifile: string, filename for animation; set to "" if no saving is requested
    decimation: number of data points to skip
    Note: the *Line#.out files will be automatically read from the same dir
    4.6.2020:  COPYRIGHT: RRD Engineering and MAKANI Wind Power.
    """

    #Start by getting Aircraft MIP locations from KFAST output
    colors=['red','green','blue']
    lncolors=['black','cyan','brown']
    coords=['x','y','z']
    ncoords=len(coords)
    print('Reading')

    KTrajstr=np.core.defchararray.add(np.tile('KiteP',ncoords),np.core.defchararray.add(coords,np.tile('i',ncoords)))
    
    df=pd.read_csv(path2kfile,header=1,squeeze=True,delim_whitespace=True,skiprows=1,low_memory=False) #nrows=10,b
    idx_k=df.index[1::decimation]
    
    time_k=df.loc[idx_k,'Time'].values.astype('float')
    KTraj_df=df.loc[idx_k,KTrajstr.tolist()].astype('float')
    Xg_k=KTraj_df.to_numpy('float')

    # Then get DCMS  
    ctr3=np.linspace(1,9,9,dtype=int).astype('str')
    dcmg2b_k_str=np.core.defchararray.add(np.tile('MIPDCM',9),ctr3)
    dcmg2b_k=df.loc[idx_k,dcmg2b_k_str].astype('float').to_numpy()

    dcmg2b_plt=dcmg2b_k.reshape([-1,3,3],order='F') #this is put into F, as this is the correct 3x3

    # Now read in line data and plot
    # find all line files
    wdir=path2kfile.parent
    linefiles=sorted([f for f in wdir.rglob('*.Line*.out')])
    nlines=len(linefiles)

    lines=nlines*[0] #initialize
    nNodes=nlines*[0]
    maxcol=nlines*[0]

    ax=4*[0]
    
    for ii,linefile in enumerate(linefiles):
        
        print('Reading Line ',str(ii),linefile)

        with open(linefile.as_posix(),'r') as infile:
            #count how many points we have
            header = infile.readline().split()
            maxcol[ii]=header.index('Seg1Ten')#this is the column in the file where I have to stop reading
            nNodes[ii]=int((maxcol[ii]-1)/3) 
            
            line_data=np.loadtxt(infile,skiprows=2)
            nrows=line_data.shape[0] #original number of rows
            #reduce the line_data to unique time rows
            junk, idx = np.unique(np.flipud(line_data[:,0].copy()), return_index=True)
            lines[ii]=line_data[nrows-idx-1,0:maxcol[ii]+1]
            
    #Find the indices of the line files to plot together with trajectory by selecting the same time stamps
    _,it2,_=np.intersect1d(lines[0][:,0], time_k,assume_unique=True,return_indices=True)
    if it2.__len__()==0:
        print('No common time stamps found in line files; try changing decimation')
        exit
    
    plines=4*[0]
    Q1,Q2,Q3,Q4=3*[0],3*[0],3*[0],3*[0]  #initialize 3 vectors +3 lines in each of the 4 plots

    Q=(nlines*4)*[0] #initialize 3 vectors +3 lines in each of the 4 plots
    
    def init_plot():
        """sets the inital axes and plots trajectory)"""

        #Aircraft trajectory
        plines[3]=ax[3].plot(Xg_k[:,0],Xg_k[:,1],Xg_k[:,2], 'blue',label='Xg_k')
        ax[3].set_xlabel('x [m]')
        ax[3].set_ylabel('y [m]')
        ax[3].set_zlabel('z [m]')
        ax[3].set_xlim(0,440)
        ax[3].set_ylim(-180,120)
        ax[3].set_zlim(90,320)

        #-yz
        plines[0]=ax[0].plot(Xg_k[:,1],Xg_k[:,2], 'blue',label='Xg_k')
        ax[0].set_xlabel('y')   
        ax[0].set_ylabel('z')
        ax[0].set_xlim(120,-180)
        ax[0].set_ylim(90,350)
        #xz
        plines[1]=ax[1].plot(Xg_k[:,0],Xg_k[:,2], 'blue',label='Xg_k')
        ax[1].set_xlabel('x')
        ax[1].set_ylabel('z')
        ax[1].set_xlim(0,440)
        ax[1].set_ylim(90,350)
        #xy
        plines[2]=ax[2].plot(Xg_k[:,1],Xg_k[:,0], 'blue',label='Xg_k')
        ax[2].set_xlabel('y')
        ax[2].set_ylabel('x')
        ax[2].set_xlim(120,-180)
        ax[2].set_ylim(0,440)
        
        for ii in range(0,3):
            Q1[ii] = ax[3].quiver(Xg_k[0,0],Xg_k[0,1],Xg_k[0,2], \
                dcmg2b_plt[0,ii,0], dcmg2b_plt[0,ii,1], dcmg2b_plt[0,ii,2],\
                label=coords[ii]+'_vector_K',length=10,color=colors[ii])
            #yz
            Q2[ii] = ax[0].quiver(Xg_k[0,1],Xg_k[0,2], \
                 -dcmg2b_plt[0,ii,1], dcmg2b_plt[0,ii,2],\
                label=coords[ii]+'_vector_K',scale=10,color=colors[ii])        
            #xz
            Q3[ii] = ax[1].quiver(Xg_k[0,0],Xg_k[0,2], \
                 dcmg2b_plt[0,ii,0], dcmg2b_plt[0,ii,2],\
                label=coords[ii]+'_vector_K',scale=10,color=colors[ii])        
            #yx
            Q4[ii] = ax[2].quiver(Xg_k[0,1],Xg_k[0,0], \
                 -dcmg2b_plt[0,ii,1], dcmg2b_plt[0,ii,0],\
                label=coords[ii]+'_vector_K',scale=10,color=colors[ii])        


        for ii,iline in enumerate(lines):
           Q[ii], =ax[3].plot(iline[it2[0],1:maxcol[ii]-2:3],iline[it2[0],2:maxcol[ii]-1:3],iline[it2[0],3:maxcol[ii]:3],color=lncolors[ii])
           Q[ii+3], =ax[0].plot(iline[it2[0],2:maxcol[ii]-1:3],iline[it2[0],3:maxcol[ii]:3],color=lncolors[ii])
           Q[ii+6], =ax[1].plot(iline[it2[0],1:maxcol[ii]-2:3],iline[it2[0],3:maxcol[ii]:3],color=lncolors[ii])
           Q[ii+9], =ax[2].plot(iline[it2[0],2:maxcol[ii]-1:3],iline[it2[0],1:maxcol[ii]-2:3],color=lncolors[ii])

        
        time_text.set_text('')
        return Q1,Q2,Q3,Q4, Q, time_text
    
    def animate(it):
        
        #Now cycle in time and plot Triad and then line if time matches 
        #for it,myt in enumerate(time_k):
        #print('it=',it)
        myt=time_k[it]
        for ii in range(0,3):
            Q1[ii].remove()
            Q1[ii]=ax[3].quiver(Xg_k[it,0],Xg_k[it,1],Xg_k[it,2], \
                   dcmg2b_plt[it,ii,0], dcmg2b_plt[it,ii,1], dcmg2b_plt[it,ii,2],\
                       label=coords[ii]+'_vector_K',length=10,color=colors[ii])
            #yz
            Q2[ii].set_offsets(np.c_[Xg_k[it,1],Xg_k[it,2]])
            Q2[ii].set_UVC(-dcmg2b_plt[it,ii,1], dcmg2b_plt[it,ii,2])        
            #xz
            Q3[ii].set_offsets(np.c_[Xg_k[it,0],Xg_k[it,2]])
            Q3[ii].set_UVC(dcmg2b_plt[it,ii,0], dcmg2b_plt[it,ii,2])        
            #yx
            Q4[ii].set_offsets(np.c_[Xg_k[it,1],Xg_k[it,0]])
            Q4[ii].set_UVC( -dcmg2b_plt[it,ii,1], dcmg2b_plt[it,ii,0])        
        
        for ii,iline in enumerate(lines):
           junk=np.vstack([ iline[it2[it],1:maxcol[ii]-2:3],iline[it2[it],2:maxcol[ii]-1:3] ])
           
           Q[ii].set_data(junk) #For Q[3] I need to make some magic to make it display 3d
           Q[ii].set_3d_properties( iline[it2[it],3:maxcol[ii]:3] )
           
           Q[ii+3].set_data(iline[it2[it],2:maxcol[ii]-1:3],iline[it2[it],3:maxcol[ii]:3])
           Q[ii+6].set_data(iline[it2[it],1:maxcol[ii]-2:3],iline[it2[it],3:maxcol[ii]:3])
           Q[ii+9].set_data(iline[it2[it],2:maxcol[ii]-1:3],iline[it2[it],1:maxcol[ii]-2:3])

        #ax[3].set_title('KFAST triads and MD lines t={:5.4f}'.format(myt))
        time_text.set_text('KFAST triads and MD lines t={:5.4f}'.format(myt))
        return Q1,Q2,Q3,Q4, Q, time_text

    print('Plotting')
    fig= plt.figure(figsize=(10, 10), dpi= 80, facecolor='w', edgecolor='k')
    
    ax[0] = fig.add_subplot(2, 2, 1)
    ax[1] = fig.add_subplot(2, 2, 2)
    ax[2] = fig.add_subplot(2, 2, 3)
    ax[3] = fig.add_subplot(2, 2, 4, projection='3d')
    time_text = ax[0].text(120, 360, '')
    date_text = ax[1].text(0, 360, datetime.datetime.now().strftime("%Y%m%d at %H:%M"))

    ani = animation.FuncAnimation(fig, animate, idx_k.size,  init_func=init_plot, interval=10)#, blit=True)
    
    if anifile !="":
        print('Saving animation...')
        ani.save(Path(wdir,anifile), writer='imagemagick')
    else:
        plt.show()

if __name__ == '__main__':
    # Sample call:
    #>>>path2kfile=Path('~/sandbox/glue-codes/kitefast/test_cases/RRD_m600_land_hiStart/KiteFast.out')
    #>>> python PlotMooring(path2kfile,decimation=10)

    #In case you want to change defaults, modify these lines, or use the --invdir/--csvdir options
    kfile_default=Path('/Users/rmudafor/Development/makani/sandbox/glue-codes/kitefast/test_cases/RRD_m600_land_hiStart/KiteFast.out')
    anif_default="" #'tether_ani.gif'

    parser=argparse.ArgumentParser(description='PLots mooring line as time series')
    parser.add_argument('--path2kfile',    metavar='path2kfile',  type=str,  help= 'path to KFAST output file', default=kfile_default)
    parser.add_argument('--decimation',    metavar='decimation',  type=int,  help= 'decimation of the main KFAST file', default=40)
    parser.add_argument('--anifile',    metavar='anifile',  type=str,  help= 'animation output file', default=anif_default)
    args=parser.parse_args()

    PlotMooring(path2kfile=args.path2kfile,anifile=args.anifile,decimation=args.decimation)
