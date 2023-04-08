# -*- coding: utf-8 -*-

# this script is 
from model import *
from run import *

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#%%

# plot_simulation(agent_pos, save=False, filename="test")
# run standard plot
def plot_simulation(agent_pos, save, filename): # more input relevant for different plots
    
    # Params for figure
    anim_interval = 200 # ms to update (e.g. 200 is 5 fps)
    plot_length = [0,100] # metres from the beginning to the end of the path (x_lim)
    
    # create figure
    fig, ax = plt.subplots(figsize=(20,2))
    
    # animation function
    def animate(frame):
        
        # clear canvas
        ax.clear()
        
        # set the boundaries of the plot
        ax.set_xlim([plot_length[0],plot_length[1]])
        ax.set_ylim([0,3])
        # rename the y-tick labels
        ax.set_yticks([0, 0.5, 1, 1.5, 2, 2.5, 3], [-0.5, 0, 0.5, 1, 1.5, 2, 2.5])
        # draw the edges of the cycle path
        ax.plot([0,300], [0.5,0.5], color='grey', zorder=1)
        ax.plot([0,300], [2.5,2.5], color='grey', zorder=1)
        # draw the keep-right position p
        ax.plot([0,300], [0.8,0.8], color='khaki', zorder=1)
        ax.plot([0,300], [1.2,1.2], color='khaki', zorder=1)
        
        # obtain agents visible in the current simulation step
        agent_pos_frame = agent_pos[agent_pos['Step'] == frame]
        # obtain their position
        x_pos = agent_pos_frame['Position_x']
        y_pos = agent_pos_frame['Position_y']
        # plot centers of cyclists in the simulation step
        ax.scatter(x_pos, y_pos, color='black', zorder=2)
        # plot diamond shaped size of cyclists
        ax.plot([x_pos-1,x_pos,x_pos+1,x_pos,x_pos-1], [y_pos,y_pos+0.4,y_pos,y_pos-0.4,y_pos], color='black', zorder=2)
        
        # get minutes and seconds from the simulation step
        minutes = int(frame/60)
        seconds = frame % 60
        
        # naming the plot
        ax.set_title(f'{filename}: Simulation time {minutes:02}:{seconds:02} (step {frame})')
        ax.set_xlabel('Cycle path length (m)')
        ax.set_ylabel('Cycle path width (m)')
        
    # matplotlib animation function
    ani = animation.FuncAnimation(fig, animate, agent_pos['Step'].unique(), interval=anim_interval)
    fig.tight_layout(pad=2)
    
    # save animation as gif into current directory
    if save:
        writer = animation.PillowWriter(fps=5, metadata=dict(artist='Me'), bitrate=1000)
        ani.save("{}.gif".format(filename), writer=writer)

    fig.show()
