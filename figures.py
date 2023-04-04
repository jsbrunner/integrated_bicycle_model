# -*- coding: utf-8 -*-

# this script is 
from model import *
from run import *

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#%%

# reset data frame index to use current index as a column
agent_pos = agent_pos.reset_index(level=[0,1]) # reset index to make column callable

#%%

# split position tuple into x and y position
agent_pos['Position'] = agent_pos['Position'].apply(lambda pos: list(pos))
agent_pos[['Position_x', 'Position_y']] = pd.DataFrame(agent_pos['Position'].tolist(), index=agent_pos.index)
# print(agent_pos)

#%%

# create figure
fig, ax = plt.subplots(figsize=(20,2))

# animation function
def animate(frame):
    
    # clear canvas
    ax.clear()
    
    # obtain agents visible in the current simulation step
    agent_pos_frame = agent_pos[agent_pos['Step'] == frame]
    # obtain their position
    x_pos = agent_pos_frame['Position_x']
    y_pos = agent_pos_frame['Position_y']
    # plot centers of cyclists in the simulation step
    ax.scatter(x_pos, y_pos, color='black')
    # plot diamond shaped size of cyclists
    ax.plot([x_pos-1,x_pos,x_pos+1,x_pos,x_pos-1], [y_pos,y_pos+0.4,y_pos,y_pos-0.4,y_pos], color='black')
    
    # set the boundaries of the plot
    ax.set_xlim([0,300])
    ax.set_ylim([0,3])
    # rename the y-tick labels
    ax.set_yticks([0, 0.5, 1, 1.5, 2, 2.5, 3], [-0.5, 0, 0.5, 1, 1.5, 2, 2.5])
    # draw the edges of the cycle path
    ax.plot([0,300], [0.5,0.5], color='lightgrey')
    ax.plot([0,300], [2.5,2.5], color='lightgrey')
    
    # get minutes and seconds from the simulation step
    minutes = int(frame/60)
    seconds = frame % 60
    
    # naming the plot
    ax.set_title(f'Simulation time {minutes:02}:{seconds:02} (step {frame})')
    ax.set_xlabel('Cycle path length (m)')
    ax.set_ylabel('Cycle path width (m)')
    
# matplotlib animation function
ani = animation.FuncAnimation(fig, animate, agent_pos['Step'].unique(), interval=20)
fig.tight_layout(pad=2)

plt.show()

# save animation as gif into current directory
# writer = animation.PillowWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
# ani.save('scatter.gif', writer=writer)


