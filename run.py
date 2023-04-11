# -*- coding: utf-8 -*-

#%%
# Use this file to execute the simulation
#%%
from model import *
from figures import *
from analysis import *
import pandas as pd

#%%
# Run model
duration = 300 # seconds of simulation time (choose 5 mins for faster testing)
model = BikeLane()
for i in range(duration): # simulation time steps
    model.step()
    ''' PROGRESS BAR (optional)
    if i % 360 == 0:
        j = int(i/360)
        progress = (j + 1) / (3600/360)
        print(f"[{'=' * j}{' ' * (10 - j)*100}] {progress}%", end='\r')
    '''
#%%
# Analyze data collection results
agent_pos = model.datacollector.get_agent_vars_dataframe() # Pandas dataframe storing the position of all bicycles at every time step
# agent_pos.head()

#%%
# reset data frame index to use current index as a column
agent_pos = agent_pos.reset_index(level=[0,1]) # reset index to make column callable
# split position tuple into x and y position
agent_pos['Position'] = agent_pos['Position'].apply(lambda pos: list(pos))
agent_pos[['Position_x', 'Position_y']] = pd.DataFrame(agent_pos['Position'].tolist(), index=agent_pos.index)
# Export and store the data on your local disk for further analysis 
agent_pos.to_csv('simulation_data.csv', sep=';')
# ...

#%%
# plot things

plot_simulation(agent_pos, save=True, filename="test_10.04.23")

