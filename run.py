# -*- coding: utf-8 -*-

#%%
# Use this file to execute the simulation
#%%
from model import *
from figures import *

#%%
# Run model
model = BikeLane()
for i in range(3600): # 3600 time steps
    model.step()

#%%
# Analyze data collection results
agent_pos = model.datacollector.get_agent_vars_dataframe() # Pandas dataframe storing the position of all bicycles at every time step
agent_pos.head()

# Export and store the data on your local disk for further analysis 
# ...
