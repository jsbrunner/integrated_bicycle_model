# -*- coding: utf-8 -*-

'''
*********************************************************
*** USE THIS FILE FOR THE EXECUTION OF THE SIMULATION ***
*********************************************************
'''

from model import *
# from figures import *
import figures
from analysis import *
import pandas as pd
import random

#%% Scenario-related  parameters
random.seed(4) # Random seed for the scenario, note that for initial testings, it is better to use the same random seed so that the results are the same
duration = 60  # seconds of simulation time (choose 5 mins for faster testing)
dt = 0.2  # time step length
time_steps = int(duration/dt)  # number of time steps
Demand = [30,10]  # Inflow demand (bicycle/h), each value represents the demand of half an hour (Hence, right now this is a one hour scenario with 150 bicycles in each half-an-hour.)
path_width = 4  # path width in m (-0.5 m on each side)
path_length = ...  # path length in m

v0_mean = 5 # m/s mean for distribution of desired speed
v0_sd = 2 # m/s standard deviation of desired speed
p_mean = 1 # m distance from edge
p_sd = 0.2 # m st. deviation for distribution of p

visuals = True  # output visualization (or not)
save_visuals = True  # save visuals
data = False  # save dataframe (or not)
analysis = False  # output analysis parameters (or not)

check_cyclist_id = 4  # follow an individual cyclist to make debugging easier

#%% Run model
model = BikeLane()
for i in range(time_steps):  # simulation time steps
    model.step()
    
#%% Get variables of the model and save them into a dataframe
agent_pos = model.datacollector.get_agent_vars_dataframe()

#%% reset data frame index to use current index as a column and split position tuple into x and y position
agent_pos = agent_pos.reset_index(level=[0,1]) # reset index to make column callable
agent_pos['Position'] = agent_pos['Position'].apply(lambda pos: list(pos))
agent_pos[['Position_x', 'Position_y']] = pd.DataFrame(agent_pos['Position'].tolist(), index=agent_pos.index)

#%% Export and store the data on your local disk for further analysis 
if data:
    agent_pos.to_csv('simulation_data.csv', sep=';')

#%% Plot things
if visuals:
    figures.plot_simulation(agent_pos, save=save_visuals, filename="test_09.05.23")
    
#%%
''' 
******************************************************************
*** FOR THE PACKAGE, CALL THE RUN FUNCTION TO RUN A SIMULATION ***
******************************************************************

x Duration
x Time step length
x Path width and length
x Bicycle demand (array of length 2 for the first and for the second half of the duration)
- All behavioral parameters
x Visualization yes or no
x Data output yes or no
x Analysis output yes or no
x Random seed
- Distribution of inflow (fixed or stochastic and which distribution)

'''