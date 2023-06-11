# -*- coding: utf-8 -*-

from model import micromodel
from figures import plot_simulation
from analysis import plot_space_time
from analysis import plot_fd
from analysis import collisions


#%% Run model
model_df = micromodel(seed=1)  # full list of predefined parameters in model.py

#%% Animation visual
plot_simulation(model_df)

#%% Space-time diagram
plot_space_time(model_df)

#%% Fundamental diagram
plot_fd(model_df)

#%% Collisions
collisions(model_df)
    
