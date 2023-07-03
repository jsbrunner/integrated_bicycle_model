# -*- coding: utf-8 -*-


#%% 1 Define parameters
bottleneck_width = 2  # values [1.0,1.5,2.0] are feasible. Otherwise it will not include the bottleneck but just the straight path.
duration = 3600
demand = [300,600,1000,1500,1000,1]
# 18'000 / 6 = 3000 [100,200,300,500,600,1000,1500,3000]

#%% 5 Test parameters
bottleneck_width = 1  # values [1.0,1.5,2.0] are feasible. Otherwise it will not include the bottleneck but just the straight path.
duration = 1000  # 60/0.2 = 300 -> 300/1 = 300
demand = [1000]
check_cyclist_id = 100000
dt = 0.5
seed = 3
demand_input = 'stochastic'

#%% 6 Test parameters
bottleneck_width = 1  # values [1.0,1.5,2.0] are feasible. Otherwise it will not include the bottleneck but just the straight path.
duration = 3600  # 1200/0.5 = 2400 -> 2400/6 = 400
demand = [150, 200, 400, 600, 550, 400]
check_cyclist_id = 100000
dt = 0.5
seed = 3
demand_input = 'stochastic'


#%% 6 Test parameters
bottleneck_width = 0  # values [1.0,1.5,2.0] are feasible. Otherwise it will not include the bottleneck but just the straight path.
duration = 600  # 1200/0.5 = 2400 -> 2400/6 = 400
demand = [600,]
check_cyclist_id = 50
dt = 0.2
seed = 3
demand_input = 'stochastic'


#%% Run model
from model import micromodel
model_df = micromodel(seed=seed,  # full list of predefined parameters in model.py
                      bottleneck_width=bottleneck_width, 
                      duration=duration,
                      demand=demand,
                      check_cyclist_id=check_cyclist_id,
                      dt=dt,
                      data_filename=0,
                      demand_input=demand_input,
                      path_width=2)  

#%% Animation visual
from figures import plot_simulation
plot_simulation(model_df, 
                bottleneck_width=bottleneck_width,
                plot_length=[50,150],
                check_cyclist_id=check_cyclist_id,
                dt=dt)

#%% Space-time diagram
from analysis import plot_space_time
plot_space_time(model_df,
                dt=dt)

#%% Fundamental diagram
from analysis import plot_fd
plot_fd(model_df, 
        duration=duration,
        agg_time=15,
        dt=dt,
        path_width=2)


#%%

'''
********************************************
*** BASE SCENARIO - STRAIGHT PATH (BS-S) ***
********************************************
'''

from model import micromodel
BS_S = micromodel(data_filename="BS_S",
                  demand=[50,100,150,200,250,275,300,325,250,150,50,0]) 
#%%
from analysis import plot_fd
plot_fd(BS_S,
        fd_filename="BS-S")
#%%
from figures import plot_simulation
plot_simulation(BS_S,
                plot_length=[80,270])
#%%
from analysis import plot_space_time
plot_space_time(BS_S,
                space_time_filename="BS-S")

#%%
'''
*****************************************
*** BASE SCENARIO - BOTTLENECK (BS-B) ***
*****************************************
'''

from model import micromodel
BS_B = micromodel(data_filename="BS_B",
                  bottleneck_width=2,
                  demand=[50,100,150,150,175,200,225,200,225,200,175,150])  # demand=[50,75,100,125,150,175,200,175,150,125,100,75] 
#%%
from analysis import plot_fd
plot_fd(BS_B,
        fd_filename="BS-B")
#%%
from figures import plot_simulation
plot_simulation(BS_B,
                plot_length=[170,270])
#%%
from analysis import plot_space_time
plot_space_time(BS_B,
                space_time_filename="BS-B")






#%%
'''
*****************************************
*** PATH WIDTH 1 (PW-1) ***
*****************************************
'''

from model import micromodel
PW_1 = micromodel(data_filename="PW_1",
                  demand=[i*0.75 for i in [50,100,150,200,250,275,300,325,250,150,50,0]],
                  path_width=1.5) 
#%%
from analysis import plot_fd
plot_fd(PW_1,
        fd_filename="PW-1",
        path_width=1.5)
#%%
from figures import plot_simulation
plot_simulation(PW_1,
                plot_length=[200,300],
                path_width=1.5)
#%%
from analysis import plot_space_time
plot_space_time(PW_1,
                space_time_filename="PW-1")
#%%
'''
*****************************************
*** PATH WIDTH 2 (PW-2) ***
*****************************************
'''

from model import micromodel
PW_2 = micromodel(data_filename="PW_2",
                  demand=[i*1.25 for i in [50,100,150,200,250,275,300,325,250,150,50,0]],
                  path_width=2.5) 
#%%
from analysis import plot_fd
plot_fd(PW_2,
        fd_filename="PW-2",
        path_width=2.5)
#%%
from figures import plot_simulation
plot_simulation(PW_2,
                plot_length=[200,300],
                path_width=2.5)
#%%
from analysis import plot_space_time
plot_space_time(PW_2,
                space_time_filename="PW-2")
#%%
'''
*****************************************
*** PATH WIDTH 3 (PW-3) ***
*****************************************
'''

from model import micromodel
PW_3 = micromodel(data_filename="PW_3",
                  demand=[i*1.35 for i in [50,100,150,200,250,275,300,325,250,150,50,0]],
                  path_width=3) 
#%%
from analysis import plot_fd
plot_fd(PW_3,
        fd_filename="PW-3",
        path_width=3)
#%%
from figures import plot_simulation
plot_simulation(PW_3,
                plot_length=[200,300],
                path_width=3)
#%%
from analysis import plot_space_time
plot_space_time(PW_3,
                space_time_filename="PW-3")






#%%
'''
*****************************************
*** SPEED DISTRIBUTION 1 (SD-1) ***
*****************************************
'''

from model import micromodel
SD_1 = micromodel(data_filename="SD_1",
                  demand=[i*1.1 for i in [50,100,150,200,250,275,300,325,250,150,50,0]],
                  v0_sd=0.2) 
#%%
from analysis import plot_fd
plot_fd(SD_1,
        fd_filename="SD-1")
#%%
from figures import plot_simulation
plot_simulation(SD_1,
                plot_length=[200,300])
#%%
from analysis import plot_space_time
plot_space_time(SD_1,
                space_time_filename="SD-1")

#%%
'''
*****************************************
*** SPEED DISTRIBUTION 2 (SD-2) ***
*****************************************
'''

from model import micromodel
SD_2 = micromodel(data_filename="SD_2",
                  demand=[i*0.8 for i in [50,100,150,200,250,275,300,325,250,150,50,0]],
                  v0_sd=1.6) 
#%%
from analysis import plot_fd
plot_fd(SD_2,
        fd_filename="SD-2")
#%%
from figures import plot_simulation
plot_simulation(SD_2,
                plot_length=[50,150])
#%%
from analysis import plot_space_time
plot_space_time(SD_2,
                space_time_filename="SD-2")








#%%
'''
*****************************************
*** PASSING THRESOLD 1 (PT-1) ***
*****************************************
'''

from model import micromodel
PT_1 = micromodel(data_filename="PT_1",
                  demand=[i*1 for i in [50,100,150,200,250,275,300,325,250,150,50,0]],
                  gamma=0.6) 
#%%
from analysis import plot_fd
plot_fd(PT_1,
        fd_filename="PT-1")
#%%
from figures import plot_simulation
plot_simulation(PT_1,
                plot_length=[0,100])
#%%
from analysis import plot_space_time
plot_space_time(PT_1,
                space_time_filename="PT-1")

#%%
'''
*****************************************
*** PASSING THRESOLD 2 (PT-2) ***
*****************************************
'''

from model import micromodel
PT_2 = micromodel(data_filename="PT_2",
                  demand=[i*1 for i in [50,100,150,200,250,275,300,325,250,150,50,0]],
                  gamma=0.95) 
#%%
from analysis import plot_fd
plot_fd(PT_2,
        fd_filename="PT-2")
#%%
from figures import plot_simulation
plot_simulation(PT_2,
                plot_length=[0,100])
#%%
from analysis import plot_space_time
plot_space_time(PT_2,
                space_time_filename="PT-2")





#%%
'''
*****************************************
*** SAFETY REGION 1 (SR-1) ***
*****************************************
'''

from model import micromodel
SR_1 = micromodel(data_filename="SR_1",
                  demand=[i*1.1 for i in [50,100,150,200,250,275,300,325,250,150,50,0]],
                  alpha=0.5,
                  beta=0.03) 
#%%
from analysis import plot_fd
plot_fd(SR_1,
        fd_filename="SR-1")
#%%
from figures import plot_simulation
plot_simulation(SR_1,
                plot_length=[200,300])
#%%
from analysis import plot_space_time
plot_space_time(SR_1,
                space_time_filename="SR-1")

#%%
'''
*****************************************
*** SAFETY REGION 2 (SR-2) ***
*****************************************
'''

from model import micromodel
SR_2 = micromodel(data_filename="SR_2",
                  demand=[i*0.9 for i in [50,100,150,200,250,275,300,325,250,150,50,0]],
                  alpha=1.4,
                  beta=0.1) 
#%%
from analysis import plot_fd
plot_fd(SR_2,
        fd_filename="SR-2")
#%%
from figures import plot_simulation
plot_simulation(SR_2,
                plot_length=[200,300])
#%%
from analysis import plot_space_time
plot_space_time(SR_2,
                space_time_filename="SR-2")
