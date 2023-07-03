# A New Microscopic Bicycle Simulation Model Considering Non-lane-based Traffic Characteristics
Johannes S. Brunner <br />
ETH Zurich <br />
July 2023 <br />

## Required python packages
- Mesa
- Matplotlib
- Pandas
- Math
- Random
- Statsmodels

## Run the simulation
The functions are provided with the standard model parameters that work well. Note, that the parameters need to be consistent across all of the following functions. The model might not work as expected when parameters are modified.
```
from model import micromodel
model = micromodel(seed = 4,  # random seed
               duration = 3600,  # simulation duration (s) 3600/12 = 300 s
               dt = 0.2,  # simulation time step length (s)
               demand = [50,100,150,200,250,300,350,400,300,200,100,50],  # list with demands []
               path_width = 2,  # width of the simulated path (m, excl. 2x 0.5 m space on side of the path); put 3 m or less for the bottleneck to work
               bottleneck_width = 0,  # (m); put numbers [1.0,1.5,2.0] for the bottleneck to be active; all other values mean that the bottleneck is not active
               v0_mean = 4.5,  # mean of desired speed (m/s)
               v0_sd = 1,  # standard deviation of desired speed (m/s)
               p_mean = 1,  # mean of desired lateral position / distance from right edge +0.5 (m)
               p_sd = 0.2,  # standard deviation for desired lateral position (m)
               check_cyclist_id = -1,  # follow the choices of an individual cyclist with his unique_id; put -1 for no output
               b_length = 2,  # bicycle length (m)
               b_width = 0.8,  # bicycle width (m)
               d_standing = 0.1,  # minimum standing distance to other cyclists (m)
               a_des = 1.5,  # relaxation time for acceleration (s)
               b_max = 2,  # maximum braking (m/s^2, >0)
               omega_max = 0.3,  # maximum lateral speed (m/s)
               omega_des = 0.15,  # desired lateral speed (m/s)
               d_omega_max = 0.2,  # maximum lateral acceleration (m/s^2)
               phi = 4,  # scale length of the consideration range (-)
               alpha = 0.8,  # scale length of the safety region (-)
               beta = 0.05,  # scale width of the safety region (-)
               gamma = 0.85,  # passing threshold (-)
               lookback = 1,  # proportion of cyclists looking back before moving laterally [0,1]
               side_obstacle = 0.2,  # width deducted from both sides of the extended path to simulate obstacles (m)
               data_filename = "simulation_data",  # type 0 if file should not be saved
               demand_input = 'stochastic')  # 'fixed' for fixed interval inflow
```
## Plot the interactive animation
```
from figures import plot_simulation
plot_simulation(model, 
                    dt = 0.2,  # time step length (s)
                    path_width = 2,
                    bottleneck_width = 0,
                    anim_interval = 500, # time to update (ms); 500 ms = 2 FPS
                    plot_length = [0,300],  # start and end of space to show the simulation (m)
                    check_cyclist_id = -1, 
                    animation_filename = "animation")
```
## Create fundamental diagram
```
from analysis import plot_fd
plot_fd(model, 
            dt = 0.2,  # time step length (s)
            duration = 3600,  # simulation duration (s)
            agg_time = 15,  # aggregation interval for fundamental diagram (s)
            agg_dist = [100, 250],  # aggregation distance for fundamental diagram (min and max value in m)
            path_width = 2,
            fd_filename = "fundamental_diagram")
```
## Create space-time diagram
```
from analysis import plot_space_time
plot_space_time(model, 
                    dt = 0.2,
                    space_time_filename = 'space_time')
```
