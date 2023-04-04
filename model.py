# -*- coding: utf-8 -*-


#%%
from mesa import Agent, Model
from mesa.time import SimultaneousActivation
from mesa.space import ContinuousSpace
from mesa.datacollection import DataCollector
import random
#%%
# The model.py file specifies the scenario-related parameters and design the agent and model class.

# "..." means more things to be added here


# In the current setup, all bicycles move at 4 m/s and keep a 0.5 m lateral distance with the edge.
# The bike lane is 300 m long and 2 m wide.
# Normally, the entire object (bicycle and cyclist) always stay within the 2-m-wide lane space.
# In highly congested situation or queueing situation, cyclists may utilize more lateral space to overtake or stop.
# Therefore, on both sides of the bike lane, there are 0.5-m-wide extra lateral spaces which may be occupied by the bicycle handlebar area.

#%%
# Scenario-related  parameters
random.seed(1) # Random seed for the scenario, note that for initial testings, it is better to use the same random seed so that the results are the same
Demand = [400,100] # Inflow demand (bicycle/h), each value represents the demand of half an hour (Hence, right now this is a one hour scenario with 150 bicycles in each half-an-hour.)

# In this case, we first assume bicycles are generated with a same interval (uniformly distributed) according to the demand.
# Automatically generated scenario-related  parameters
Interval = [int(3600 / Demand[i]) for i in range(len(Demand))] # Time interval in each half-an-hour
print(Interval)
Inflow_time = [] # time points that bicycles enter the bike lane
for i in range(len(Demand)):
    Inflow_time.extend(list(range(0 + 1800 * i, 1800 * (i+1), Interval[i])))
'''
Stochasticity desired for the inflow (not equal interval)
'''

#%%
# Agent class
class Bicycle(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        # Fixed attributes
        self.unique_id = unique_id
        self.length = 2 # Bicycle length, can be changed
        self.width = 0.8 # Bicycle width, can be changed
        self.desired_speed = 4.0 # should be randomized
        
        # Dynamic attributes
        self.pos = (0,1.0) # Current position, a "tuple" type position variable is required by Mesa and used in its other built-in function
        self.lat_distance_req = 0.2 # Lateral distance required, should be dynamic according to current speed
        self.speed = 4.0 # Current (actual) longitudinal speed
        self.look_ahead_dist = 50 # to be changed
        self.look_back_dist = 10 # to be changed
        # ..., e.g., desired time gap, desired acceleration (distribution?), lateral speed, angle, etc.
        self.next_coords = [0,0] # Attribute which stores the determined next coordinates
    
    # Get functions
    def getPos(self):
        return [self.pos[0],self.pos[1]]
    def getSpeed(self):
        return self.speed
    #...
    
    # Find neighbors or any individual agent which influence its own behavior (haven't been tested)
    def findLeaders(self):
        neighbors = self.model.space.get_neighbors(self.pos,self.look_ahead_dist,False)
        leaders = [l for l in neighbors if l.getPos()[0] > self.pos[0]]
        return leaders
    def findFollowers(self):
        neighbors = self.model.space.get_neighbors(self.pos,self.look_back_dist,False)
        followers = [l for l in neighbors if l.getPos()[0] < self.pos[0]]
        return followers
    # ...
    
    # Define some more functions here to help determine the behavior
    # ...
    
    # Calculate and update the attributes in the next step
    # Determine and update the next coordinates
    def calPos(self): # some more parameters to be added
        self.next_coords[0] = self.pos[0] + self.speed * 1 # to be modified
        self.next_coords[1] = self.pos[1] # to be modified
    def calSpeed(self): # some more parameters to be added
        self.speed = self.speed # to be modified
    # ...
    
    # Read surroundings and determine next coordinates after they all take actions (Note that the agent hasn't really moved when this function is called)
    def step(self):
        self.calPos()
        self.calSpeed()
        # ...
    
    # Take (physical) actions, this function would be called automatically after the step() function
    def advance(self):
        self.model.space.move_agent(self,self.next_coords) # update on the canvas
        self.pos = (self.next_coords[0],self.next_coords[1]) # update self attributes
        #print("Bicycle ",self.unique_id,"move to x = ",self.pos[0]," y = ",self.pos[1] - 0.5)
        # clear bicycles which finish the trip
        if self.pos[0] >= 300:
            self.model.to_be_removed.append(self)

#%%
# Model class
class BikeLane(Model):
    def __init__(self):
        super().__init__()
        self.schedule = SimultaneousActivation(self)
        
        # Create the canvas, which is a 300-m-long bike lane, 2 m wide with 0.5 m extra lateral spaces on both sides
        self.space = ContinuousSpace(300.1, 3, torus=False)
        
        # Initialize model variables
        self.time_step = 0
        self.inflow_count = 1 # The number of bicycle in the vertical queue that will enter
        self.n_agents = 0  # Current number of agents (bicycles) on the entire bike lane
        self.initial_coords = (0,1.0) # All bicycles enter the lane with a 0.5 m lateral distance to the right edge of the lane
        self.to_be_removed = [] # A list storing bicycles which finish the trip at the time step and to be removed
        
        # Data collection functions, collect positions of every bicycle at every step, namely trajectories
        self.datacollector = DataCollector(agent_reporters={"Position": "pos", "Speed": "speed"})
    
    def deduct(self):
        self.n_agents = self.n_agents - 1
    
    def step(self):
        # Execute agents' functions, including both step and advance
        self.schedule.step()
        # Remove out of bound agents
        for b in self.to_be_removed:
            #print("Remove Bicycle ",b.unique_id)
            self.schedule.remove(b)
            self.space.remove_agent(b)
        self.deduct() # reduce n_agents by 1
        self.to_be_removed = []
        # Add bicycle agents at certain time steps
        if self.inflow_count < len(Inflow_time):
            if self.time_step == Inflow_time[self.inflow_count]:
                b = Bicycle(self.inflow_count, self)
                self.schedule.add(b)
                self.space.place_agent(b, self.initial_coords)
                self.inflow_count += 1
                self.n_agents += 1
        # Update the time
        self.time_step += 1
        # Execute data collector
        self.datacollector.collect(self)
    

#%%




