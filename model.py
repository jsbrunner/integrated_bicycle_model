# -*- coding: utf-8 -*-


#%%
from mesa import Agent, Model
from mesa.time import SimultaneousActivation
from mesa.space import ContinuousSpace
from mesa.datacollection import DataCollector
import random
import math
#%%

''' Instructions Ying-Chuan '''
# The model.py file specifies the scenario-related parameters and design the agent and model class.

# "..." means more things to be added here

# In the current setup, all bicycles move at 4 m/s and keep a 0.5 m lateral distance with the edge.
# The bike lane is 300 m long and 2 m wide.
# Normally, the entire object (bicycle and cyclist) always stay within the 2-m-wide lane space.
# In highly congested situation or queueing situation, cyclists may utilize more lateral space to overtake or stop.
# Therefore, on both sides of the bike lane, there are 0.5-m-wide extra lateral spaces which may be occupied by the bicycle handlebar area.

#%%

''' 
********************
*** MODEL INPUTS ***
********************
''' 
# write them as inputs to the model later

# Scenario-related  parameters (inputs may be changed when calling the library's functions)
random.seed(4) # Random seed for the scenario, note that for initial testings, it is better to use the same random seed so that the results are the same
# print(random.gauss(4, 2))
Demand = [300,75] # Inflow demand (bicycle/h), each value represents the demand of half an hour (Hence, right now this is a one hour scenario with 150 bicycles in each half-an-hour.)

v0_mean = 4 # m/s mean for distribution of desired speed
v0_sd = 1 # m/s standard deviation of desired speed
v_lat_max = 0.5 # m/s maximum lateral speed

p_mean = 1 # m distance from edge
p_sd = 0.2 # m st. deviation for distribution of p

a_des = 1.4 # feasible relaxation time for acceleration
b_max = 1 # m/s**2 maximum braking force 
''' Relaxation time for braking -> look what the NDM needs '''

dt = 1 # simulation time step length in seconds 

path_width = 4 # path width in m (-0.5 m on each side)

# In this case, we first assume bicycles are generated with a same interval (uniformly distributed) according to the demand.
# Automatically generated scenario-related  parameters
Interval = [int(300 / Demand[i]) for i in range(len(Demand))] # Time interval in each half-an-hour
# print(Interval)
Inflow_time = [] # time points that bicycles enter the bike lane
for i in range(len(Demand)):
    Inflow_time.extend(list(range(0 + 150 * i, 150 * (i+1), Interval[i])))
''' Stochasticity desired for the inflow (not equal interval) '''

''' make a variable for the time step length, in case we want to change it later '''

#%%
# Agent class
class Bicycle(Agent):
    
    ''' 
    ************************************
    *** INITIALIZATION AND VARIABLES ***
    ************************************
    '''
    
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        
        # Fixed attributes
        self.unique_id = unique_id
        self.length = 2 # bicycle length
        self.width = 0.8 # bicycle width
        self.v0 = random.gauss(v0_mean, v0_sd) # distribution of desired speed
        self.p = random.uniform(p_mean-p_sd, p_mean+p_sd) # distribution of desired lateral position
        self.a_des = a_des # fixed value for the desired/feasible acceleration
        self.b_max = b_max # fixed value for the maximum deceleration/braking
        self.omega_max = 1.0 # m/s fixed value for the maximum lateral speed
        self.omega_des = 0.2 # m/s fixed value for the desired lateral speed
        # further:
        self.zeta = ... # parameter for lateral stabilization with speed
        self.alpha = ... # scale length of safety region
        self.beta = ... # scale width of safety region
        self.gamma = 0.95 # passing threshold
        self.lambd = ... # coefficient for consideration range (caution with the var name)
        # OPTIONAL
        self.phi = ... # length of necessary 'spatial gain'
        self.eta = ... # coefficient for backward view
        self.psi = ... # coefficient for far border of anticipation range
        
        # Dynamic attributes (these following values initialize the simulation)
        self.pos = (0,self.p) # Current position, a "tuple" type position variable is required by Mesa and used in its other built-in function
        self.speed = self.v0 # Current (actual) longitudinal speed
        self.balance = 0 # additional width for stabilization (add value on both sides)
        self.sr_width = 0.2 # width of the safety region
        self.sr_length = 6 # length of the safety region
        self.cr_length = 20 # consideration range length
        self.ar_length = 0 # length of the anticipation range
        self.bw_length = 10 # length of the backwards view
        self.acc = 0 # actual longitudinal acceleration/braking for the current time step
        self.v_lat = 0 # actual lateral speed for the current time step
        self.next_coords = [0,0] # Attribute which stores the determined next coordinates
        self.cat1_cyclists = [] # list of significantly slower cyclists in consideration range
        self.cat2_cyclists = [] # list of slightly slower cyclists in consideration range
        self.blocked_space_indiv = [] # auxiliary list used across level 1 and 2
        self.des_lat_pos = 0 # desired lateral position
        self.trajectory = [] # list including the coordinates for the desired path (therefore also implicitly the moving angle)
        self.leader = 0
        self.v0_density = self.v0 # would account for the perception of density/congestion downstream
    
    
    ''' 
    *********************
    *** GET FUNCTIONS ***
    *********************
    '''
    
    def getPos(self):
        return [self.pos[0],self.pos[1]]
    def getSpeed(self):
        return self.speed
    #...more functions may be necessary
    
    
    ''' 
    ***************************
    *** AUXILIARY FUNCTIONS ***
    ***************************
    '''
    
    # Find neighbors or any individual agent which influence its own behavior (haven't been tested)
    ''' Functions by Ying-Chuan
    def findLeaders(self):
        neighbors = self.model.space.get_neighbors(self.pos,self.look_ahead_dist,False)
        leaders = [l for l in neighbors if l.getPos()[0] > self.pos[0]]
        return leaders '''

    '''
    def findFollowers(self):
        neighbors = self.model.space.get_neighbors(self.pos,self.look_back_dist,False)
        followers = [l for l in neighbors if l.getPos()[0] < self.pos[0]]
        return followers
    '''
    
    def findCat1(self):
        neighbors = self.model.space.get_neighbors(self.pos,self.cr_length+10,False) # add 10 metres so that the circular radius does not matter anymore
        leaders = [l for l in neighbors if l.getPos()[0] > self.pos[0] and l.getPos()[0] < (self.pos[0]+self.cr_length)] # obtain cyclists in consideration range
        self.cat1_cyclists = [l for l in leaders if l.getSpeed() <= (self.gamma*self.v0)] # obtain cat1 cyclists
    
    def findCat2(self):
        ...
    
    '''
    def findClosestLeader(self):
        ...
        # 1. get_neighbors as in functions before (but with longer distance than fov)
        # 2. reduce list to the neighbors in front (and within fov)
        # 3. loop over list and find the closest agent
        # 4. return closest agent
        # leaders = self.findLeaders()
        # get closest leader
        # print(leaders)
    '''    
    
    
    ''' 
    ************************
    *** UPDATE FUNCTIONS ***
    ************************
    '''
    
    # Calculate and update the attributes in the next step
    # Determine and update the next coordinates
    def calPos(self): # some more parameters to be added
        self.next_coords[0] = self.pos[0] + self.speed * dt # to be modified
        self.next_coords[1] = self.des_lat_pos # self.p # self.pos[1] # to be modified
    def calSpeed(self): # some more parameters to be added
        self.speed = self.v0 # to be modified
    
    
    ''' 
    ***********************
    *** LEVEL FUNCTIONS ***
    ***********************
    '''
    
    ''' LEVEL 1: Desired lateral position '''
    def findLatPos(self): 
        # find cat1 cyclists in consideration range
        self.findCat1()
        # print(self.cat1_cyclists)
        if len(self.cat1_cyclists)==0: # if there is no cat. 1 cyclist in the consideration range
            self.des_lat_pos = self.p # just go to the desired lateral position
            # print("no cat. 1 leader")
        else:
            self.des_lat_pos = 2 # random.uniform(1,3)
            # print(self.des_lat_pos)
            # obtain envelope, idea is maybe outdated
            self.blocked_space_indiv = [] # list the touples with lateral positions of cat1 cyclists
            unblocked_space = []
            
            for i in self.cat1_cyclists:
                self.blocked_space_indiv.append((i, i.getPos()[1]-0.4, i.getPos()[1]+0.4)) # change 0.4 to the actual width including stabilization
                
            # print('\nunsorted\n', self.blocked_space_indiv)
            self.blocked_space_indiv.sort(key=lambda a: a[2], reverse=True) # sorted cyclists from left to right
            # print('\nsorted\n', self.blocked_space_indiv)
            
            gap_found = False # boolean to terminate the following loop
            
            while gap_found==False:
                
                # find gaps between cyclists
                for i in range(len(self.blocked_space_indiv)):
                    if i==0: # if it is the first cyclist from the left
                        unblocked_space.append([path_width, self.blocked_space_indiv[i][2], round(path_width-self.blocked_space_indiv[i][2],2)])
                    elif self.blocked_space_indiv[i-1][1] <= self.blocked_space_indiv[i][2]: # if the projection overlaps, there is not an additional unblocked space
                        continue
                    else: # add an additional unblocked space
                        unblocked_space.append([self.blocked_space_indiv[i-1][1], self.blocked_space_indiv[i][2], round(self.blocked_space_indiv[i-1][1]-self.blocked_space_indiv[i][2],2)])
                unblocked_space.append([self.blocked_space_indiv[-1][1], 0, round(self.blocked_space_indiv[-1][1],2)]) # add a final gap towards the right edge of the path
                # print('\ngaps\n', unblocked_space)
                
                # check if there is a gap big enough to fit (left to right priority, list is already sorted alike)
                for i in unblocked_space:
                    if i[2] >= self.width+0.2: # change 0.2 to the correct width including safety region later
                        self.des_lat_pos = i[1]+0.8 # (i[0]+i[1])/2 # desired position is the middle of the gap (could be a better formula, but for now it is ok)
                        ''' ONE COULD MAKE AN AUXILIARY FUNCTION FOR THIS, BC THE POSITION IN THE GAP IS USED AGAIN IN LVL 2
                        MAKE SURE THAT THE P VALUE IS USED HERE AS WELL TO BRING HETEROGENEITY '''
                        gap_found = True
                        break
                if gap_found==True:
                    break
                
                # remove cyclist the furthest downstream if gap is not found
                furthest_agent_pos = 0
                furthest_agent = ...
                for i in range(len(self.blocked_space_indiv)):
                    if self.blocked_space_indiv[i][0].getPos()[0] > furthest_agent_pos:
                        furthest_agent_pos = self.blocked_space_indiv[i][0].getPos()[0]
                        furthest_agent = i    
                del self.blocked_space_indiv[furthest_agent]
                # if there is one cyclist blocking the whole path and you delete him it is strange what happens


    ''' LEVEL 2: Moving angle and leader '''
    def findTraj(self):  
        
        req_lat_move = self.des_lat_pos - self.getPos()[1] # desired position minus actual position -> gives direction left or right directly
        
        if len(self.cat1_cyclists)==0: # handle case with no slower cyclists (it is only the remaining cyclists )
            if abs(req_lat_move) < self.omega_des:
                self.v_lat = req_lat_move
            else:
                if req_lat_move < 0: # handle whether to move right or left
                    self.v_lat = -self.omega_des
                else:
                    self.v_lat = self.omega_des
        
        else: # case with slower cyclists remaining; self.blocked_space_indiv[][0]
            obstr_cyclists = [] # potentially obstructing from reaching desired position
            remove_indices = []
            for i in range(len(self.blocked_space_indiv)):
                obstr_cyclists.append(self.blocked_space_indiv[i][0]) # append cyclist object from the remaining cat. 1 cyclists
            for i in range(len(obstr_cyclists)): # find non-obstructing cyclists
                if req_lat_move < 0:
                    if obstr_cyclists[i].getPos()[1] > self.getPos()[1] or obstr_cyclists[i].getPos()[1] < self.des_lat_pos:
                        #remove non-dominant cyclist
                        remove_indices.append(i)
                else:
                    if obstr_cyclists[i].getPos()[1] < self.getPos()[1] or obstr_cyclists[i].getPos()[1] > self.des_lat_pos:
                        remove_indices.append(i)
            
            # print('\n\nCat. 1 cyclists: ', self.cat1_cyclists)
            # print('\nSpace blockers before removal: ', obstr_cyclists)
            # remove non-dominant cycists
            for i in sorted(remove_indices, reverse=True):
                del obstr_cyclists[i]
            # print('\nSpace blockers after removal: ', obstr_cyclists)
            
            # if there is no obstructing cyclist, do the same as above and go towards the desired position at the end of the CR
            if len(obstr_cyclists)==0: # handle case with no obstructing cyclists (it is only the remaining cyclists )
                if abs(req_lat_move) < self.omega_des:
                    self.v_lat = req_lat_move
                else:
                    if req_lat_move < 0: # handle whether to move right or left
                        self.v_lat = -self.omega_des
                    else:
                        self.v_lat = self.omega_des
                        
            else: # if there are obstructing cyclists, project these cyclists to when you would pass
                # obtain the distance until you pass the obstructing cyclists
                # work with obstr_cyclist list and add distance to pass
                # the calculation now is a bit more advanced
                for i in range(len(obstr_cyclists)):
                    # calc dist to passing point
                    p1 = self.getPos()[0]
                    v1 = self.getSpeed()
                    p2 = obstr_cyclists[i].getPos()[0]
                    v2 = obstr_cyclists[i].getSpeed()
                    time_to_pass = (p2-p1)/(v1-v2)
                    dist_to_pass = v1*time_to_pass
                    
                    # calc lateral passing point
                    lat_passing_point = 0
                    if req_lat_move < 0:
                        lat_passing_point = obstr_cyclists[i].getPos()[1]-1 # one meter to the right of the center
                    else:
                        lat_passing_point = obstr_cyclists[i].getPos()[1]+1 # one meter to the right of the center
                    
                    # calc moving angle (which angle is the absolute steepest)
                    angle_temp = math.atan2((lat_passing_point-self.getPos()[1]), dist_to_pass)
                    obstr_cyclists[i] = [obstr_cyclists[i], abs(angle_temp), dist_to_pass, lat_passing_point-self.getPos()[1]]
                
                print('\n', obstr_cyclists)
                # go for the steepest angle (which angle is the absolute steepest)
                relevant_cyc_index = 0
                steepest_angle_temp = 0
                for i in range(len(obstr_cyclists)):
                    if obstr_cyclists[i][1] > steepest_angle_temp:
                        steepest_angle_temp = obstr_cyclists[i][1]
                        relevant_cyc_index = i

                # check if the angle is feasible with the maximum lateral speed
                max_angle_cur = abs(math.atan2(self.omega_max, self.getSpeed()))  # maximum angle at current speed would be
                print(steepest_angle_temp)
                print(max_angle_cur)
                # go the maximum lateral speed or the required speed 
                
        # find the leader (with basically the easy formula)
        # project all cat. 1 and cat. 2 cyclists one step further
        # obtain closest of those inside the shape
                
            
    ''' LEVEL 3: Acceleration according to NDM '''
    def findAcc(self): 
        ...
    
    
    
    ''' 
    **********************************
    *** STEP AND ADVANCE FUNCTIONS ***
    **********************************
    '''
    
    # Read surroundings and determine next coordinates after they all take actions (Note that the agent hasn't really moved when this function is called)
    def step(self):
        ''' CALL LEVEL FUNCTIONS '''
        self.findLatPos() # model level 1
        self.findTraj() # moving angle and identify leader, level 2
        # self.findAcc() # level 3
        
        ''' CALL UPDATE FUNCTIONS '''
        self.calPos()
        self.calSpeed()
        # update regions:
        # self.calSize()
        # self.calFOV()
        # self.calSafetyRegion()
        # 
        
    
    # Take (physical) actions, this function would be called automatically after the step() function
    def advance(self):
        self.model.space.move_agent(self,self.next_coords) # update on the canvas
        self.pos = (self.next_coords[0],self.next_coords[1]) # update self attributes
        # print("Bicycle ",self.unique_id,"move to x = ",self.pos[0]," y = ",self.pos[1] - 0.5)
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
        self.space = ContinuousSpace(300.1, path_width, torus=True) # Changed the torus=False here: otherwise, there will be an error because agents are 'out of bounds'
        
        # Initialize model variables
        self.time_step = 0
        self.inflow_count = 1 # The number of bicycle in the vertical queue that will enter
        self.n_agents = 0  # Current number of agents (bicycles) on the entire bike lane
        self.initial_coords = (0,1) # bicycles enter 0.5 m from the path edge
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