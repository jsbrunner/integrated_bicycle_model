# -*- coding: utf-8 -*-

'''
**************************************************
*** MODEL FILE (DO NOT CHANGE PARAMETERS HERE) ***
**************************************************
'''

#%%
from mesa import Agent, Model
from mesa.time import SimultaneousActivation
from mesa.space import ContinuousSpace
from mesa.datacollection import DataCollector
import random
import math
import run

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


# In this case, we first assume bicycles are generated with a same interval (uniformly distributed) according to the demand.
Interval = [int(run.time_steps*0.5 / run.Demand[i]) for i in range(len(run.Demand))] # Time interval in both of the two slots
print(Interval)
Inflow_step = [] # time points that bicycles enter the bike lane
for i in range(len(run.Demand)):
    Inflow_step.extend(list(range(0 + int(run.time_steps/2) * i, int(run.time_steps/2) * (i+1), Interval[i])))
print(Inflow_step)
''' Stochasticity desired for the inflow (not always equal interval) '''


#%% Agent class

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
        self.length = 2  # bicycle length
        self.width = 0.8  # bicycle width
        self.v0 = random.uniform(run.v0_mean - run.v0_sd, run.v0_mean + run.v0_sd)  # distribution of desired speed
        self.p = random.uniform(run.p_mean - run.p_sd, run.p_mean + run.p_sd)  # distribution of desired lateral position
        self.a_des = 2  # feasible relaxation time for acceleration
        self.b_max = 2  # m/s**2 maximum braking force (positive value)
        self.omega_max = 0.5  # m/s fixed value for the maximum lateral speed
        self.omega_des = 0.2  # m/s fixed value for the desired lateral speed
        # further:
        self.zeta = 0.4  # parameter for lateral stabilization with speed (maximum add. width for stabilization in m)
        self.alpha = 1  # scale length of safety region
        self.beta = 0.05  # scale width of safety region
        self.gamma = 0.85  # passing threshold
        self.phi = 3 # coefficient for consideration range (caution with the var name)
        # OPTIONAL
        # self.phi = ...  # length of necessary 'spatial gain'
        self.eta = ...  # coefficient for backward view
        self.psi = ...  # coefficient for far border of anticipation range
        
        # Dynamic attributes (these following values initialize the simulation)
        self.pos = (0,self.p)  # Current position, a "tuple" type position variable is required by Mesa and used in its other built-in function
        self.speed = self.v0  # Current (actual) longitudinal speed
        self.balance = 0  # additional width for stabilization (add value on both sides)
        self.sr_width = 0.2  # width of the safety region
        self.sr_length = 10  # length of the safety region
        self.cr_length = 20  # consideration range length
        self.ar_length = 0  # length of the anticipation range
        self.bw_length = 20  # length of the backwards view
        self.acceleration = 0  # actual longitudinal acceleration/braking for the current time step
        self.v_lat = 0  # actual lateral speed for the current time step
        self.next_coords = (0,self.p)  # Attribute which stores the determined next coordinates
        self.next_speed = 0
        self.cat1_cyclists = []  # list of significantly slower cyclists in consideration range
        self.cat12_cyclists = []  # list of slightly slower cyclists in consideration range
        self.cat3_behind = []  # list of faster cyclists in the backward view
        self.blocked_space_indiv = []  # auxiliary list used across level 1 and 2
        self.des_lat_pos = 0  # desired lateral position
        self.trajectory = []  # list including the coordinates for the desired path (therefore also implicitly the moving angle)
        self.leader = 0  # variable to save leading cyclist's object id
        self.v0_density = self.v0  # would account for the perception of density/congestion downstream
        self.cut_off_flag = False  # True if cyclist would cut-off somebody else
    
    
    ''' 
    *********************
    *** GET FUNCTIONS ***
    *********************
    '''
    # get position of a bicycle object
    def getPos(self):
        '''
        if self.pos is not None: # it is strange that some positions are None???
            return [self.pos[0],self.pos[1]]
        else:
            return [0,0]''' # there have been None-value objects
        return [self.pos[0],self.pos[1]]
    
    # get speed of a bicycle object
    def getSpeed(self):
        return self.speed
    
    
    ''' 
    ***************************
    *** AUXILIARY FUNCTIONS ***
    ***************************
    '''
    
    
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
    
    def findCat12(self):
        neighbors = self.model.space.get_neighbors(self.pos,self.cr_length+10,False) # add 10 metres so that the circular radius does not matter anymore
        leaders = [l for l in neighbors if l.getPos()[0] > self.pos[0] and l.getPos()[0] < (self.pos[0]+self.cr_length)] # obtain cyclists in consideration range
        self.cat12_cyclists = [l for l in leaders if l.getSpeed() <= (self.v0)] # obtain cat1 and cat2 cyclists
    
    def findCat3Behind(self):  # not really cat 3 but the cyclists that are currently faster than you are
        neighbors = self.model.space.get_neighbors(self.pos,self.bw_length+10,False)
        followers = [l for l in neighbors if l.getPos()[0] < self.pos[0] and l.getPos()[0] > (self.pos[0]-self.bw_length)] # obtain cyclists in backward view
        self.cat3_behind = [l for l in followers if l.getSpeed() > (self.getSpeed())] # obtain cat3 cyclists
    
    ''' 
    ************************
    *** UPDATE FUNCTIONS ***
    ************************
    '''
    
    # Calculate and update the attributes in the next step
    # Determine and update the next coordinates
    def calPos(self): 
        self.next_coords = (self.pos[0] + self.speed*run.dt + self.acceleration*0.5*run.dt**2, self.pos[1] + self.v_lat*run.dt)  # new x and y position values
    
    # Determine and update the next speed
    def calSpeed(self):
        self.next_speed = self.speed + self.acceleration * run.dt # apply acceleration from ndm
        
    def updateSize(self):
        self.width = 0.8  # 0.8 + (self.zeta-self.zeta*(self.next_speed/(2+self.next_speed)))
        ''' describe this formula in more detail '''
    
    def updateCR(self):
        self.cr_length = self.phi*self.next_speed
    
    def updateSR(self):
        self.sr_length = self.length/2 + self.alpha*self.next_speed
        self.sr_width = self.width/2 + self.beta*self.next_speed
    
    ''' 
    ***********************
    *** LEVEL FUNCTIONS ***
    ***********************
    '''
    
    ''' LEVEL 1: Desired lateral position '''
    def findLatPos(self): 
        if self.unique_id==run.check_cyclist_id: print("\n****** LEVEL 1: LATERAL POSITION ******")
        # find cat1 cyclists in consideration range
        self.findCat1()
        # print(self.cat1_cyclists)
        # if there is no cat. 1 cyclist in the consideration range
        if len(self.cat1_cyclists)==0:  
            self.des_lat_pos = self.p  # just go to the desired lateral position
            if self.unique_id==run.check_cyclist_id: print("No cat. 1 leader, des_lat_pos={}".format(round(self.des_lat_pos,2)))
        else:
            # self.des_lat_pos = 2  # is this necessary? # random.uniform(1,3)
            # print(self.des_lat_pos)
            # obtain envelope, idea is maybe outdated
            self.blocked_space_indiv = []  # empty list the touples with lateral positions of cat1 cyclists
            unblocked_space = []  # will contain the borders and width of the lateral gap(s)
            
            # obtain lateral positions blocked by cat. 1 cyclists in consideration range
            for i in self.cat1_cyclists:
                self.blocked_space_indiv.append((i, i.getPos()[1]-0.4, i.getPos()[1]+0.4))  # change 0.4 to the actual width including stabilization
                
            # print('\nunsorted\n', self.blocked_space_indiv)
            self.blocked_space_indiv.sort(key=lambda a: a[2], reverse=True)  # sort cyclists from left to right
            # print('\nsorted\n', self.blocked_space_indiv)
            
            # boolean to terminate the following loop (searching for a wide-enough lateral gap)
            gap_found = False
            
            while gap_found==False:
                
                # if the path is narrow so that the only blocking cyclist is removed, there needs to be a criterion
                if len(self.blocked_space_indiv)==0:
                    unblocked_space.append([run.path_width,0,run.path_width])
                    self.des_lat_pos = self.p
                    break
                
                # find gaps/unblocked spaces between cyclists
                for i in range(len(self.blocked_space_indiv)):
                    # if it is the first cyclist from the left
                    if i==0: 
                        unblocked_space.append([run.path_width, self.blocked_space_indiv[i][2], round(run.path_width-self.blocked_space_indiv[i][2],2)])  # also add width of the gap 
                    elif self.blocked_space_indiv[i-1][1] <= self.blocked_space_indiv[i][2]:  # if the projection overlaps, there is not an additional unblocked space
                        continue
                    else: # add an additional unblocked space
                        unblocked_space.append([self.blocked_space_indiv[i-1][1], self.blocked_space_indiv[i][2], round(self.blocked_space_indiv[i-1][1]-self.blocked_space_indiv[i][2],2)])
                unblocked_space.append([self.blocked_space_indiv[-1][1], 0, round(self.blocked_space_indiv[-1][1],2)])  # add a final gap towards the right of the path
                # print('\ngaps\n', unblocked_space)
                
                # check if there is a gap big enough to fit, including safety region (left to right priority, list is already sorted alike)
                for i in unblocked_space:
                    if i[2] >= self.width+0.4:  # change 0.2 to the correct width including safety region later
                        self.des_lat_pos = i[1]+0.8  # (i[0]+i[1])/2 # desired position is the middle of the gap (could be a better formula, but for now it is ok)
                        ''' ONE COULD MAKE AN AUXILIARY FUNCTION FOR THIS, BC THE POSITION IN THE GAP IS USED AGAIN IN LVL 2
                        MAKE SURE THAT THE P VALUE IS USED HERE AS WELL TO BRING HETEROGENEITY '''
                        gap_found = True
                        break
                if gap_found==True:
                    break
                
                # remove cyclist the furthest downstream if no gap is found
                furthest_agent_pos = 0
                furthest_agent = ...
                for i in range(len(self.blocked_space_indiv)):
                    if self.blocked_space_indiv[i][0].getPos()[0] > furthest_agent_pos:
                        furthest_agent_pos = self.blocked_space_indiv[i][0].getPos()[0]
                        furthest_agent = i
                
                # delete furthest downstream cyclist from the list of blocking cyclists
                del self.blocked_space_indiv[furthest_agent]
            if self.unique_id==run.check_cyclist_id: print("Cat. 1 present, des_lat_pos={}".format(round(self.des_lat_pos,2)))


    ''' LEVEL 2: Moving angle and leader '''
    def findTraj(self):  
        if self.unique_id==run.check_cyclist_id: print("\n****** LEVEL 2: TRAJECTORY ************")
        # compute lateral movement distance to reach the desired position
        req_lat_move = self.des_lat_pos - self.getPos()[1]  # desired position minus actual position -> gives direction left or right directly
        obstr_cyclists = []  # cyclists potentially obstructing from reaching desired position
        if self.unique_id==run.check_cyclist_id: print("Reg_lat_move={}".format(round(req_lat_move,2)))
        # case with no slower cyclists
        if len(self.cat1_cyclists)==0: 
            # move the desired lateral speed to the position
            if abs(req_lat_move) < self.omega_des:  # handle case where desired lateral speed would overshoot the position within one time step
                self.v_lat = req_lat_move
            else:  # when the required lateral distance is not covered in one time step, handle whether tho move right or left at desired lateral speed
                if req_lat_move < 0:
                    self.v_lat = -self.omega_des
                else:
                    self.v_lat = self.omega_des
            if self.unique_id==run.check_cyclist_id: print("No cat. 1 leader, v_lat={}".format(round(self.v_lat,2)))
        else: # case with slower cyclists remaining
            ''' Obstructing cyclists might be +-x distance left or right from the 'corridor'.'''
            
            # remove cat. 1 cyclists that are not influencing the potential trajectory
            remove_indices = []
            for i in range(len(self.blocked_space_indiv)):
                obstr_cyclists.append(self.blocked_space_indiv[i][0])  # append cyclist object from the remaining cat. 1 cyclists
            
            # find non-obstructing cyclists depending on the direction of lateral movement
            for i in range(len(obstr_cyclists)):  
                if req_lat_move < 0:  # when moving to the right
                    if obstr_cyclists[i].getPos()[1] > self.getPos()[1]+1 or obstr_cyclists[i].getPos()[1] < self.des_lat_pos-1:
                        remove_indices.append(i)
                else:  # when moving to the left
                    if obstr_cyclists[i].getPos()[1] < self.getPos()[1]-1 or obstr_cyclists[i].getPos()[1] > self.des_lat_pos+1:
                        remove_indices.append(i)
                        ''' change +-1 to the width with safety region width '''
            # print('\n\nCat. 1 cyclists: ', self.cat1_cyclists)
            # print('\nSpace blockers before removal: ', obstr_cyclists)
            
            # remove cycists not influencing the trajectory
            for i in sorted(remove_indices, reverse=True):
                del obstr_cyclists[i]
            # print('\nSpace blockers after removal: ', obstr_cyclists)
            
            # if there is no obstructing cyclist, do the same as above and go towards the desired position at the end of the CR
            if len(obstr_cyclists)==0:
                if abs(req_lat_move) < self.omega_des:
                    self.v_lat = req_lat_move
                else:
                    if req_lat_move < 0:
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
                    
                    # calculate lateral passing point
                    lat_passing_point = 0
                    if req_lat_move < 0:
                        lat_passing_point = obstr_cyclists[i].getPos()[1]-1.2 # one meter to the right of the center
                    else:
                        lat_passing_point = obstr_cyclists[i].getPos()[1]+1.2 # one meter to the right of the center
                    ''' Need to use same metric here as for the determination of the lateral position '''
                    # calc moving angle (which angle is the absolute steepest)
                    angle_temp = math.atan2((lat_passing_point-self.getPos()[1]), dist_to_pass)
                    obstr_cyclists[i] = [obstr_cyclists[i], abs(angle_temp), dist_to_pass, lat_passing_point-self.getPos()[1]]
                
                # print('\n', obstr_cyclists)
                # go for the steepest angle (which angle is the absolute steepest)
                relevant_cyc_index = 0
                steepest_angle_temp = 0
                for i in range(len(obstr_cyclists)):
                    if obstr_cyclists[i][1] > steepest_angle_temp:
                        steepest_angle_temp = obstr_cyclists[i][1]
                        relevant_cyc_index = i

                # check if the angle is feasible with the maximum lateral speed
                max_angle_cur = abs(math.atan2(self.omega_max, self.getSpeed()))  # maximum angle at current speed would be
                # print(steepest_angle_temp)
                # print(max_angle_cur)
                
                # go the maximum lateral speed or the required speed 
                if steepest_angle_temp >= max_angle_cur:
                    if req_lat_move < 0:
                        self.v_lat = -self.omega_max
                    else:
                        self.v_lat = self.omega_max
                else:
                    if req_lat_move < 0:
                        self.v_lat = -(self.getSpeed()*math.tan(steepest_angle_temp))
                    else:
                        self.v_lat = (self.getSpeed()*math.tan(steepest_angle_temp))
                # print('V_lat: ', self.v_lat)
            if self.unique_id==run.check_cyclist_id: print("Cat. 1 present, v_lat={}".format(round(self.v_lat,2)))
            
        ''' Optional "look-back" module ''' 
        '''                       
        # look back and abort lateral movement if faster cyclists are present
        self.findCat3Behind()
        # restrict to v_lat distance and project forward
        proj_Cat3Behind = []  # projection of such cyclists
        if self.v_lat <= 0:
            proj_Cat3Behind = [l for l in self.cat3_behind if l.getPos()[1] <= self.pos[1] and l.getPos()[1] > (self.pos[1]-self.v_lat-1)]
        else:
            proj_Cat3Behind = [l for l in self.cat3_behind if l.getPos()[1] > self.pos[1] and l.getPos()[1] < (self.pos[1]+self.v_lat+1)]
        # sort for closest cyclist
        '''
        ''' needs more thoughts on how to work '''
        '''
        if len(proj_Cat3Behind) != 0:
            self.v_lat = 0
            self.cut_off_flag = True
        '''
        
        ''' Find the leader '''
        # find the leader
        self.findCat12() # get slower cyclists in front
        # print(self.cat12_cyclists)
        # potential_leaders = self.cat12_cyclists
        
        potential_leaders = []
        self.leader = 0
        if len(self.cat12_cyclists)==0: # if there is no leader
            self.leader = 0
        else: # if there are leader(s)
            # subtract obstructing cyclists from potential leaders
            # print('\nObstructing: ', obstr_cyclists)
            # print('Leading: ', self.cat12_cyclists)
            
            del_from_pot_lead = []
            for i in obstr_cyclists:
                del_from_pot_lead.append(i[0])
            # print('Obstructing: ', del_from_pot_lead)
            
            if self.cut_off_flag == False:
                potential_leaders = list(set(self.cat12_cyclists) - set(del_from_pot_lead))
            else:
                potential_leaders = self.cat12_cyclists
            # print('Potential leaders: ', potential_leaders)
            
            # potential_leaders = self.cat12_cyclists
            ''' cut off cyclists outside the region (left and right --> but SR width is still not implemented) '''
            if self.v_lat >= 0:
                potential_leaders = [i for i in potential_leaders if i.getPos()[1] >= self.getPos()[1]-1 and i.getPos()[1] <= self.des_lat_pos+1]
            if self.v_lat < 0:
                potential_leaders = [i for i in potential_leaders if i.getPos()[1] <= self.getPos()[1]+1 and i.getPos()[1] >= self.des_lat_pos-1]
            ''' cut off cyclists in front but behind the moving angle (latter needs to be implemented) '''
            
            if len(potential_leaders) != 0:
                # leaders = [l for l in neighbors if l.getPos()[0] > self.pos[0] and l.getPos()[0] < (self.pos[0]+self.cr_length)]
                # obtain closest of those inside the shape
                # print('\nPotential leaders: ', potential_leaders)
                closest_pos = self.getPos()[0]+self.cr_length # start finding closest leader in consideration range
                for i in potential_leaders: # find the closest potential leader
                    if i.getPos()[0] < closest_pos:
                        self.leader = i
                        closest_pos = i.getPos()[0]
                # print('Leader: ', self.leader.getPos()[0])
            else:
                self.leader = 0
        if self.unique_id==run.check_cyclist_id and self.leader!=0: print("Direct leader {}".format(self.leader.unique_id))
        if self.unique_id==run.check_cyclist_id and self.leader==0: print("No direct leader")
                
            
    ''' LEVEL 3: Acceleration according to NDM '''
    def findAcc(self):
        if self.unique_id==run.check_cyclist_id: print("\n****** LEVEL 3: ACCELERATION **********")
        # define the ndm parameters and functions
        headway_s = 0  # headway to leader (between centers of cyclists)
        delta_v = 0  # speed difference to leader
        safety_dist_d = self.sr_length  # longitudinal safety distance for NDM
        acc = 0  # realised acceleration
        dec1 = 0  # realised deceleration 1
        dec2 = 0  # realised deceleration 2
        
        # calculate potential (positive) acceleration
        if self.leader == 0: # if there is no leader
            # calculate potential (positive) acceleration
            acc = (self.v0-self.getSpeed())/self.a_des
            # acc = a_des
        
        elif self.leader != 0:  # if there is a leader
            if headway_s <= safety_dist_d:
                acc = 0
            else:
                acc = (self.v0-self.getSpeed())/self.a_des
                # acc = a_des
            
            # print('\nacc: ', acc)
            # calculate delta_v
            delta_v = self.getSpeed() - self.leader.getSpeed()
            # print('delta v: ', delta_v)
            
            # print(self.leader.getPos())
            # calculate long. headway and safety relevant safety distance
            headway_s = abs(self.getPos()[0]-self.leader.getPos()[0])
            # print('headway s: ', headway_s)
            # print('safety dist d: ', safety_dist_d)
            # calculate first deceleration part: matching the speed of the slower leader
            if delta_v > 0:
                dec1 = min((delta_v**2)/(2*(headway_s-self.length)), self.b_max) # necessary deceleration to match speed
                ''' maybe adapt this part here so that the aim is to match the safety distance length '''
            # print('dec1: ', dec1)
            # calculate second deceleration part: fall back to maintain the desired safety distance
            if delta_v <= 0.5 and headway_s <= safety_dist_d: 
                dec2 = self.b_max / ((self.length-safety_dist_d)**2) * ((headway_s-safety_dist_d)**2)
            # print('dec2: ', dec2)
            ''' Check this again later, maybe abs(delta_v) necessary '''
            if self.unique_id==run.check_cyclist_id: print("delta_v={}, hw_s={}, safety_d={}".format(round(delta_v,2),round(headway_s,2),round(safety_dist_d,2)))
        
        self.acceleration = acc - min(dec1+dec2, self.b_max) # limit total deceleration to b_max
        # print('Acceleration: ', self.acceleration)
        if self.unique_id==run.check_cyclist_id: print("acc={}, dec1={}, dec2={}".format(round(acc,2),round(dec1,2),round(dec2,2)))
        if self.unique_id==run.check_cyclist_id: print("Total acceleration={}".format(round(self.acceleration,2)))
    ''' 
    **********************************
    *** STEP AND ADVANCE FUNCTIONS ***
    **********************************
    '''
    
    # Read surroundings and determine next coordinates after they all take actions (Note that the agent hasn't really moved when this function is called)
    def step(self):
        ''' CALL LEVEL FUNCTIONS '''
        if self.unique_id==run.check_cyclist_id: print("active\n(v0={}, p={})".format(round(self.v0,2),round(self.p,2)))
        self.findLatPos() # level 1: lateral position
        self.findTraj() # level 2: moving angle and leader
        self.findAcc() # level 3: accelerations
        
        ''' CALL UPDATE FUNCTIONS '''
        self.calPos()
        self.calSpeed()
        # update regions:
        self.updateSize()
        self.updateCR()
        self.updateSR()
        if self.unique_id==run.check_cyclist_id: print("\n****** UPDATED VALUES *****************")
        if self.unique_id==run.check_cyclist_id: print("v={}, cr_length={}, sr_length={}".format(round(self.speed,2),round(self.cr_length,2),round(self.sr_length,2)))
        
    
    # Take (physical) actions, this function would be called automatically after the step() function
    def advance(self):
        self.model.space.move_agent(self,self.next_coords) # update on the canvas
        self.pos = (self.next_coords[0],self.next_coords[1]) # update self attributes
        # self.next_coords = tuple(self.next_coords) # make tuple from next coords
        self.speed = self.next_speed
        # print("Bicycle ",self.unique_id,"move to x = ",self.pos[0]," y = ",self.pos[1] - 0.5)
        # clear bicycles which finish the trip
        if self.pos[0] >= 300:
            self.model.to_be_removed.append(self)

#%% Model class

class BikeLane(Model):
    def __init__(self):
        super().__init__()
        self.schedule = SimultaneousActivation(self)
        
        # Create the canvas, which is a 300-m-long bike lane, 2 m wide with 0.5 m extra lateral spaces on both sides
        self.space = ContinuousSpace(300.1, run.path_width, torus=True) # Changed the torus=False here: otherwise, there will be an error because agents are 'out of bounds'
        
        # Initialize model variables
        self.time_step = 0
        self.inflow_count = 1 # The number of bicycle in the vertical queue that will enter
        self.n_agents = 0  # Current number of agents (bicycles) on the entire bike lane
        self.initial_coords = (0,1) # bicycles enter 0.5 m from the path edge
        self.to_be_removed = [] # A list storing bicycles which finish the trip at the time step and to be removed
        
        # Data collection functions, collect positions of every bicycle at every step, namely trajectories
        self.datacollector = DataCollector(agent_reporters={"Position": "pos", "Speed": "speed", "latSpeed": "v_lat", "ID": "unique_id", "desSpeed": "v0", "srLength": "sr_length", "srWidth": "sr_width", "crLength": "cr_length"})
    
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
        if self.inflow_count < len(Inflow_step):
            if self.time_step == Inflow_step[self.inflow_count]:
                b = Bicycle(self.inflow_count, self)
                self.schedule.add(b)
                self.space.place_agent(b, self.initial_coords)
                self.inflow_count += 1
                self.n_agents += 1
        # Update the time
        self.time_step += 1
        print("\n\n\nStep {}, cyclist {}".format(self.time_step,run.check_cyclist_id))  # review steps in console
        # Execute data collector
        self.datacollector.collect(self)

#%%