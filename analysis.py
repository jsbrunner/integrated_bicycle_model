# -*- coding: utf-8 -*-
"""
Created on Sat Apr  8 12:15:25 2023

@author: jsb10
"""
import pandas as pd
import matplotlib.pyplot as plt
#%%

def plot_space_time(agent_pos, dt):
    pd.set_option('display.max_columns', None)
    unique_cyclists = agent_pos.AgentID.unique()
    print(unique_cyclists)
    
    agent_pos['Time'] = agent_pos['Step'] * dt
    print(agent_pos)
    fig, ax = plt.subplots(figsize=(10,6))
    for i in unique_cyclists:
        cyclist_traj = agent_pos[agent_pos.AgentID == i]
        ax.plot(cyclist_traj['Time'], cyclist_traj['Position_x'], color='black')
    ax.set_title('Space-Time Diagram')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    plt.show()  


def plot_fd(agent_pos, dt, duration, agg_time, agg_dist):
    agg_steps = int(agg_time/dt)
    agent_pos['Time'] = agent_pos['Step'] * dt
    print(agent_pos)
    agg_temp = 0
    agg_intervals = []
    while agg_temp <= (duration/dt):
        agg_intervals.append(agg_temp)
        agg_temp += agg_steps
        
    # for i in agg_intervals
    print(agg_intervals)
    q_k_v = pd.DataFrame(columns=['Flow', 'Density', 'Speed'])
    print(q_k_v)
    
    for i in range(1,len(agg_intervals)):
        # print('\n\n', i)
        agent_pos_temp = agent_pos[(agent_pos['Step'] <= agg_intervals[i]) & (agent_pos['Step'] > agg_intervals[i-1])]
        agent_pos_temp = agent_pos_temp[(agent_pos_temp['Position_x'] <= agg_dist[1]) & (agent_pos_temp['Position_x'] > agg_dist[0])]
        # print(agent_pos_temp)
        unique_cyclists_temp = agent_pos_temp['AgentID'].unique()
        # print(unique_cyclists_temp)
        vkt_sum = 0
        vht_sum = 0
        T = agg_time
        L = agg_dist[1]-agg_dist[0]  # length to derive the FD from
        for j in unique_cyclists_temp:
            agent = agent_pos_temp[agent_pos_temp['AgentID']==j]
            # print(agent)
            d_time = len(agent)*dt
            d_distance = agent['Position_x'].max() - agent['Position_x'].min()
            # print('dt: ', d_time, '; dx: ', d_distance)
            vkt_sum += d_distance
            vht_sum += d_time
        # print(vkt_sum, vht_sum)
        Q = vkt_sum / (T*L)
        K = vht_sum / (T*L)
        if vht_sum != 0:
            V = vkt_sum/vht_sum
        else: V = 0
        new_row = pd.DataFrame({'Flow': [Q], 'Density': [K], 'Speed': [V]})
        q_k_v = pd.concat([q_k_v, new_row], ignore_index=True)
    print(q_k_v)
    
    fig, ax = plt.subplots(figsize=(10,6))
    ax.scatter(q_k_v['Density'], q_k_v['Flow'], color='black')
    ax.set_title('Fundamental diagram for a 3-m-wide path')
    ax.set_xlabel('Bicycle density (bic/m)')
    ax.set_ylabel('Bicycle flow (bic/s)')
    fig.tight_layout()
    plt.show()
    
    
            
            
        
    
    
    

