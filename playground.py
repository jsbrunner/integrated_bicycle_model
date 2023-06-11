# -*- coding: utf-8 -*-
"""
Created on Fri Mar 10 17:16:44 2023

@author: jsb10
"""
'''
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import mesa
import math

x = 20
y = 1.5
y2 = -2

angle = math.atan2(y, x)

angle2 = math.atan2(y2, x)

print(angle, angle2)

y_calc = x * math.tan(angle2)

print(y_calc)
'''
'''
x=[1,2,3,4,5,6,7,8,9]
y=[2,3,6,4,9,6,7,8,1]

plt.plot(x,y, color='darksalmon', linewidth=3)
plt.show
plt.savefig('testfile.png', dpi=400)
'''

from datetime import datetime
current = datetime.now().strftime("%Y-%m-%d_%H%M")
print(current)


