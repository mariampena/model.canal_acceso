# -*- coding: utf-8 -*-
"""
Created on Fri May 20 16:44:03 2022

@author: pmayaduque
"""
import random 
arcos = [(1,2), (2,3), (3,4), (4,5), (3,6), (6,7), (7,6), (6,3), (5,4), (4,3), (3,2), (2,1)]
for i in range(7):
    rnd = random.random()
    for arc in arcos:
        if rnd < 0.3:
            ini = 6
        else:
            ini = 0
        print('"({}, {})" : {},'.format(i+1, arc, ini))