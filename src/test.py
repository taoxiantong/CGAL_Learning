#!/usr/bin/python
# -*- coding: UTF-8 -*-
"""
python解方程
"""
 
from scipy.optimize import fsolve
 
def solve_function(unsolved_value):
    x,y,z=unsolved_value[0],unsolved_value[1],unsolved_value[2]
    return [
        x**2+y**2+z**2-1,
        0.5*x+0.5*y+0.5**0.5*z,
        -0.5**0.5*x+0.5**0.5*y-0.5,
    ]
 
solved=fsolve(solve_function,[0, 0, 0])
print(solved)
 
 
print("Program done!")
 
"""
运行结果：
[-1.  3.  5.]
Program done!
"""