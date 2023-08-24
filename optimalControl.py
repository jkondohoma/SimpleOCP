#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 20 12:16:52 2023
based on "small optimal control problem" from "Pyomo — Optimization Modeling in Python" 
chapter 11: Differential Algebraic Equations

"""

from pyomo.environ import *
from pyomo.dae import *
import matplotlib.pyplot as plt
import time

model = m = ConcreteModel()

#Time Horizon
m.tf = Param(initialize = 1)
m.t = ContinuousSet(bounds=(0, m.tf))

#State Variables
m.u = Var(m.t, initialize=0) #control variable
m.x1 = Var(m.t)
m.x2 = Var(m.t)
m.x3 = Var(m.t)

#Differentiall equations
m.dx1dt = DerivativeVar(m.x1, wrt=m.t)
m.dx2dt = DerivativeVar(m.x2, wrt=m.t)
m.dx3dt= DerivativeVar(m.x3, wrt=m.t)

#Objective function
m.obj = Objective(expr=m.x3[m.tf])


def _x1dot(m, t):
    return m.dx1dt[t] == m.x2[t]
m.x1dot = Constraint(m.t, rule=_x1dot)

def _x2dot(m, t):
    return m.dx2dt[t] == -m.x2[t] + m.u[t]
m.x2dot = Constraint(m.t, rule=_x2dot)

def _x3dot(m, t):
    return m.dx3dt[t] == m.x1[t]**2 + m.x2[t]**2 + 0.005*m.u[t]**2 
m.x3dot = Constraint(m.t, rule=_x3dot)

def _con(m, t):
    return m.x2[t] - 8*(t-0.5)**2 + 0.5 <= 0
m.con = Constraint(m.t, rule=_con)

def _init(m):
    yield m.x1[0] == 0
    yield m.x2[0] == -1
    yield m.x3[0] == 0
m.init_conditions = ConstraintList(rule=_init)

# Discretize model using radau collocation
TransformationFactory('dae.collocation').apply_to(m, nfe=7, ncp=6,
                                                  scheme='LAGRANGE-RADAU' )
# Solve algebraic model
start = time.time()
results = SolverFactory('glpk').solve(m)
end = time.time()
print(" Execution time: ")
print(end - start)


def plotter(subplot, x, *series, **kwds):
    plt.subplot(subplot)
    for i,y in enumerate(series):
        plt.plot(x, [value(y[t]) for t in x], 'brgcmk'[i%6]+kwds.get('points',''))
    plt.title(kwds.get('title',''))
    plt.legend(tuple(y.cname() for y in series))
    plt.xlabel(x.cname())
import matplotlib.pyplot as plt
plotter(121, m.t, m.x1, m.x2, m.x3,  title='Differential Variables')
plotter(122, m.t, m.u, title='Control Variable', points='o')
plt.show()
print("Print values for variables")
for v in model.component_data_objects(Var):
  print (str(v), v.value)
print("\n")
print("-----------------------------------\n") 
print(results)
# print("\n")
# for i in model.x3:
#   print(str(m.x3[i]), m.x3[i].value)



