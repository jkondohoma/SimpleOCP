#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 20 17:54:32 2023

@author: jaellekondohoma
"""

from pyomo.environ import *
from pyomo.dae import *
import matplotlib.pyplot as plt

model = m = ConcreteModel()

T = 50                  # Final time                                               [s]
exhaust_velocity = 2500 # the average velocity at which exhaust gases are expelled [m/s]
g = 9.81                # Acceleration due to gravity                              [m/s]
initial_altitude = 1000 #                                                          [m]
final_altitude = 2000 #                                                            [m]
initial_velocity = 0 #                                                             [m/s]
final_velocity = 0 #                                                               [m/s]
initial_mass =     1000 #                                                          [kg]
mass_loss_rate = 3 #                                                               [kg]

# Time points and step size
m.tf = Param(initialize=T)
m.t = ContinuousSet(bounds=(0,m.tf))

m.u = Var(m.t, initialize=0)
m.h = Var(m.t) #altitude
m.v = Var(m.t) #velocity
m.m = Var(m.t) #mass

m.dhdt = DerivativeVar(m.h, wrt=m.t)
m.dvdt = DerivativeVar(m.v, wrt=m.t)
m.dmdt = DerivativeVar(m.m, wrt=m.t)

m.obj = Objective(expr=m.u[m.tf])

# diffrential equations (dynamics constraints)
def _hdot(m, t):
    return m.dhdt[t] == m.v[t];
m.hdot = Constraint(m.t, rule=_hdot)

def _vdot(m, t):
    return m.dvdt[t] == (m.u[t] * exhaust_velocity * m.m[t]) - g;
m.vdot = Constraint(m.t, rule=_vdot)

def _mdot(m, t):
    return m.dmdt[t] == -m.u[t] * mass_loss_rate
m.x3dot = Constraint(m.t, rule=_mdot)

#thrust constraint
def _con_u_max(m, t):
    return m.u[t] <= 1
m.con_max = Constraint(m.t, rule=_con_u_max)

def _con_u_min(m, t):
    return 0 <= m.u[t]
m.con_min = Constraint(m.t, rule=_con_u_min)

def _con(m, t):
    return m.v[t] - 8*(t-0.5)**2 + 0.5 <= 0
m.con = Constraint(m.t, rule=_con)


def _init(m):
    #initial values
    yield m.h[0] == initial_altitude
    yield m.v[m.t.first()] == initial_velocity
    yield m.m[m.t.first()] == initial_mass
    
    #terminal values
    yield m.h[m.t.last()] == final_altitude
    yield m.v[m.t.last()] == final_velocity
    

m.init_conditions = ConstraintList(rule=_init)

# discretizer = TransformationFactory("dae.collocation").apply_to(m,nfe=7,ncp=6,scheme="LAGRANGE-RADAU")

# Solve the model
results = SolverFactory('ipopt').solve(m)

def plotter(subplot, x, *y, **kwds):
    plt.subplot(subplot)
    for i,_y in enumerate(y):
        plt.plot(list(x), [value(_y[t]) for t in x],"brgcmk"[i%6])
        if kwds.get("points", False):
            plt.plot(list(x), [value(_y[t]) for t in x], "o")
        plt.title(kwds.get("title",""))
        plt.legend(tuple(_y.name for _y in y))
        plt.xlabel(x.name)
            
            
plotter(121, m.t, m.h, m.v, title="Differential Variables")
plotter(122, m.t, m.u, title="Control Variable", points=True)
plt.show()