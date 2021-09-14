#!/usr/bin/env python

import numpy as np
import move_robot_
import matplotlib.pyplot as plt 
from casadi import *
import math
import csv
from numpy import *
#import matplotlib
#matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
class Iteration_Callback(Callback):
    def __init__(self, name, nx, ng, opts={}):
        Callback.__init__(self)

        self.nx = nx
        self.ng = ng

        self.x_sols = []
        self.f_sols = []
        self.g_sols = []
        self.lam_x_sols = []
        self.lam_g_sols = []
        self.lam_p_sols = []
        '''
        #for plotting the graph in realtime
        self.fig= plt.figure()
        self.ax1= self.fig.add_subplot(1,2,1)
        self.ax2= self.fig.add_subplot(2,2,1)
        self.fig.show()
        '''
        # Initialize internal objects
        self.construct(name, opts)

    def get_n_in(self): return nlpsol_n_out()
    def get_n_out(self): return 1
    def get_name_in(self, i): return nlpsol_out(i)
    def get_name_out(self, i): return "ret"

    def get_sparsity_in(self, i):
        n = nlpsol_out(i)
        if n == 'f':
            return Sparsity.scalar()
        elif n in ('x', 'lam_x'):
            return Sparsity.dense(self.nx)
        elif n in ('g', 'lam_g'):
            return Sparsity.dense(self.ng)
        else:
            return Sparsity(0, 0)

    def eval(self, arg):
        
        darg = {}
        for (i, s) in enumerate(nlpsol_out()): darg[s] = arg[i]
        x_sol = [float(elem) for elem in darg['x'].full()]
        
        self.x_sols.append(x_sol)
        self.f_sols.append(darg['f'].full()[0][0])
        self.g_sols.append(darg['g'].full()[0][0])
        self.lam_x_sols.append(darg['lam_x'].full())
        self.lam_g_sols.append(darg['lam_g'].full())
        self.lam_p_sols.append(darg['lam_p'].full())
        #print("solution",self.f_sols)
        '''
        #for plotting the graph in realtime
        self.ax1.plot(self.x_sols)
        self.ax2.plot(self.f_sols)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.show()
        '''
	plt.plot(self.f_sols)
        #plt.plot(self.f_sols,color='C0')
        #plt.plot(self.g_sols,color='C0')
        #print(self.g_sols)
        return[0]

dest=[4,2,0]
#way=[3,1]  
angle_rad=dest[2]*(np.pi/180)
t=SX.sym('t')
v=SX.sym('v')
dt= 20
rob_rad=0.1
obs_rad=0.1
ang=SX.sym("ang",dt)
#v=SX.sym("v")
#t=SX.sym("t")
print(ang)
func,func1,func2=0,0,0
g=0
theta=0
fuel_cost=0
angle=0
coff=0.0045
ang_coeff=1
for i in range(0,dt):
        func1=func1+v*cos(theta)*t
        func2=func2+v*sin(theta)*t
        theta=theta + (v / 1.2)*tan(ang[i])*t
        angle= sqrt((theta-angle_rad)**2)*ang_coeff
        #if i>=dt/2:
        fuel_cost=fuel_cost+(-0.0177*v**2+1.48*v+3.39)
        #if i==dt/2:
            #func_con=(sqrt(coff+(func1-way[0])**2+(func2-way[1])**2))
        if i==dt-1:
            func=(sqrt(coff+(func1-dest[0])**2+(func2-dest[1])**2))+fuel_cost+angle
            #g=sqrt((func1-way[0])**2+(func2-way[1])**2)-(rob_rad+obs_rad)
            #g=func_con
            print(func)     
            print(g)
nlp= {}
nlp['x']=vertcat(ang,v,t)
nlp['f']=func
nlp['g']=0
mycallback = Iteration_Callback('mycallback',(dt+2), 1)
opts = {}
opts['iteration_callback'] = mycallback
opts['ipopt.tol'] = 1e-8
opts["ipopt.max_iter"]=3000
#create solver instance
F = nlpsol('F','ipopt',nlp,opts);
print(F)

#solver=F(x0=[0.1,2],lbx=lbx,ubx=ubx)
x0=[]
lbx=[]
ubx=[]
# the loop for setting angle
for i in range(0,dt):
    x0.append(0.02)
    lbx.append(-0.52359)
    ubx.append(0.52359)
# the loop for setting velocity  
for i in range(0,1):
    x0.append(0.2)
    lbx.append(0.02)
    ubx.append(3)
# the loop for setting sample time 
for i in range(0,1):
    x0.append(0.2)
    lbx.append(0.02)
    #ubx.append(2) #previous value
    ubx.append(100)
print(x0)
solver=F(x0=x0,lbx=lbx,ubx=ubx,lbg=0,ubg=0)
out=[]
out.append(solver["x"])
print("solved output",out)
print("solved function output",solver["f"])
plot1=plt.figure(1)
plt.legend(["Cost"])
plt.xlabel("Iterations")
plt.ylabel("Cost Value")
plt.savefig("case_2_cost")


import matplotlib.pyplot as plt
import math
ang=[]
v=[]
t=[]
th=[]
for i in range(0,dt):
    ang.append(out[0][i])
print("angle",ang)
v.append(out[0][dt])
t.append(out[0][dt+1])
print("v",v)
print("T",t)
xc,yc=[],[]
theta=0.0
x1,x2=0,0
for i in range(0,dt):
    x1=x1+v[0]*math.cos(theta)*t[0]
    x2=x2+v[0]*math.sin(theta)*t[0]
    theta = theta + (v[0] / 1.2) * math.tan(ang[i])*t[0]
    xc.append(x1)
    yc.append(x2)
    th.append(theta)
print(x1,x2)
#plt.figure(figsize=(6, 4))
plot2=plt.figure(2)
plt.plot(xc,yc,dest[0],dest[1],"o")
plt.legend(["Trajectory","Destination"])
plt.xlabel("X Coordinates")
plt.ylabel("Y Coordinates")
plt.grid(True)
plt.savefig("case_2_ang_traj")
details=["x_coord","y_coord"]
way=[]
for i in range(0,len(xc)):
	way.append([xc[i],yc[i]])
print(way)
with open("case2_1_angle.csv",'w') as f:
	write=csv.writer(f)
	write.writerow(details)
	write.writerows(way)
move_robot_.move_function(way)
print("all_done")



