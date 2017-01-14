# -*- coding: utf-8 -*-
"""
Created on Sat Jan 14 16:04:40 2017

@author: leonidas
"""

import numpy
import matplotlib.pyplot as plt

def push(obstacle,v,scale=-0.2):
    force 	= [ .0, .0 ];
    length= len(obstacle);
    min_dist = 2.0
    for i in range(length):
        if (obstacle[i][0] - v[0] < min_dist) and (obstacle[i][1] - v[1] < min_dist):
            dist = numpy.sqrt( (obstacle[i][0] - v[0]) * (obstacle[i][0] - v[0]) + (obstacle[i][1] - v[1]) * (obstacle[i][1] - v[1]))
            force[0] = force[0] + (1/dist -1/min_dist) * scale * abs( obstacle[i][0] - v[0])/pow(dist,3);
            force[1] = force[1] + (1/dist -1/min_dist) * scale * abs( obstacle[i][1] - v[1])/pow(dist,3);
    return force;

def attract(t,v,scale=1):
    att_x= scale * (t[0]-v[0]);
    att_y= scale * (t[1]-v[1]);
    return att_x,att_y;

def APF_path():
    v_x	= 0.;
    v_y   = 0.;
    dt    = 0.1;
    c     = 0.;
    d     = 0.;
    scale_push  = -.4;
    scale_att   = 1.;
    while (abs(tango[1]-pose[1]) > tiny) or (abs(tango[0]-pose[0]) > tiny):
        att_x,att_y= attract(tango,pose,scale_att);
        push_f			= push(obs,pose,scale_push);
        v_x = dt * ( push_f[0] + att_x ) + v_x;
        v_y = dt * ( push_f[1] + att_y ) + v_y;
        v   = pow(v_x,2)+pow(v_x,2);
        if v>9:
            v_x = v_x*3/numpy.sqrt(v);
            v_y = v_y*3/numpy.sqrt(v);
        pose[0]		= pose[0] + dt * v_x;
        pose[1]		= pose[1] + dt * v_y;
        c = pose[0];
        d = pose[1];
        regist.append([c,d]);
        print v_x,v_y
        plt.scatter(c,d,color='m')

    temp = numpy.array(regist);
    obst = numpy.array(obs);
    b = obst.transpose();
    a = temp.transpose();
    plt.plot( a[0], a[1]);
    plt.plot(tango[0],tango[1],marker='*')
    plt.scatter(b[0],b[1],marker='o',color='r')
    plt.grid();

if __name__ == "__main__":
    obs		= [[1,2],[4,5],[5,1],[7,7],[6,7],[8,3],[8,9],[8,7],[9,9],[10,6]];
    # obstacles position
    tango	= [10,11]; #target position
    init	= [0,0];   #initial position
    tiny	= 0.1;     #max toleratable error 
    pose	= init;    #position in the loop  
    regist= [[.0,.0]];# generated path
    APF_path()