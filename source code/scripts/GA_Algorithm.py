#!/usr/bin/env python

import numpy as np
import rospy,rospkg,roslaunch
import random,os,math,time
from scipy.stats import truncnorm
import pandas as pd
import scipy.optimize as opt
from gazebo_connection import GazeboConnection

class GA_algorithm():
    def __init__(self,num_parent,num_child,num_parameter,sigma,duration,direct,r_noa,max_sim_time):
        self.num_parent = num_parent
        self.num_child = num_child
        self.derivation = sigma.copy()
        self.num_parameter = num_parameter
        self.duration = int(duration)
        csv_read = pd.read_csv('/home/donghao/result/Hydro_ID/target/gait_AN'+str(r_noa)+'_high.csv',header=None)
#        csv_read = pd.read_csv('/home/donghao/result/Hydro_ID/target/gait_AN'+str(r_noa)+'_low.csv',header=None)
        temp = csv_read.to_numpy()
        self.marker = temp[:-1,:]
        rospy.logerr ("Version - V4")
        self.frequency = temp[-1,0]
        self.speed = temp[-1,1]
        self.r_marker = r_noa+3
        self.r_noa    = r_noa
        self.cro_ind = int(math.floor((r_noa*1.5+3)/2))
        weight = 1*np.ones((self.r_marker))
        weight[0] = 0.5
        weight[-1]= 5
        self.weight = weight/(np.sum(weight)*np.ones((self.r_marker)))
        if   r_noa == 2:
            self.a = np.array([-6, -1, -1,        -6, -1, -1,       -6, -1,                                         -0.5,-0.5,-0.5])
        elif r_noa == 4:
            self.a = np.array([-6, -1, -1,        -6, -1, -1,       -6, -1,       -0.5, -0.1,                       -0.5,-0.5,-0.5,-0.5,-0.5])
        elif r_noa == 6:
            self.a = np.array([-6, -1, -1,        -6, -1, -1,       -6, -1,       -0.5, -0.1,       -0.5, -0.1,     -0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5])
        self.b = abs(self.a)
        self.max_sim_time = max_sim_time
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/donghao/Hydro_ID/src/des_mubot/launch/AN"+str(self.r_noa)+"_time01.launch"])
        launch.start()
        time.sleep(1)
        self.gazebo = GazeboConnection()
        launch.shutdown()

    def first(self,ini_past):
        first_generation = np.zeros((self.num_child*self.num_parent,self.num_parameter))
        for j in range(self.num_parameter):
            a = (self.a[j]-ini_past[j])/self.derivation[j]
            b = (self.b[j]-ini_past[j])/self.derivation[j]
            for i in range(self.num_child*self.num_parent):
                first_generation[i,j] = truncnorm.rvs(a,b,loc=ini_past[j],scale=self.derivation[j])
        return first_generation

    def continue_learning(self):
        csv_read = pd.read_csv('/home/donghao/result/data/1/rollout_param.csv',header=None)
        temp = csv_read.to_numpy()
        new_generation = temp[- (self.num_child*self.num_parent):,:]
#        evo_finished = int ( temp.shape[0] ) // (self.num_child*self.num_parent))-1
        evo_finished = pd.read_csv('/home/donghao/result/data/1/fitness.csv',header=None).to_numpy().shape[0]
        return new_generation,evo_finished

    def regenerate(self, child, reward):
        sorted_index = reward.argsort()[::-1][:self.num_parent]
        parents = child[sorted_index]
        parents_reward = reward[sorted_index]
        cross = parents.copy()
        cross[0,::2] = parents[1,::2]
        cross[1,::2] = parents[2,::2]
        cross[2,::2] = parents[3,::2]
        cross[3,::2] = parents[0,::2]
        new_generation = np.zeros((self.num_child*self.num_parent,self.num_parameter))
        temp = np.zeros((self.num_child, self.num_parameter))
        for i in range(self.num_parent):
            for j in range(self.num_parameter):
                a = (self.a[j]-cross[i,j])/self.derivation[j]
                b = (self.b[j]-cross[i,j])/self.derivation[j]
                for k in range(self.num_child):
                    temp[k,j] = truncnorm.rvs(a,b,loc=cross[i,j],scale = self.derivation[j])
            new_generation[0+self.num_child*i:self.num_child*(i+1),:] = temp
        new_generation[:4,:] = parents[:4,:]
        return new_generation,parents,parents_reward

    def f_fit(self,x,ampli,theta,slope,inter):
        return ampli*np.sin( 2*np.pi*(self.frequency*x+theta) )+slope*x+inter

    def a_fit(self,cycle,alpha):
        return self.Sim_amp*np.sin(2*np.pi* (cycle+alpha) )

    def sta_check(self):
        indicator = 0
        fit_result,param_result,vel,colli_in = self.fitness_function(indicator)

        if colli_in:
            indicator = 1
            done = self.step_3()
            fit_result,param_result,vel,colli_in = self.fitness_function(indicator)

            if colli_in:
                indicator = 2
                done = self.step_10()
                fit_result,param_result,vel,colli_in = self.fitness_function(indicator)

                if colli_in:
                    indicator = 3
                    fit_result = -1
                    param_result = np.zeros((self.r_marker*2))
                    vel = -999999


        return fit_result,param_result,vel,indicator


    def fitness_function(self,indicator):
        csv_data  = pd.read_csv('/home/donghao/result/Hydro_ID/links_record.csv',header=None)

        if indicator ==0:
            time_s =  np.arange(0,self.duration,0.001)+0.001
            duration_s = self.duration*1000
            period_s = int(1000/self.frequency)
        elif indicator == 1:
            time_s =  np.arange(0,self.duration,0.0004)+0.0004
            duration_s = self.duration*2500
            period_s = int(2500/self.frequency)
        else:
            time_s =  np.arange(0,self.duration,0.0001)+0.0001
            duration_s = self.duration*10000
            period_s = int(10000/self.frequency)

        matrix    = csv_data.to_numpy()
        vel       = -np.mean(matrix[0:duration_s,-1])
        gait      = matrix[0:duration_s,1:-1]                    #

        param_rou = np.zeros((self.r_marker, 4))
        param_rou[:,0] = ( np.max (gait[ :period_s,:],axis=0) + np.max (gait[-period_s:,:],axis=0)   -   np.min (gait[:period_s,:],axis=0) - np.min(gait[-period_s:,:],axis=0) )/4
        param_rou[:,2] = ( np.mean(gait[-period_s:,:],axis=0) - np.mean(gait[ :period_s,:],axis=0) ) / ( np.mean(time_s[-period_s:]) - np.mean(time_s[:period_s]) )
        param_rou[:,3] =   np.mean(gait[ :period_s,:],axis=0)

        param_fit = np.zeros((self.r_marker,4))
        for ind in range(self.r_marker):
            param_fit[ind,:], _ = opt.curve_fit(self.f_fit,time_s, gait[:,ind],p0 = param_rou[ind,:])


        if ((np.max(param_fit[:,2]) - np.min(param_fit[:,2])) > 30) or (csv_data.isnull().values.any()) or (np.max(abs(param_fit[:,2]))>50):
            colli_in = True
            return 0,0,0,colli_in
        else:
            colli_in = False

        learn_gait = param_fit[:,:2].copy()
        cycle_step = np.arange(0,1,0.02)
        cycle = []
        Sim_amp=[]
        Exp_all=[]
        for i in range(self.r_marker):
            Exp_all = np.hstack( (Exp_all,(self.marker[i,0]*np.sin(2*np.pi*(cycle_step+self.marker[i,1])) * self.weight[i] )))
            cycle = np.hstack((cycle,cycle_step+learn_gait[i,1]))
            Sim_amp = np.hstack((Sim_amp,np.ones(cycle_step.size)*learn_gait[i,0]* self.weight[i]))
        self.Sim_amp = Sim_amp.copy()
        if learn_gait[-1,0]<0:
            alp_guess = (self.marker[-1,1]-learn_gait[-1,1]-0.5)%1
        else:
            alp_guess = (self.marker[-1,1]-learn_gait[-1,1])%1
        alp,_ = opt.curve_fit(self.a_fit,cycle,Exp_all,p0=alp_guess,bounds=(0,1),maxfev=20000)


        final_gait = learn_gait.copy()
        final_gait[:,1] = final_gait[:,1]+alp
        sin_error = np.zeros((self.r_marker))
        Exp_sin = np.zeros((cycle_step.size))
        Sim_sin = np.zeros((cycle_step.size))
        for i in range(self.r_marker):
            Exp_sin = self.marker[i,0]  * np.sin( 2*np.pi*(cycle_step+self.marker[i,1]) )
            Sim_sin = final_gait[i,0]   * np.sin( 2*np.pi*(cycle_step+final_gait[i,1]) )
            sin_error[i] = abs(  np.sqrt( np.mean((Exp_sin-Sim_sin)**2) )  / np.sqrt( np.mean( (Exp_sin)**2)))* self.weight[i]

        fit_sin = np.sum(sin_error)
        fit_vel = abs(vel/self.speed - 1)
        fit_result = 1/(fit_sin+fit_vel*0.2)
        re_gait = np.reshape(final_gait,(1,-1))
        self.Sim_amp = []
        return fit_result,re_gait,vel,colli_in


    def step(self):
        done = False
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/donghao/Hydro_ID/src/des_mubot/launch/AN"+str(self.r_noa)+"_time01.launch"])
        launch.start()
        time.sleep(2)
        self.gazebo.unpause()
        time.sleep(self.max_sim_time)
        while not done:
            now = rospy.get_time()
            if now >= self.max_sim_time:
                done = True
            else:
                done = False
        launch.shutdown()
        return done

    def step_3(self):
        rospy.logerr("3 Times slower!")
        done = False
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/donghao/Hydro_ID/src/des_mubot/launch/AN"+str(self.r_noa)+"_time03.launch"])
        launch.start()
        time.sleep(2)
        self.gazebo.unpause()
        time.sleep(self.max_sim_time)
        while not done:
            now = rospy.get_time()
            if now >= self.max_sim_time:
                done = True
            else:
                done = False
        launch.shutdown()
        return done

    def step_10(self):
        rospy.logerr("10 Times slower!")
        done = False
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/donghao/Hydro_ID/src/des_mubot/launch/AN"+str(self.r_noa)+"_time10.launch"])
        launch.start()
        time.sleep(2)
        self.gazebo.unpause()
        time.sleep(self.max_sim_time)
        while not done:
            now = rospy.get_time()
            if now >= self.max_sim_time:
                done = True
            else:
                done = False
        launch.shutdown()
        return done
