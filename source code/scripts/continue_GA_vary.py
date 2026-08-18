#!/usr/bin/env python

import numpy as np
import rospy,rospkg,roslaunch
import random,os,math,time,shutil
from scipy.stats import truncnorm
import pandas as pd
import scipy.optimize as opt
from gazebo_connection import GazeboConnection
from GA_Algorithm import GA_algorithm

if __name__ == '__main__':

    rospy.init_node('mubot_identification', anonymous=True, log_level=rospy.WARN)
    # Set the logging system
    outdir     = '/home/donghao/result/data/'                                            # Data storage location
    hyd_file   = '/home/donghao/result/Hydro_ID/hyd_param.csv'                          # Data Exchangeing location
    spr_file   = '/home/donghao/result/Hydro_ID/spr_param.csv'                          # Data Exchangeing location

    # Loads parameters from the ROS param server
    n_trial         = rospy.get_param("/mubot/n_trial")                                 #
    n_evaluation    = rospy.get_param("/mubot/n_evaluation")                            #
    n_parent        = rospy.get_param("/mubot/n_parent")                                #
    n_child         = rospy.get_param("/mubot/n_child")                                 #
    n_population    = n_child*n_parent

    r_max_sim_time  = rospy.get_param("/mubot/max_sim_time")                            #
    r_duration      = rospy.get_param("/mubot/duration")                                #
    r_noa           = rospy.get_param("/mubot/actuator_number")
    spr_param       = int(r_noa+1)
    hyd_param       = int(r_noa+6)
    n_params        = spr_param+hyd_param
    # Initialize parameters
                                         #record total mu
    if   r_noa ==2:                                                                       #record total sigma
        record_mu    = np.zeros((n_evaluation+1,n_params))
        record_sigma = np.array([0.5, 0.2, 0.2,     0.5, 0.2, 0.2,       0.5, 0.2,                                                  0.05,0.05,0.05])
    elif r_noa ==4:
        record_mu    = np.zeros((n_evaluation+1,n_params))
        record_sigma = np.array([0.5, 0.2, 0.2,     0.5, 0.2, 0.2,       0.5, 0.2,     0.2, 0.05,                                   0.05,0.05,0.05,0.05,0.05])
    elif r_noa ==6:
        record_mu    = np.zeros((n_evaluation+1,n_params))
        record_sigma = np.array([0.5, 0.2, 0.2,     0.5, 0.2, 0.2,       0.5, 0.2,     0.2, 0.05, 0.2, 0.05,                        0.05,0.05,0.05,0.05,0.05,0.05,0.05])
    record_fitness = np.zeros((n_population))                                    #record total fitness
    record_indicator = np.zeros((n_population))
    pop_mu = np.zeros(n_params)                                                  #mu for current population
    pop_params  = np.zeros((n_population,n_params))
    record_gait = np.zeros((n_population,2*(r_noa+3)+1))

    #Initialize learning algorithm
    GA = GA_algorithm(n_parent,n_child,n_params,record_sigma,r_duration,outdir,r_noa,r_max_sim_time)

    z = 0
    if not os.path.exists(outdir+str(z+1)):
        os.makedirs(outdir+str(z+1))

    pop_params,evo_finished = GA.continue_learning()
    x = np.copy(evo_finished)

    while x < n_evaluation:
        y = 0
        record_fitness.fill(0)
        record_indicator.fill(0)
        record_gait.fill(0)

        if x==evo_finished:
#            csv_read_fit = pd.read_csv(outdir+'/'+str(z+1)+'/temp/temp_fitness.csv',header=None)
#            temp_fit = csv_read_fit.to_numpy()

            temp_gait = pd.read_csv(outdir+'/'+str(z+1)+'/temp/temp_rollout_gait.csv',header=None).to_numpy()

#            csv_read_sta = pd.read_csv(outdir+'/'+str(z+1)+'/temp/temp_sta_ind.csv',header=None)
#            temp_sta = csv_read_sta.to_numpy()

            record_fitness                     = pd.read_csv(outdir+'/'+str(z+1)+'/temp/temp_fitness.csv',      header=None).to_numpy().flatten()
            record_gait[:temp_gait.shape[0],:] = temp_gait
            record_indicator                   = pd.read_csv(outdir+'/'+str(z+1)+'/temp/temp_sta_ind.csv',      header=None).to_numpy().flatten()
            y = temp_gait.shape[0]
            rospy.logerr("Read Gait: "+str(y))
        else:
            pd.DataFrame(pop_params).to_csv(outdir+'/'+str(z+1)+'/rollout_param.csv',index=False,header=False,float_format='%.6e',mode='a')

        while y < n_population:
            hyd_write = np.zeros((1,hyd_param))
            spr_write = np.zeros((1,spr_param))
            hyd_write[0,:8] = pop_params[y,:8]
            spr_write[0,:]   = pop_params[y,hyd_param:]
            if r_noa >2 :
                hyd_write[0,8:10] = pop_params[y,6:8]+pop_params[y,8:10]
                if r_noa == 6 :
                    hyd_write[0,10:12] = pop_params[y,6:8]+pop_params[y,10:12]
            pd.DataFrame(hyd_write.reshape(1,-1)).to_csv(hyd_file,index=False,header=False,float_format='%.6e',mode='w')
            pd.DataFrame(spr_write.reshape(1,-1)).to_csv(spr_file,index=False,header=False,float_format='%.6e',mode='w')

            # Run simulation
            rospy.logerr(" E: "+str(x+1)+" P: " + str(y+1)+"   Hydro_ID:"+str(pop_params[y,:]))
            done = False
            done = GA.step()
            record_fitness[y], record_gait[y,:-1], record_gait[y,-1], record_indicator[y]= GA.sta_check()

            pd.DataFrame(record_fitness).  to_csv(outdir+'/'+str(z+1)+'/temp/temp_fitness.csv',     index=False,header=False,float_format='%.6e',mode='w')
            pd.DataFrame(record_indicator).to_csv(outdir+'/'+str(z+1)+'/temp/temp_sta_ind.csv',     index=False,header=False,float_format='%d',  mode='w')
            pd.DataFrame(record_gait[:y+1,:]).   to_csv(outdir+'/'+str(z+1)+'/temp/temp_rollout_gait.csv',index=False,header=False,float_format='%.6e',mode='w')

            y += 1

        # Regenerate
        ep_params,parents,parents_fitness = GA.regenerate(pop_params,record_fitness)

        if x==0:
            pd.DataFrame(parents.reshape(1,-1))         .to_csv(outdir+'/'+str(z+1)+'/param.csv',        index=False,header=False,float_format='%.6e',mode='w')
            pd.DataFrame(record_fitness.reshape(1,-1))  .to_csv(outdir+'/'+str(z+1)+'/fitness.csv',      index=False,header=False,float_format='%.6e',mode='w')
            pd.DataFrame(record_gait.reshape(1,-1))     .to_csv(outdir+'/'+str(z+1)+'/rollout_gait.csv', index=False,header=False,float_format='%.6e',mode='w')
            pd.DataFrame(record_indicator.reshape(1,-1)).to_csv(outdir+'/'+str(z+1)+'/sta_ind.csv',      index=False,header=False,float_format='%d',  mode='w')
        else:
            pd.DataFrame(parents.reshape(1,-1))         .to_csv(outdir+'/'+str(z+1)+'/param.csv',        index=False,header=False,float_format='%.6e',mode='a')
            pd.DataFrame(record_fitness.reshape(1,-1))  .to_csv(outdir+'/'+str(z+1)+'/fitness.csv',      index=False,header=False,float_format='%.6e',mode='a')
            pd.DataFrame(record_gait.reshape(1,-1))     .to_csv(outdir+'/'+str(z+1)+'/rollout_gait.csv', index=False,header=False,float_format='%.6e',mode='a')
            pd.DataFrame(record_indicator.reshape(1,-1)).to_csv(outdir+'/'+str(z+1)+'/sta_ind.csv',      index=False,header=False,float_format='%d',  mode='a')
        x += 1
        pop_params = ep_params

    rospy.logwarn("Learning is over. Yay!")

    shutil.rmtree(outdir+'/'+str(z+1)+'/temp')


