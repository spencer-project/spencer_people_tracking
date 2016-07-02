#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Fabian Girrbach, Social Robotics Lab, University of Freiburg
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Dependencies for ROS
import rospy
import rospkg
from std_msgs.msg import Float32, Bool

# Other dependencies
import yaml
from collections import namedtuple
from time import time, sleep
import subprocess
from signal import SIGKILL, SIGTERM
import os
from multiprocessing import Process, Pipe

# PySMAC
import pysmac 


# definition of parameters which should be optimized
parameters_to_optimize=dict(\
        markov_trans_prob_0_0=( 'real', [1e-5, 1], 0.8), # Markov transition probabilities                markov_trans_prob_0_0=( [0, 1], 0.2), # Markov transition probabilities
        markov_trans_prob_0_1=( 'real',[1e-5, 1], 0.1), # Markov transition probabilities
        markov_trans_prob_0_2=( 'real',[1e-5, 1], 0.1), # Markov transition probabilities
        markov_trans_prob_1_0=( 'real',[1e-5, 1], 0.3), # Markov transition probabilities
        markov_trans_prob_1_1=( 'real',[1e-5, 1], 0.6), # Markov transition probabilities
        markov_trans_prob_1_2=( 'real',[1e-5, 1], 0.1), # Markov transition probabilities
        markov_trans_prob_2_0=( 'real',[1e-5, 1], 0.4), # Markov transition probabilities
        markov_trans_prob_2_1=( 'real',[1e-5, 1], 0.4), # Markov transition probabilities
        markov_trans_prob_2_2=( 'real',[1e-5, 1], 0.2), # Markov transition probabilities
        IMM0_process_noise_level = ('real',[1e-5, 2.0], 0.01),
        IMM1_process_noise_level = ('real',[1e-5, 2.0], 0.1),
        IMM2_process_noise_level = ('real',[1e-5, 2.0], 0.1),
        ct_turn_rate_variance= ('real',[0, 5], 2.5),
        measurement_noise=("real", [0.01, 2], 0.1), # same as x1, but the initial value is -1
        process_noise_level=("real", [1e-5, 2.5], 0.1), # same as x1, but the initial value is -1
        logic_initiator_max_velocity=("real", [0.1, 2.5], 2.0), # same as x1, but the initial value is -1
        logic_initiator_min_velocity=("real", [0.1, 1], 0.5), # same as x1, but the initial value is -1
        logic_initiator_systematic_scan_error=("real", [0.001, 0.1], 0.07), # same as x1, but the initial value is -1
        logic_initiator_number_scans_before_acceptance=("integer", [4, 15], 6 ),
        logic_initiator_max_number_consecutive_missed_observations=("integer", [0, 7], 3),
        logic_initiator_use_incremental_checking=("integer", [0, 1], 1),
        track_is_mature_after_total_num_matches=("integer", [20, 300], 50), # same as x1, but the initial value is -1
        max_occlusions_before_deletion_of_mature_track=("integer", [10, 100], 50), # same as x1, but the initial value is -1
        max_occlusions_before_deletion=("integer", [1, 20], 20), # same as x1, but the initial value is -1
        )


# list of files where the parameters for the optimization should be found
parameter_files_to_search = ['/launch/params/thesis_imm_dynamic.yaml'
                            ,'/launch/params/thesis_ekf_logic_dynamic.yaml'
                            ,'/launch/params/thesis_logic_initiator_dynamic.yaml'
                            ,'/launch/params/thesis_occlusion_manager_basic_logic_dynamic.yaml' ]

# list of ros package names where parameters can be found
packages_for_paramteres = ['srl_nearest_neighbor_tracker']

#  roslaunch command which should be executed 
roslaunch_commands = [  ['roslaunch', 'srl_nearest_neighbor_tracker' , 'dynamic_tuning.launch']]

forbidden_clauses = ['{(max_occlusions_before_deletion_of_mature_track < max_occlusions_before_deletion)}',
                    '{(logic_initiator_max_velocity < logic_initiator_min_velocity)}',
                    '{(logic_initiator_number_scans_before_acceptance < logic_initiator_max_number_consecutive_missed_observations)}']


parameter_list = []
fail_result = -1.0
mota_result = -1.0

def find_parameters():
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    for package_name in packages_for_paramteres:
        package_path = rospack.get_path(package_name)
        for file in parameter_files_to_search:
            param_file = package_path + file
            try:
                params_in_file = yaml.load(open(param_file))
                for param_key in parameters_to_optimize:
                    if param_key in params_in_file:
                        print 'Found {} in {} paramfile {}'.format(param_key, param_file, parameters_to_optimize[param_key])
                        new_param = {'name':param_key, 'path':param_file, 'default':parameters_to_optimize[param_key][-1], 'current':50}
                        parameter_list.append(new_param)
            except:
                pass    
    
    
def write_parameters():
    print "inside write parameters"
    for param in parameter_list:
        print "current param {}".format(param)
        with open(param['path'], 'r') as param_file:
            params_in_file = yaml.load(param_file)
        if param['name'] == 'number_of_models':
            if param['current'] == 'TWO':
                params_in_file[param['name']] = 2
            elif param['current'] == 'THREE':
                params_in_file[param['name']] = 3
            elif param['current'] == 'FOUR':
                params_in_file[param['name']] = 4
        else:
            params_in_file[param['name']] = param['current']
        with open(param['path'], 'w') as param_file:
            param_file.write(yaml.dump(params_in_file, default_flow_style=True) )

def adaptMarkovProbabilities():
    m_00 = next(param for param in parameter_list if param['name']=='markov_trans_prob_0_0')
    m_01 = next(param for param in parameter_list if param['name']=='markov_trans_prob_0_1')
    m_02 = next(param for param in parameter_list if param['name']=='markov_trans_prob_0_2')
    m_10 = next(param for param in parameter_list if param['name']=='markov_trans_prob_1_0')
    m_11 = next(param for param in parameter_list if param['name']=='markov_trans_prob_1_1')
    m_12 = next(param for param in parameter_list if param['name']=='markov_trans_prob_1_2')
    m_20 = next(param for param in parameter_list if param['name']=='markov_trans_prob_2_0')
    m_21 = next(param for param in parameter_list if param['name']=='markov_trans_prob_2_1')
    m_22 = next(param for param in parameter_list if param['name']=='markov_trans_prob_2_2')

    sum0 = m_00['current'] + m_01['current'] + m_02['current']
    sum1 = m_10['current'] + m_11['current'] + m_12['current']
    sum2 = m_20['current'] + m_21['current'] + m_22['current']
    m_00['current'] /=  sum0
    m_01['current'] /=  sum0
    m_02['current'] /=  sum0
    m_10['current'] /=  sum1
    m_11['current'] /=  sum1
    m_12['current'] /=  sum1
    m_20['current'] /=  sum2
    m_21['current'] /=  sum2
    m_22['current'] /=  sum2
   

def resultCallback(result):
    print "PyMot results received {}".format(result)
    mota = result.data
    global mota_result     
    mota_result = mota
    global process_sim
    os.killpg(process_sim.pid, SIGTERM)
    sleep(3)
    rospy.signal_shutdown('Mota result received')
    
def clean_ros_logs():
    # After many iterations ros gathered a lot of log files therfore we clean them after each iteration
    subprocess.call(['rosclean','purge','-y'])
    
def start_node(child_conn, ros_command):
    clean_ros_logs()
    # launch tracker and everything which comes with it
    global process_sim, mota_result
    mota_result = fail_result
    process_sim = subprocess.Popen(ros_command, preexec_fn=os.setsid) 
    t = Process(target=ExceededTimeThread, args=(330,child_conn,))
    t.start()
    node = rospy.init_node('tuning_node', anonymous=True)
    while rospy.is_shutdown():
        print 'Waiting for ROS to start'
        sleep(1) 
    rospy.Subscriber("/pymot_result", Float32, resultCallback)
    rospy.spin()
    sleep(3)
    t.terminate()   
    tuning_object = dict()
    tuning_object['result'] = mota_result
    child_conn.send(tuning_object)

def ExceededTimeThread(seconds, child_conn):
    passed = 0
    while passed < seconds:
        sleep(10)
        passed += 10
        print ("Time passed {}".format(passed))
    global process_sim
    os.killpg(process_sim.pid, SIGTERM)
    sleep(3)
    subprocess.call(['rosnode','kill','--all'])

def optimize_parameters(**kwargs):
    print "Function was called with arguments: {}".format(kwargs)
    # Modify values in parameter list depending on passed values
    for arg in kwargs.keys():
        print "Current key: {}".format(arg)
        if arg == "instance":
            roslaunch_command = roslaunch_commands[kwargs[arg]]
            print "Current ROS Launch command is {}".format(roslaunch_command)
            continue
        try:
            current = next(param for param in parameter_list if param['name']==arg)
            current['current'] = kwargs[arg]
        except:
            pass
    adaptMarkovProbabilities()
    write_parameters()
    sleep(0.5)
    parent_conn, child_conn = Pipe()
    p = Process(target=start_node, args=(child_conn,roslaunch_command,))
    p.start()
    result = parent_conn.recv()
    print 'Received current result {}'.format(result['result'])
    p.join()
    p.terminate()   
    return -result['result']
        
def init_optimization():  
    opt = pysmac.SMAC_optimizer(working_directory= '/home/fabian/imm_dynamic_tmp',persistent_files=True, debug = False)
    parameter_definition= parameters_to_optimize
    print parameter_definition
    
    value, parameters = opt.minimize(optimize_parameters        # the function to be minimized
                    , 350                                # the maximum number of function evaluations
                    , parameter_definition 
                    , t_limit_function_s=360
                    , forbidden_clauses=forbidden_clauses
                    , num_runs=2
                    , deterministic=True
                    , num_train_instances=len(roslaunch_commands))                     # for the mainstation file one evaluation needs 6min    

    print('The minimum value %f was found for the configurations %s' % (value, parameters))


if __name__ == '__main__':
    try:
        find_parameters()
        init_optimization()
    except rospy.ROSInterruptException:
        pass