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

from pyfanova.fanova import Fanova
import numpy as np
import os, sys, getopt
from pyfanova.visualizer import Visualizer
from pyfanova.config_space import ConfigSpace
import matplotlib.pyplot
from subprocess import call

fanova = None
vis = None

def merge_runs(evaluation_folder):
    if not os.path.isdir(evaluation_folder +'/merged_state_runs'):
        subdirectories = [ os.path.join(evaluation_folder, name) for name in os.listdir(evaluation_folder) if os.path.isdir(os.path.join(evaluation_folder, name)) ]
        print subdirectories
        try:
    #         utils.state_merge(subdirectories, evaluation_folder+'/merged')
            call(["/home/fabian/tuning/pysmac/pysmac/smac/smac-v2.10.03-master-778/util/state-merge", "--directories", evaluation_folder, '--scenario-file', subdirectories[-1]+'/scenario.txt',  '--outdir' ,evaluation_folder +'/merged_state_runs/'])
        except:
            print "Error raised by state merge"
            pass
    prepare_fanova(evaluation_folder+'/merged_state_runs')
        
        
def prepare_fanova(merged_folder):
    global fanova, vis
    fanova = Fanova(merged_folder, improvement_over="QUANTILE", quantile_to_compare=0.8)
    vis = Visualizer(fanova)
    param_names = fanova.get_parameter_names()
    vis.create_all_plots(merged_folder)
    vis.create_most_important_pairwise_marginal_plots(merged_folder, len(param_names)**2)
    print 'Param names : {}'.format(param_names)
    print 'Integer Param names : {}'.format(fanova.get_config_space().get_integer_parameters())
    for param in param_names:
        marginal = fanova.get_marginal(param)
        print 'Param {} has marginal {}'.format(param, marginal)
    
def main(argv):
    scenario_folder = ''
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["help","ifile=","ofile="])
    except getopt.GetoptError:
        print 'eval_tuning.py -i <input_scenario_folder>'
        sys.exit(2)
    for opt, arg in opts:
        if opt in ('-h',"--help"):
            print 'eval_tuning.py -i <input_scenario_folder>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            scenario_folder = arg
    print 'Input file is "', scenario_folder
    if not(os.path.isdir(scenario_folder)):
        print "Folder {} does not exist".format(scenario_folder)
        sys.exit()
    merge_runs(scenario_folder)
   

if __name__ == "__main__":
   main(sys.argv[1:])
