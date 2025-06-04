import control
import sys
import controlDesign447 as cd447

# eventually delete
import matplotlib.pyplot as plt
import numpy as np
import time

#
# Control Design problem definition file template
#
#    Modify this file below to set up your control design problem
#


def cc(x):  # keeps code easier to read
    return np.conj(x)  # get the complex conjugate
#

#  Process this command line
s = control.TransferFunction.s
allowedTasks = ['Rlocus', 'Optimize', 'StepResponse']
task = cd447.process_cmd_line(sys.argv,allowedTasks)



###############################################
# #
#    Plant Definition Here
#

Plant = 2.4/((s+2)*(s+4))   # just an example system


############################################### controller setup section
#
#    Controller Name
#
#        Give your controller a descriptive name (replace string below)
#
controllerD = {}   # a dictionary to hold all control design parameters
controllerD['Name'] = 'Template File for Control Design Setup'


############################################
#
#    Start here for controller configuration
#     (only uncomment ONE controller type)

# two simple controller options:

###################################
# 1) Constant single gain controller

#          The simplest possible controller but limited power
# controllerD['Ctype']  = 'Kct'  # a single gain controller
# controllerD['Params'] = [ 1000 ]  # example K=4

###################################
# 2) Lead-Lag Compensator
#          A widely used basic controller
# controllerD['Ctype'] = 'LLC'  # lead/lag compensator controller
# pole = -2
# zero = -3
# controllerD['Params'] = [60,   pole, zero]  # example [ K, pole, zero]


#############################
# 3) PID Controller
#       () As used by 99% of industrial controllers )
controllerD['Ctype'] = 'PID'
# flags (do not change these here)
PIDgains = False  # do not change
PIDZeros = False  # do not change



##############################
#
#    Two ways to initialize the PID controller
#

#
#   PID Controller: Set up with gain vector
#
# PIDgains = True   # this tells system we start with gains Kp,Ki,Kd
# Kp = 40
# Ki = 2
# Kd = 0.8

#
#   PID Controller: Set up with Kd and two zeros
#
# PIDZeros = True   # this tells system we start with gains Kd, z1, z2
# Kd = 1.0
# z1 = -5
# z2 = -12
#############################

#########################  End of controller setup section  ##################





##############################################################################
##############################################################################
#   Workspace logs here (as comments)
#      This is a hand place to document your results
#

# 1)  Explain how you start off your design.  For example:
# PIDZeros = True   # this tells system we start with gains Kd, z1, z2
# Kd = 1.0
# z1 = -6 + 6j
# z2 = cc(z1)

# 2)  Summarize key results of first step

# 3) further iterations


####################### End of Workspace Log Section ###########################
################################################################################


#
# set up regularization pole (make C(s) proper)
#
#  (25 is a default value that works for most systems)
controllerD['RegSep'] = 25  # how far to separate regularization pole from most negative real part


############### PID Controller automated sanity checks (do not modify)  ################
if controllerD['Ctype'] =='PID':
    # flag chooses between how we initialize PID controller:
    if PIDgains and PIDZeros:
        cd447.error('Please set only one of [PIDgains, PIDzeros]')
    if not(PIDgains or PIDZeros):
        cd447.error('You must set one fo the flags:  [PIDgains, PIDzeros] ')

    if PIDgains:
        #  Set controller based on Kp,Ki,Kd
        controllerD['Zeros'] = cd447.PIDZerosFromKs(Kp,Ki,Kd)
        controllerD['Params']= [Kp,Ki,Kd]
        #
    elif PIDZeros:
        #  Set controller based on Kd, z1, z2
        controllerD['Zeros'] = [z1,z2]
        controllerD['Params']= cd447.PIDKsFromZeros(Kd,z1,z2)

    print('Initializing PID with: ', controllerD['Params'])

# instantiate the controller based on parameter dictionary
contObj = cd447.controller(controllerD) # instantiate the controller as above

##########################   end of PID sanity Checks  #################################


###############################################
#
#    Search Setup
#
# Search Parameter Dictionary
SPd = {}
SPd['Name'] = 'SearchSetup: ' + controllerD['Name']

#
#    Your performance specs go here
#
# Desired Performance target
SPd['tsd']         =   0.6  # Desired settling time (sec)
SPd['pod']         =  0.20  # # Desired overshoot ratio (e.g. 10%=0.10)
SPd['ssed']        =  0.01  # Desired steady state error
#  (below are default values if you do not have Control Effort or Gain Margin Specs.)
SPd['cu_max']      =   200  # Desired Maximum control effort (arbitrary units)
SPd['gm_db']       =    20  # Desired gain margin in dB (positive = stable)

#
#   Set up your search below
#
# Search Parameters
SPd['scale_range'] =  2   # Search range multiplier ('width' of search)
SPd['nvals']       =  15   # Number of points per parameter (# grid points)
SPd['reportScheme']=  'WSO'  # which weights to print the limit-report on ('WSO' = TS + %OS)

#  Simulation parameters (probably don't need to change)
SPd['tmax']        =  4*SPd['tsd']    #maximum simulation time
SPd['dt']          =  1/500          # Time step ( heuristic)

###################################################################################
##############     End of user inputs  ############################################
###################################################################################


# do not change these:
SPd['Plant_TF'] = Plant
SPd['Controller'] = contObj
#
# Sanity check SPd
#
if task=='Optimize':
    delta = 100*SPd['scale_range']**(1.0/SPd['nvals'])-100  # %age change per step
    if delta < 3.0:  # 3%
        print(f' Warning: parameter delta is less than 3%')
        print(f'     nvals: {SPd['nvals']}    range: {SPd['scale_range']:4.2f}')
        print(f'     effective delta {delta:4.2f}%')
        print('    consider larger range or fewer nvals.')
        x = input(' To continue, <CR>')


print(f'Starting {task}')

if task == 'Optimize':

    est_search_time = cd447.estimate_time(SPd)
    print(f'Estimated Search Time: {est_search_time:8.2f} min.')
    x = input('Do you want to continue? <CR>')

    #
    #  Here is the actual parameter space search iteration
    #
    start_time = time.time()
    searchResD = cd447.optimize_ctl(Plant, contObj, SPd)

    duration, rate = cd447.compute_time(SPd,start_time)
    expo = len(contObj.params)
    print(f"\n        Actual Search Time: {duration:5.2f} minutes.  N sims = {(SPd['nvals']+1)**expo}  ({int(rate):,} ticks/min)")

    # WSNames = ['WTS','WOS', 'WSSE', 'WU', 'WM', 'Wbal', 'WSO']

    x = input('Do you want to see results printout? <CR>')
    cd447.printResults(searchResD)
    print('\n\n')
    x = input('Do you want to graph step responses? <CR>')
    cd447.graphResults(searchResD, 'ECE447 Example Optimizations: all schemes'+contObj.ctype)

    graph2Selection = ['WSO', 'Wbal','WSSE']
    cd447.graphResults(searchResD, 'ECE447 Example Optimizations: selected: '+contObj.ctype, wsnames=graph2Selection)
    plt.show()

if task == 'Rlocus':

    contObjRev =  cd447.RlocusWrapper(controllerD,Plant)

    controllerD_SR2 = controllerD
    controllerD_SR2['Params'] = contObjRev.params  # revised gains from clicked point

    cd447.StepResponseWrapper(SPd, controllerD_SR2 )

if task == 'StepResponse':

    cd447.StepResponseWrapper(SPd, controllerD )




