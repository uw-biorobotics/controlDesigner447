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
#    Plant Setup
#

Plant = 2.4/((s+2)*(s+4))   # just an example system


############################################### controller setup section
#
#    Controller Setup
#
#   controller info goes in a dictionary:
#

controllerD = {}
controllerD['Name'] = 'Template File for Control Design Setup'


############################################
#
#    Start here for controller configuration
#

# two simple controller options:

# Constant single gain controller
#          The simplest possible controller but limited power
# controllerD['Ctype']  = 'Kct'  # a single gain controller
# controllerD['Params'] = [ 1000 ]  # example K=4

# Lead-Lag Compensator
#          A widely used basic controller
# controllerD['Ctype'] = 'LLC'  # lead/lag compensator controller
# pole = -2
# zero = -3
# controllerD['Params'] = [60,   pole, zero]  # example [ K, pole, zero]


#############################
#
# PID Controller
#        As used by 99% of industrial controllers
controllerD['Ctype'] = 'PID'
# flags (do not change these)
PIDgains = False
PIDZeros = False
##############################
#         Examples:   Two ways to initialize the PID controller
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

#
##############################################################  End of controller setup section





##############################################################################
##############################################################################
#   Demo problem using PID controller:
#
#   Example Problem solving sequence for a PID controller with the plant above
#

# 1)  Our first controller guess is based on a "manual" design:
# PIDZeros = True   # this tells system we start with gains Kd, z1, z2
# Kd = 1.0
# z1 = -6 + 6j
# z2 = cc(z1)

# 2)  We got a reasonable (but slow) step response clicking at Kd=0.06.  Let's
#     do a broad search around resulting gains: K_pid = [.72, 4.32, .06]
#     using the "Optimize" verb with search range 50, and 15 values (est 0.76 min)
PIDgains = True
Kp=0.72
Ki=4.32
Kd=0.06

# 3)  Pretty close step response is achieved with the "WSO" result:
#              Kp:   36.000 Ki: 76.1 Kd:    3.000
#

#
#        End of example.   (would do a couple more optimization rounds
#                           a for real problem)
#   a very nice result was achieved at  Kp: 72.000 Ki:  101.594 Kd:    3.780
#

##############################################################################
##############################################################################





#
# set up regularization pole (make C(s) proper)
#
controllerD['RegSep'] = 25  # how far to separate regularization pole from most negative real part


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



###############################################
#
#    Search Setup
#
# Search Parameter Dictionary
SPd = {}
SPd['Name'] = 'SearchSetup: ' + controllerD['Name']

# Desired Performance target
SPd['tsd']         =   0.6  # Desired settling time (sec)
SPd['pod']         =    20  # Desired overshoot ratio (e.g. 10%)
SPd['ssed']        =  0.01  # Desired steady state error
SPd['cu_max']      =   200  # Desired Maximum control effort (arbitrary units)
SPd['gm_db']       =    20  # Desired gain margin in dB (positive = stable)

# Search Parameters
SPd['scale_range'] =  2   # Search range multiplier
SPd['nvals']       =  15   # Number of points per parameter
SPd['tmax']        =  4*SPd['tsd']    #maximum simulation time
SPd['dt']          =  1/500          # Time step ( heuristic)
SPd['reportScheme']=  'WSO'  # which weights to print the limit-report on ('WSO' = TS + %OS)

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

    graph2Selection = ['WOS', 'Wbal','WSSE']
    cd447.graphResults(searchResD, 'ECE447 Example Optimizations: selected: '+contObj.ctype, wsnames=graph2Selection)
    plt.show()

if task == 'Rlocus':

    contObjRev =  cd447.RlocusWrapper(controllerD,Plant)

    controllerD_SR2 = controllerD
    controllerD_SR2['Params'] = contObjRev.params  # revised gains from clicked point

    cd447.StepResponseWrapper(SPd, controllerD_SR2 )

if task == 'StepResponse':

    cd447.StepResponseWrapper(SPd, controllerD )




