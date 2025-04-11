import control
import sys
import controlDesign447 as cd447

# eventually delete
import matplotlib.pyplot as plt
import numpy as np
import time


def cc(x):  # keeps code easier to read
    return np.conj(x)
#
# Control Design problem definition file template
#
# flags
PIDgains = False
PIDZeros = False


#  Process this command line
s = control.TransferFunction.s
allowedTasks = ['Rlocus', 'Optimize', 'StepResponse']
args = sys.argv
if len(args)<2:
    cd447.error('Please use a command line argument to select a task '+str(allowedTasks))
task = args[1]
if task not in allowedTasks:
    cd447.error(task + ' is not a recognized task '+str(allowedTasks))


###############################################
# #
#    Plant Setup
#

Plant = (s+8)/((s+3)*(s+6)*(s+10))   # Example 9.5 Nise Book


###############################################
#
#    Controller Setup
#
#   controller info goes in a dictionary:
#
controllerD = {}
controllerD['Name'] = 'Standardized Perf Test example'

controllerD['Ctype'] = 'PID'
Kd = 1 #  nominal for root locus


# Initial Start
PIDgains=True
Kp = 200.1   #
Ki = 400.2   #
Kd = 4.0
#controllerD['Name'] = 'Sp25 HW 9.7 Claude Solution Controller V01'


#
# set up regularization pole (make C(s) proper)
#
controllerD['RegSep'] = 150  # how far to separate regularization pole

###################### End of Controller setup
#
#chooses between how we initialize PID controller:
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
else:
    cd447.error('No Controller setting mode is set: one must be true: [PIDgains, PIDzeros]')


print('Initializing PID with: ', controllerD['Params'])
#  A good optimum: 48.5, .577, .752

contObj = cd447.controller(controllerD) # instantiate the controller as above



###############################################
#
#    Search Setup
#
# Search Parameter Dictionary
SPd = {}
SPd['Name'] = controllerD['Name']
SPd['Plant_TF'] = Plant
SPd['controller'] = contObj

# Desired Performance target
SPd['tsd']         =   0.6  # Desired settling time (sec)
SPd['pod']         =    20  # Desired overshoot ratio (e.g. 10%)
SPd['ssed']        =  0.01  # Desired steady state error
SPd['cu_max']      =   200  # Desired Maximum control effort (arbitrary units)
SPd['gm_db']       =    20  # Desired gain margin in dB (positive = stable)

# Search Parameters
SPd['scale_range'] =  3   # Search range multiplier
SPd['nvals']       =  20   # Number of points per parameter
SPd['tmax']        =  4*SPd['tsd']    #maximum simulation time
SPd['dt']          =  1/500          # Time step ( heuristic)
SPd['reportScheme']=  'WSO'  # which weights to print the limit-report on ('WSO' = TS + %OS)

# do not change these:
SPd['Name'] = controllerD['Name']
SPd['Plant_TF'] = Plant
SPd['controller'] = contObj
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




