import numpy as np
import sys
import time
import control
# import controlDesigner447 as cd447
import cd as cd447
import matplotlib.pyplot as plt

#
# Control Design problem definition file template
#

s = control.TransferFunction.s

###############################################
# #
#    Plant Setup
#

Plant = 0.8/(s+0.8)   # Example

###############################################
#
#    Controller Setup
#
#   controller info goes in a dictionary:
#
cdi = {}
cdi['Name'] = 'Setup Example Controller'

# Constant single gain controller
# cdi['Ctype']  = 'Kct'  # a single gain controller
# cdi['Params'] = [ 21]  # example K=4
# cdi['Pnames'] = [ 'K' ]

# Lead-Lag Compensator
# cdi['Ctype'] = 'LLC'  # lead/lag compensator controller
# cdi['Params'] = [60,  -1, -4]  # example [ K, pole, zero]
# cdi['Pnames'] = ['K', 'pole', 'zero']

# PID Controller
cdi['Ctype'] = 'PID'  # PID controller

cdi['Params'] = [ 100, 5, 2.0 ]  # example Kp, Ki, Kd
cdi['Params'] = [ , 1.5, .7 ]  # example Kp, Ki, Kd


z1 = -2
z2 = -0.25
cdi['Params'] = cd447.PIDKsFromZeros(1.0, z1, z2)  # example Kp, Ki, Kd
print('Initializing PID with: ', cdi['Params'])
#  A good optimum: 48.5, .577, .752
cdi['Pnames'] = ['Kp','Ki','Kd']

contObj = cd447.controller(cdi) # instantiate the controller as above



###############################################
#
#    Search Setup
#
# Search Parameter Dictionary
SPd = {}
SPd['plant'] = Plant
SPd['controller'] = contObj

# Desired Performance target
SPd['tsd']         =   0.4  # Desired settling time (sec)
SPd['pod']         =    10  # Desired overshoot ratio (e.g. 10%)
SPd['ssed']        =  0.01  # Desired steady state error
SPd['cu_max']      =      # Desired Maximum control effort (arbitrary units)
SPd['gm_db']       =    20  # Desired gain margin in dB (positive = stable)

# Search Parameters
SPd['scale_range'] =  1.50  # Search range multiplier
SPd['nvals']       =  12  # Number of points per parameter
SPd['tmax']        =  3*SPd['tsd']         #maximum simulation time
SPd['dt']          =  3*SPd['tsd']/1500    # Time step ( heuristic)
SPd['reportScheme']=  'WSO'  # which weights to print the limit-report on ('WSO' = TS + %OS)

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
print(f"\n        Actual Search Time: {duration:5.2f} minutes.  N sims = {(SPd['nvals']+1)**expo}  ({rate:8.1f} ticks/min)")

 # WSNames = ['WTS','WOS', 'WSSE', 'WU', 'WM', 'Wbal', 'WSO']

x = input('Do you want to see results printout? <CR>')
cd447.printResults(searchResD)
print('\n\n')
x = input('Do you want to graph step responses? <CR>')
cd447.graphResults(searchResD, 'ECE447 Example Optimizations: all schemes'+contObj.ctype)

graph2Selection = ['WOS', 'Wbal','WSSE']
cd447.graphResults(searchResD, 'ECE447 Example Optimizations: selected: '+contObj.ctype, wsnames=graph2Selection)
plt.show()
