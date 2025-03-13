import control
import sys
import cd as cd447

# eventually delete
import matplotlib.pyplot as plt
import numpy as np
import time

#
# Control Design problem definition file template
#

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
cdi = {}
cdi['Name'] = 'Nise Example 9.5 Controller'

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
cdi['Params'] = [56.50, 28.00, 84.00]  # after RL (bad SSE)
cdi['Params'] = [56.50, 28.00, 40]  # after 1st Optim.
cdi['Params'] = [54, 40.00, 20]  # after 2nd Optim.
cdi['Params'] = [260, 129, 4.6]  # Reset to NISE gains.
cdi['Params'] = [238, 150,  2.0]  # NISE + 1round -> CLOSE!
cdi['Params'] = [300, 120,  1]  # NISE + 2round --> very close on Ts



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
SPd['Name'] = cdi['Name']
SPd['plant'] = Plant
SPd['controller'] = contObj

# Desired Performance target
SPd['tsd']         =   0.4  # Desired settling time (sec)
SPd['pod']         =    20  # Desired overshoot ratio (e.g. 10%)
SPd['ssed']        =  0.01  # Desired steady state error
SPd['cu_max']      =   200  # Desired Maximum control effort (arbitrary units)
SPd['gm_db']       =    20  # Desired gain margin in dB (positive = stable)

# Search Parameters
SPd['scale_range'] =  2.7  # Search range multiplier
SPd['nvals']       =  15  # Number of points per parameter
SPd['tmax']        =  4*SPd['tsd']    #maximum simulation time
SPd['dt']          =  1/500          # Time step ( heuristic)
SPd['reportScheme']=  'WSO'  # which weights to print the limit-report on ('WSO' = TS + %OS)

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

    contObj = cd447.controller(cdi) # instantiate the controller as above

    #########################################################################
    # customize plot output for your specs
    #
    tsd = 0.4
    tmax = 4.0 * tsd
    plotHeight = 1.5 # step plus overshoot

    contObj.regSep = 10   # How far away is your reg pole?

    # make the controller TF
    C_TF = contObj.generate()

    INTERACTIVE = True

    if INTERACTIVE:

        plt.figure(figsize=(12,12))

        # gainrange = np.arange(0.0, 100, 0.01)
        # Root Locus argument is loop gain, CPH(s)
        control.root_locus_plot(C_TF*Plant) #, gains=gainrange)

        plt.show()


        #
        ##     Get gain(s) from RL click point
        #

        # ask user for click info from RL
        K = input('\n\n          What gain did you click? (x to quit) ')

        try:
            Kcl = float(K)  # closed loop gain constant
        except:
            quit() # e.g. enter 'x' to quit

        #
        # plR = float(input('Real part of pole? '))
        # plI = float(input('Imag part of pole? '))
        # J = 0+1j
        # poleloc = plR + J*plI

    else:  # if you're debugging and dont want to input same point over and over
        # use this:
        Kcl = xxx

    contObj.updateK(Kcl) # update controller scalar gain


    Kd = Kcl

    print('  New Parameters: ')
    for n in contObj.pnames:
        print(f' {n:10}',end='')
    print('')
    for p in contObj.params:
        print(f' {p:10.2f}',end='')
    print('')

    # print(f'Kp,Ki,Kd: {Kp:10.3f}, {Ki:10.3f}, {Kd:10.3f} ')
    # print(f'            {Kp:10.3e}, {Ki:10.3e}, {Kd:10.3e} ')



    # new controller TF with the clicked gain and with regularization
    C2_TF = contObj.generate()

    print('Computed OL poles(C2P(s), K) = ', control.poles(C2_TF*Plant))
    print('Computed OL zeros(C2_TFP(s), K) = ', control.zeros(C2_TF*Plant))
    print('Computed CL poles(C2_TFP(s), K) = ', control.poles(control.feedback(C2_TF*Plant,1)) )

    task = 'StepResponse'  # plot it!


if task == 'StepResponse':
    print('!!!!  Got here')
    tmax = SPd['tmax']
    tsd  = SPd['tsd']
    ProbName = SPd['Name']
    #
    #  Now step response of the designed controller and plant
    #
    plotHeight = 1.5
    CstepPlot = contObj.generate()
    t = np.linspace(0, tmax, 400)
    fig, ax = plt.subplots(1,2,figsize=(16,8))

    #
    # left box: step response
    #
    sys = control.feedback(CstepPlot*Plant,1)
    _,y1 = control.step_response(sys,t)
    # compensate for gain adjustment in plant
    ax[0].plot(t,y1)
    ax[0].set_title('Step Response')
    ax[0].set_xlim([0,tmax])
    # ax[0].set_ylim([0,2.0])
    ax[0].grid()

    # Horizontal window for Ts
    for limit in [0.98, 1.02]:
        ax[0].plot([tsd, tmax], [limit, limit], 'r')  # Ts response goal
    # vertical Ts deadline
    ax[0].plot( [tsd, tsd], [0, plotHeight], 'g')  # Ts goal


    #
    # right box: control effort
    H=1
    CE = control.feedback(CstepPlot, Plant*H)

    _,y2 = control.step_response(CE,t)

    #
    #  there seems to be a glitch with control effort
    #   for FIRST simulation sample
    t = t[1:]   # skip first sample
    y2 = y2[1:]
    ax[1].plot(t,y2)
    ax[1].set_title('Control Effort')
    # ax[1].set_xlim([0,tmax])
    # ax[1].set_ylim([-45, 45])
    ax[1].grid()


    fig.suptitle(ProbName)

    ts = cd447.settle_time(t,y1)
    pctOS = cd447.PCTovershoot(t,y1)
    sse = cd447.steady_state_error(t,y1)

    print('Performance Report:')
    print(f'Peak Control effort:     {max(y2[3:]):10.3f}')  # skip transient
    print(f'RMS Control effort:      {cd447.rms447(y2[3:]):10.3f}')

    print(f'Ts:      {ts:10.3f}')
    print(f'%os:     {pctOS:10.3f}')
    print(f'SSE:     {sse:10.3f}')

    plt.show()



