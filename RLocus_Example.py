import numpy as np
import control as control
import matplotlib.pyplot as plt
import cd as cd447
# import model9p6 as sm


def settle_time(t: np.ndarray, y: np.ndarray, threshold: float = 0.02) -> float:
    """Calculate settling time (time to reach and stay within ±2% of 1.0)"""
    final_value = y[-1]
    Amp = 1.0
    ts_maxval = Amp + threshold * Amp
    ts_minval = Amp - threshold * Amp
    #     for i in reversed(range(len(t))):
    #         if abs(y[i] - final_value) > settling_band:
    #             return t[i]
    for i in reversed(range(len(t))):
        if (y[i] > ts_maxval) or (y[i] < ts_minval):
            return t[i]
    return t[0]

def XXovershoot(t: np.ndarray, y: np.ndarray) -> float:
    """Calculate maximum overshoot as a ratio (Amp = no overshoot)"""
    # return max(y) / y[-1]
    return max(y)  # since input is UNIT step.

def PCTovershoot(t, y, amp=1.0):
    return 100*(max(y)-amp)/amp

def steady_state_error(t: np.ndarray, y: np.ndarray, goal=1.0):
    """Calculate steady state error"""
    return abs(goal - y[-1])

def rms447(x):
    x = np.array(x) # if its a list e.g.
    return np.sqrt(np.mean(x**2))


s = control.TransferFunction.s
ProbName = 'ECE447 HW9 Sp25  Prob 9.6'

###############################################
# #
#    Plant Setup
#

Plant = 0.8/(s+0.8)   # Example

ppars = {}
ppars['pole'] = -0.8
ppars['RLadjust'] = 1.0

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
z1 = -2
z2 = -0.25
cdi['Params'] = cd447.PIDKsFromZeros(1.0, z1, z2)  # example Kp, Ki, Kd
#  A good optimum: 48.5, .577, .752
cdi['Pnames'] = ['Kp','Ki','Kd']

contObj = cd447.controller(cdi) # instantiate the controller as above

contObj.regSep = 10   # move regularization pole closer?
C_TF = contObj.generate()


INTERACTIVE = True

if INTERACTIVE:

    plt.figure(figsize=(12,12))

    gainrange = np.arange(0.0, 100, 0.01)
    # Root Locus argument is loop gain, CPH(s)
    control.root_locus_plot(C_TF*Plant) #, gains=gainrange)

    plt.show()


    #
    ##     Compute new PID gains from RL click point
    #

    # ask user for click info from RL
    K = input('What gain did you click? ')

    try:
        Kcl = float(K)  # closed loop gain constant
    except:
        quit() # e.g. enter 'x' to quit

    plR = float(input('Real part of pole? '))
    plI = float(input('Imag part of pole? '))
    J = 0+1j
    poleloc = plR + J*plI

else:  # if you're debugging and dont want to input same point over and over
    # use this:
    Kcl = xxx
    poleloc =xxx


# Correct K for scaling of plant
try:
    print('Adjusting for Plant gain scale:',ppars['RLadjust'])
    Kcl = Kcl * ppars['RLadjust'] # like moving adj from plant to the controller
    Plant2 = Plant/ppars['RLadjust']      #(remove adj from plant mod)
except:
    Plant2 = Plant
    print('No Plant gain adjustment')
    pass # don't adjust Kcl



##################################################
#
#  Use Mag condition to get Kd
#    |KdC_hat(s=m)P(s=m)| = 1
#
m = poleloc # specifc point in S-plane
C_hat = (1/m)*((m-z1)*(m-z2))  # PID with Kd=1
P_hat = (m+8)/((m+3)*(m+6)*(m+10))
Kd = 1.0/(np.abs(C_hat)*np.abs(P_hat))

print('New Kd from Mag condition:', Kd)

Kp,Ki,Kd = cd447.PIDKsFromZeros(Kd, z1, z2)
print(f'Kp,Ki,Kd: {Kp:10.3f}, {Ki:10.3f}, {Kd:10.3f} ')
print(f'            {Kp:10.3e}, {Ki:10.3e}, {Kd:10.3e} ')


#
# Synthesize new PID ctl at the click point gain
#
try:
    Kd = Kd * ppars['RLadjust']   # move the gain adjust back to gain
    P = P/ppars['RLadjust']       # (i.e. take it away from controller)
except:
    pass # it's OK if there's no RLadjust value


contObj.params = [Kp, Ki, Kd]

# new controller TF with the clicked gain and with regularization
C2_TF = contObj.generate()

print('Computed OL poles(C2P(s), K) = ', control.poles(C2_TF*Plant2))
print('Computed OL zeros(C2_TFP(s), K) = ', control.zeros(C2_TF*Plant2))
print('Computed CL poles(C2_TFP(s), K) = ', control.poles(control.feedback(C2_TF*Plant2,1)) )

#
#  Now step response of the designed controller and plant
#

# customize for your specs

tsd = 0.4
tmax = 4.0 * tsd
plotHeight = 1.5 # step plus overshoot
t = np.linspace(0, tmax, 400)
fig, ax = plt.subplots(1,2,figsize=(16,8))

#
# left box: step response
#
sys = control.feedback(C2_TF*Plant2,1)
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
CE = control.feedback(C2_TF, Plant2*H)

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

ts = settle_time(t,y1)
pctOS = PCTovershoot(t,y1)
sse = steady_state_error(t,y1)

print('Performance Report:')
print(f'Peak Control effort:     {np.max(y2[3:]):10.3f}')  # skip transient
print(f'RMS Control effort:      {rms447(y2[3:]):10.3f}')

print(f'Ts:      {ts:10.3f}')
print(f'%os:     {pctOS:10.3f}')
print(f'SSE:     {sse:10.3f}')

plt.show()

