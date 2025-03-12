import numpy as np
import sys
import control
import matplotlib.pyplot as plt
import time

###########################################################
#
#   Design controllers by parameter space search
#       * K controllers
#       * lead-lag controllers
#       * PID controllers



#
#   Performance Evaluator Functions
#

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
#
# def XXovershoot(t: np.ndarray, y: np.ndarray) -> float:
#     """Calculate maximum overshoot as a ratio (Amp = no overshoot)"""
#     # return max(y) / y[-1]
#     return max(y)  # since input is UNIT step.

def PCTovershoot(t, y, amp=1.0):
    return 100*(max(y)-amp)/amp

def steady_state_error(t: np.ndarray, y: np.ndarray, goal=1.0):
    """Calculate steady state error"""
    return abs(goal - y[-1])

def get_control_effort(ctl, plant, H, t):
        # get control effort with system transformation
        _, u = control.step_response(control.feedback(ctl, plant*H), t)
        return u


#
#   Utility functions
#

def error(msg):
    print('\n\nError: ', msg)
    quit()

def PIDKsFromZeros(Kd,z1,z2):
    Kp= -1 * np.real(z1+z2)*Kd
    Ki = np.real(z1*z2)*Kd
    return (Kp, Ki, Kd)

def PIDZerosFromKs(Kp,Ki,Kd):
    z1,z2 = np.roots([1, Kp/Kd, Ki/Kd])
    return (z1,z2)

def get_features(c):
    poles = control.poles(c)
    zeros = control.zeros(c)
    # print('get_features: poles, zeros: ', poles, zeros)
    features = []
    for p in poles:
        features.append(np.real(p))
    for z in zeros:
        features.append(np.real(z))
    # print('get_features():',c, ' returning ',sorted(features) )
    # print('type (features): ',features[0], type(features[0]))
    return sorted(features)

def checkGainType(K):
    #
    #  Gains, Kx should be numerical and real and positive
    #
    OKType = False  #
    if type(K) == type(3.14):
            OKType = True
    elif type(K) == type(5):
            OKType = True
    elif type(K) == type(np.float64(1.7)):
            OKType = True
    if K < 0:
            OKType = False
    if OKType:
        return True
    else:
        return False

def checkPZType(K):
    #
    #  poles and zeros should be numerical or complex
    #
    OKType = False  #
    if type(K) == type(3.14):
            OKType = True
    if type(K) == type(1.0+3j):
            OKType = True
    elif type(K) == type(5):
            OKType = True
    elif type(K) == type(np.float64(1.7)):
            OKType = True
    if OKType:
        return True
    else:
        return False

#
#  Search time estimation
#
def estimate_time(d):
    # 196770  # simulation ticks per minute on this computer

    # For rate predictions
    try:
        with open('simrate_optigain.txt', 'r') as file:
            rate = int(file.read().strip())
        TicksPermin = rate
        print(f"Found rate file: {rate} simulation ticks per min.")
    except FileNotFoundError:
        TicksPermin = 3500000   # works for BH!

    exponent = len(d['controller'].params)
    estimated_time = ( (d['nvals']+1)**exponent * d['tmax']/d['dt']  )/  TicksPermin
    return estimated_time

#
#  Search Time: actual computation of
#
def compute_time(d,start_time):
    end_time = time.time()
    duration = (end_time - start_time) / 60 # min
    exponent = len(d['controller'].params)
    totalTicks = d['tmax']* (d['nvals'] + 1)**exponent / d['dt']  # total integration ticks
    rate = totalTicks / duration
    with open('simrate_optigain.txt', 'w') as file:  # save the simulation rate
        print(int(rate),file=file)
    return duration, rate

#
#  Controller class for optimization
#

class controller:
    def __init__(self,d):
        try:
            self.name = d['Name']
            if d['Ctype'] not in ['PID','LLC', 'Kct']:
                error('controller: Illegal controller type code: '+str(ctype))
            self.ctype = d['Ctype']
            if len(d['Params']) not in [1,2,3]:
                error('controller: Illegal gain vector:'+str(gains))
        except:
            error('controller: missing terms in setup dict.')

        self.regSep = 20  # separation of regularization pole (if needed e.g. PID)
        self.params = d['Params']
        self.pnames = d['Pnames']

    #
    #  Generate a transfer function for this controller
    #
    def generate(self):
        eid = 'controller:generate()' # id for error message clarity
        params = self.params

        #
        #  a bit of type checking then on to individual generators below
        #
        ############ PID
        if self.ctype == 'PID':
            try:
                gains =  params
            except:
                error(eid + ' Illegal PID parameters')
            if len(gains) != 3:
                error(eid + ' Illegal PID parameters: '+str(params))
            return self.PID(gains[2], gains=gains)

        ############ Lead Lag
        if self.ctype == 'LLC':  # lead-lag controller
            try:
                K = params[0]
                assert K>0
                p = params[1]
                assert checkPZType(p)
                z = params[2]
                assert checkPZType(z)
            except:
                error(eid + ' Illegal Lead-Lag parameter dict.')
            return self.LeadLag(params)

        ############ Constant Gain
        if self.ctype == 'Kct':   # constant gain controller
            try:
                K = self.params[0]
            except:
                print('>>>',self.params)
                error(eid + ' Illegal constant gain parameter vect.'+str(params))

            if not checkGainType(K):
                error(eid + ' Illegal data type for constant gain: '+str(K))
            return self.ConstCtl([K])


    def PID(self, K, gains=None, zeros=None):
        #
        #  Return a transfer function of a PID controller with
        #     appropriate regularization pole to make it "proper"
        #
        #   Two possible input vectors:  gains: [K, Ki,Kd]
        #                                zeros: [K, z1,z2]
        # K = Kd (yes, repeat it if gains are the input vector)
        s = control.TransferFunction.s
        # regularization_separation is distance ratio of reg. pole
        #       to most negative system feature
        regularization_separation = self.regSep
        #
        #  some error checks
        if gains is not None and len(gains)==3:  # set new gains if appropriate
            self.params=gains
        if gains is None and zeros is None:
            error('controller.PID: No gains or zeros given')
        if gains is not None and zeros is not None:
            error('controller.PID: Both gains AND zeros should not be specified')

        if gains:
            if K != self.params[2]:
                error('controller: Please set the K parameter equal to Kd')
            self.params = gains

        if zeros:
            z1 = zeros[0]
            z2 = zeros[1]
            self.params = PIDKsFromZeros(K, z1, z2)

        Kp, Ki, Kd = self.params
        cont1 = Kd*s + Kp + Ki/s
        regpole = regularization_separation * np.min(get_features(cont1))  # 20 is "big" and arbitrary
        regsys = np.abs(regpole) / (s-regpole)

        Cont = regsys * Kd*(s*s + (Kp/Kd)*s + Ki/Kd)/s
        print('regpole = ',regpole)
        print('features(cont1): ', get_features(cont1))
        self.name = 'PID'
        return Cont


    def LeadLag(self, p):
        #
        #  Return a transfer function of a Lead-lag
        #           compensator
        #
        #  p[0] = K
        #  p[1] = pole (negative real)
        #  p[2] = zero (negative real)
        #
        s = control.TransferFunction.s
        if len(p) != 3:
            error('controller.LeadLag: wrong number of control parameters: '+str(p))
        K = p[0]
        pole = p[1]
        zero = p[2]
        if np.real(pole) != pole:
            error('controller.LeadLag: pole must be real: '+str(pole))
        if pole > 0:
            error('controller.LeadLag: pole must be negative: '+str(pole))
        if np.real(zero) != zero:
            error('controller.LeadLag: zero must be real: '+str(zero))
        if zero > 0:
            error('controller.LeadLag: zero must be negative: '+str(zero))

        self.params = p      # what to optimize
        norm = np.abs(pole/zero)
        Cont = K * norm * (s-zero)/(s-pole)  # now K is magnitude @ s=0
        self.name = 'LeadLag'
        return Cont


    def ConstCtl(self, p):  # the simplest controller!
        eid = 'controller.ConstCtl:'
        if len(p) != 1:
            error(' gain must be single value list: [K]:'+str(p))
        if not checkGainType(p[0]):
            error(eid+' Constant gain must be real and positive')
        self.params = p
        self.name = 'ConstantGain'
        return control.TransferFunction(p[0],[1])  #


#
#   Evaluate Performance of a specific controller instance
#
def cost_eval(plant, CtlObj, t):
    # plant:   a python.control.LTI object
    # CtlObj:     a Controller object. (.generate())

    """
    Evaluate controller performance (of a plant(TF) and a Controller Object)
    Returns: ts, po, ss, cu, gm, y
    ts - settling time
    po - overshoot ratio
    ss - steady state error
    cu - control effort
    gm - gain margin
    y - unit step response
    """
    s = control.TransferFunction.s

    # Unity feedback
    H = control.TransferFunction([1], [1])

    try:
        Ctl_TF = CtlObj.generate()
        sys = control.feedback(Ctl_TF * plant, H)
    except:
        print(' >>>  Controller: ', Ctl_TF, Plant)
        error('\n >>> feedback system creation fail: Is controller proper?\n')

    # Closed loop system
    try:
        sys = control.feedback(Ctl_TF * plant, H)
        loop_gain = Ctl_TF * plant * H

        # Get gain margin
        gm, pm, wg, wp = control.margin(loop_gain)
        # gm is expressed as >1 for good margin. <1 for unstable.
        gm_db = 20 * np.log10(gm) if gm < 10000 else 80  # Cap at 80dB
        # Check stability using poles
        poles = control.poles(sys)
        if np.any(np.real(poles) > 0):
            print('positive poles!', poles)
            # return bad values so not saved as opt for any reason
            return 999, 999, 999, 9.999e7, gm_db, t

        # Get response and control effort
        t_out, y = control.step_response(sys, t)
        u = get_control_effort(Ctl_TF, plant, H, t)
        ts = settle_time(t, y)
        pctOvershoot = PCTovershoot(t, y, y[-1]) # defined relative to final value not 1.0
        ss = steady_state_error(t, y)
        cu = max(abs(u[5:])) # skip first few samples (internal state??)

        return ts, pctOvershoot, ss, cu, gm_db , y

    except:
        print('\n >>>  control gain evaluation failure \n')
        return 997, 997, 997, 1e7, 0, t


#
#   code to set up search params
#
# SPd['scale_range'] =     # Search range multiplier
# SPd['nvals']       =     # Number of points per parameter
# SPd['tmax']        =     # Maximum simulation time
# SPd['dt']          =     # Time step
# SPd['tsd']         =     # Desired settling time
# SPd['pod']         =     # Desired overshoot ratio
# SPd['ssed']        =     # Desired steady state error
# SPd['cu_max']      =     # Maximum control effort
# SPd['gm_db']       =     # Desired gain margin in dB
# SPd['reportScheme']=    # which weights to print limit report on

def optimize_ctl(plant, CtlObj, SPd):

    # unpack the search params.
    scale_range = SPd['scale_range']       # Search range multiplier
    nvals       = SPd['nvals']             # Number of points per parameter
    tmax        = SPd['tmax']              # Maximum simulation time
    dt          = SPd['dt']                # Time step
    tsd         = SPd['tsd']               # Desired settling time
    pod         = SPd['pod']               # Desired overshoot ratio
    ssed        = SPd['ssed']              # Desired steady state error
    cu_max      = SPd['cu_max']            # Maximum control effort
    gm_db       = SPd['gm_db']             # Desired gain margin in dB
    reportScheme = SPd['reportScheme']     # which weights to print limit report on

    #this hack allows normalizing pct overshoot without divide by zero errors.
    if pod  < 0.01:
        pod = 0.01   # 0.01 percent

    Ctl_TF = CtlObj.generate()
    t = np.arange(0, tmax, dt)
    start_time = time.time()
    #
    # # Performance weights indices
    # WTS, WOS, WSSE, WU, WM, Wbal = range(6)
    #  WS names:
    # WTS = settling time only
    # WOS = overshoot only
    # WU  = Control effort only
    # WM  = Gain margin only
    # Wbal = all of the above equal
    # WSO  = 0.5 on Ts and %OS only

    WSNames = ['WTS','WOS', 'WSSE', 'WU', 'WM', 'Wbal', 'WSO']

    # Define weight schemes
    weight_schemes = {
    "WTS": [f"Ts = {tsd:.3f}",  [1.0, 0.0, 0.0, 0.0, 0.0,0.0]],
    "WOS": [f"Overshoot = {pod:.2f}", [0.0, 1.0, 0.0, 0.0, 0.0]],
    "WSSE": [f"SSE = {ssed:.2f}",  [0.0, 0.0, 1.0, 0.0, 0.0]],
    "WU": [f"Control Effort Max = {cu_max:.2f}", [0.0, 0.0, 0.0, 1.0, 0.0]],
    "WM": [f"Gain Margin = {gm_db:.0f}dB",  [0.0, 0.0, 0.0, 0.0, 1.0]],
    "Wbal": ["Balanced",  [0.2, 0.2, 0.2, 0.2, 0.2]],
    "WSO": ["Ts and Overshoot",  [0.5, 0.5, 0.0, 0.0, 0.0]]
    }

    # Initialize results storage
    CurOptParms = {}  # current best values
    emin = {}   # current smallest perf error
    for scheme in WSNames:
        CurOptParms[scheme] = [-1] * len(CtlObj.params)   # flag, -1, signals that nothing found yet
        emin[scheme] = 99999.99

    # Setup search ranges
    scalef = np.sqrt(scale_range)
    ranges = []
    for p in CtlObj.params:  # only the needed params
        ranges.append((p/scalef, p*scalef))

    # Calculate (geometric) step sizes
    gdk=[] # geometric delt K
    for i,p in enumerate(CtlObj.params):
        gdk.append(scalef**(1.0/nvals))
    # for gn in ['Kp','Ki','Kd']:
    #     gdk[gn] = (k_ranges[gn][1] / k_ranges[gn][0])**(1.0/nvals)

    #
    #  Generate the values that will be searched for each param
    #
    srch_vals= {}
    for i,p in enumerate(CtlObj.params):
        srch_vals[i] = []
        for k in range(nvals+1):  # +1 covers top of range
            srch_vals[i].append(ranges[i][0] * gdk[i]**k)

#     |----------------------------|
    print("|" + "-"*nvals + "|")  # "bar graph header"

    if len(CtlObj.params)==3:
        #
        # 3 Parameter optimization loop
        #
        results = {}
        for P1 in srch_vals[0]:
            print('.', end='', flush=True)
            for P2 in srch_vals[1]:
                for P3 in srch_vals[2]:
                    #  TBD: code inside loop is ALMOST identical for all three param lengths
                    #
                    #  set params for the controller under test
                    #
                    CtlObj.params = [P1,P2,P3]
                    #
                    #   Evaluate cost of this controller
                    #
                    ts1, pos1, ss, cu, gmr, y = cost_eval(plant, CtlObj,t)

                    #
                    #  Deterrmine optimality
                    #
                    if ts1 < 900:  # System is stable
                        # Compute performance errors w.r.t. specification
                        errors = [
                            abs(ts1-tsd)/tsd,
                            abs(pos1-pod)/pod,  # overshoot as percent
                            abs(y[-1]-1.0),     # SSE vs unit step
                            abs(cu-cu_max),     # control effort difference
                            abs(gmr-gm_db)/gm_db  # normalized gain marg.
                            ]

                        # Evaluate each weight scheme
                        for scheme in WSNames:
                            weighted_err = 0.0
                            for i,e in enumerate(errors):
                                weighted_err += e*weight_schemes[scheme][1][i]
                            # weighted_err = errors @ weight_schemes[scheme][1]
                            if weighted_err < emin[scheme]:
                                emin[scheme] = weighted_err  # keep track of best gains for each scheme
                                CurOptParms[scheme] = CtlObj.params # current gains/params
                    else:
                        pass   # ignore unstable param sets.

    #
    # 2 Parameter optimization loop
    #
    elif len(CtlObj.params)==2:
        results = {}
        for P1 in srch_vals[0]:
            print('.', end='', flush=True)
            for P2 in srch_vals[1]:
                #
                #  Build the controller under test
                #
                CtlObj.params = [P1,P2]
                #
                #   Evaluate cost of this controller
                #
                ts1, pos1, ss, cu, gmr, y = cost_eval(plant, CtlObj, t)

                #
                #  Deterrmine optimality
                #
                if ts1 < 900:  # System is stable
                    # Compute performance errors w.r.t. specification
                    errors = [
                        abs(ts1-tsd)/tsd,
                        abs(pos1-pod)/pod,  # overshoot as percent
                        abs(y[-1]-1.0),     # SSE vs unit step
                        abs(cu-cu_max),     # control effort difference
                        abs(gmr-gm_db)/gm_db  # normalized gain marg.
                        ]

                    # Evaluate each weight scheme
                    for scheme in WSNames:
                        weighted_err = 0.0
                        for i,e in enumerate(errors):
                            weighted_err += e*weight_schemes[scheme][1][i]
                        # weighted_err = errors @ weight_schemes[scheme][1]
                        if weighted_err < emin[scheme]:
                            emin[scheme] = weighted_err  # keep track of best gains for each scheme
                            CurOptParms[scheme] = CtlObj.params # current gains/params
                else:
                    pass   # ignore unstable param sets.
                    #
    # 1 Parameter optimization loop
    #
    elif len(CtlObj.params)==1:
        results = {}
        for P1 in srch_vals[0]:
            print('.', end='', flush=True)
            #
            #  Build the controller under test
            #
            CtlObj.params = [P1]
            #
            #   Evaluate cost of this controller
            #
            ts1, pos1, ss, cu, gmr, y = cost_eval(plant, CtlObj, t)

            #
            #  Determine optimality
            #
            if ts1 < 900:  # System is stable
                # Compute performance errors w.r.t. specification
                errors = [
                    abs(ts1-tsd)/tsd,
                    abs(pos1-pod)/pod,  # overshoot as percent
                    abs(y[-1]-1.0),     # SSE vs unit step
                    abs(cu-cu_max),     # control effort difference
                    abs(gmr-gm_db)/gm_db  # normalized gain marg.
                    ]

                # Evaluate each weight scheme
                for scheme in WSNames:
                    weighted_err = 0.0
                    for i,e in enumerate(errors):
                        weighted_err += e*weight_schemes[scheme][1][i]
                    # weighted_err = errors @ weight_schemes[scheme][1]
                    if weighted_err < emin[scheme]:
                        emin[scheme] = weighted_err  # keep track of best gains for each scheme
                        CurOptParms[scheme] = CtlObj.params # current gains/params
            else:
                pass   # ignore unstable param sets.
    else:
        error('optimize_ctl: CtlObj.params vector must be length [1,2,3]: '+str(CtlObj.params))


    print("\n\nOptimization search complete!\n")

    optResults = {}

    # basics
    optResults['WSNames'] = WSNames  # names of different weight schemes
    optResults['Tmax'] = tmax
    optResults['Dt'] = dt
    optResults['Pp'] = CtlObj.regSep

    d = {}
    wst = {}
    for n in WSNames:
        d[n] = weight_schemes[n][1]
        wst[n] = weight_schemes[n][0]
    optResults['Weights'] = d        # weight vectors indexed by scheme name
    optResults['WeightStrings'] = wst
    optResults['OptParams'] = CurOptParms  # best parameters found (for all schemes)
    goals = {}
    goals['tsd'] = tsd        # desired settle time
    goals['pod'] = pod        # desired percent overshoot (e.g. "5.0"%)
    goals['ssed'] = ssed
    goals['cu_max'] = cu_max
    goals['gm_db'] = gm_db
    optResults['Goals'] = goals
    optResults['Controller'] = CtlObj  #  cd447.controller() object
    optResults['Plant'] = plant        # control.TransferFunction()


    #
    # Check search boundaries: Report if an optimum is on edge of search.
    #
    NoBoundaries = True
    parNames = CtlObj.pnames
    print('Checking Search Boundaries (', reportScheme,')')
    # Go through the found params:
    nparams = len(optResults['OptParams'][reportScheme])

    print(f'Diagnostic: {reportScheme}  {parNames} npars: {nparams}')
    for pn in range(nparams):
        P_opt = optResults['OptParams'][reportScheme][pn]
        P_minval = srch_vals[pn][0]
        P_maxval = srch_vals[pn][1]
        if P_opt in [P_maxval, P_minval]:
            name = CtlObj.pnames[pn]
            print(f"   Search boundary reached: {name:6} {'min' if P_opt == P_minval else 'max'}")
            NoBoundaries = False

    if NoBoundaries:
        print('\n                   All Results INSIDE Boundaries\n')

    return optResults

def printResults(R):
    t = np.arange(0, R['Tmax'], R['Dt'])

    params = R['OptParams']
    weight_schemes = R['Weights']
    weight_strings = R['WeightStrings']

    print('Goals:')
    for gn in R['Goals'].keys():
        print(f'   {gn:6}: {R['Goals'][gn]}',end='')
    print('')
    #
    # Plot and print results for each Scheme
    #
    names = R['Controller'].pnames   # same in all schemes
    for scheme in R['WSNames']:
        print('\n\nReporting: ',scheme)
        params = R['OptParams'][scheme]
        CtlObj = R['Controller']
        CtlObj.params = params     # set the current params for printing
        Ctl_TF = CtlObj.generate()

        #
        #  print the params
        #
        print(f'\nScheme: {weight_strings[scheme]:20s} ',end='')
        names = CtlObj.pnames
        for i,p in enumerate(params):
            print(f'{names[i]}: {p:8.3f} ',end='')
        print('')
        # if min(params)< 0.01:   # use more sig figs in this case
        #     print(f"[{weight_strings[scheme]:20s} {}: {Kp_opt[scheme]:12.9f}  Ki: {Ki_opt[scheme]:12.9f}  Kd: {Kd_opt[scheme]:12.9f} ")
        # else:
        #     print(f"[{weight_strings[scheme]:20s} Kp: {Kp_opt[scheme]:6.3f}  Ki: {Ki_opt[scheme]:6.3f}  Kd: {Kd_opt[scheme]:6.3f}  ")
        # print(f"(Kp, Ki, Kd = {Kp_opt[scheme]:12.9f}, {Ki_opt[scheme]:12.9f},  {Kd_opt[scheme]:12.9f} ) ")
        ## cost_eval
        ts, pctOS, ss, cu, gm_db , y = cost_eval(R['Plant'], CtlObj , t)

        print(f'Settling Time: {ts:6.3f}  Overshoot: {pctOS:6.3f} %   SSE: {ss:6.3f} Ctl Effort: {cu:6.3f}   Gain Marg: {gm_db:6.1f} dB  ]')


def graphResults(R,title='ECE447, Sp25', wsnames=None):
    #
    #  R = a results dictionary
    #
    t = np.arange(0, R['Tmax'], R['Dt'])
    if wsnames is None:
        wsnames = R['WSNames']

    # Unity feedback
    H = control.TransferFunction([1], [1])

    plt.figure(figsize=[11,11])
    # plot the best step response for each weight scheme
    for sn in wsnames:
        # Define controller
        CtlObj = R['Controller']
        CtlObj.params = R['OptParams'][sn]
        Ctl_TF = CtlObj.generate()
        sys = control.feedback(Ctl_TF * R['Plant'], H)
        T,Y = control.step_response(sys, t)
        plt.plot(T,Y)

    plt.axis([0, R['Tmax'], 0, 2.0])

    # show colored lines for step response goals
    #
    # Horizontal window for Ts
    for limit in [0.98, 1.02]:
        plt.plot([R['Goals']['tsd'], R['Tmax']], [limit, limit], 'r')  # Ts response goal
    # vertical Ts deadline
    plotHeight = 1.5 # step plus overshoot
    plt.plot( [R['Goals']['tsd'], R['Goals']['tsd']], [0, plotHeight], 'g')  # Ts goal

    ax = plt.gca()
    ax.set_ylim([0.0, plotHeight])
    ax.legend(wsnames)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('System Output')
    plt.title('Optimized Step Responses: '+title)
    plt.grid()




