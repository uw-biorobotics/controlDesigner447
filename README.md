Introduction to ECE447 Control Design Search Tools   (11-Apr-2025)

These tools support design of control systems for your plant. By using simulation and exhaustive
search, performance can be computed without simplifying assumptions such as dominant pole pairs.

There are two main files:  userSetupTemplate.py and controlDesign447.py.

userSetupTemplate.py   This where you specify up to four key parts of your problem: The plant, the controller,
       the performance specifications, and search parameters.
       The best practice is to start by copying the suppled userSetupTemplate.py file
       to a new one named "userSetupMY_PROB.py" (i.e. use the hw problem number in the filename).   userSetupMY_PROB.py
       is then specific to your problem.

       This file has three "verbs."  You will add one verb to the command line:
              Rlocus, StepResponse, and Optimize
       this selects how you want to use your design information.

       For example, to get a clickable root locus for your controller and plant:

       >python3 usersSetupMY_PROB.py  Rlocus <ENTER>



controlDesign447.py is a library of classes and functions that will support your design process.
       You should not have to edit it at all.

Typical work flow to complete a design is:

       1) select the type of controller you want to use (or are required to use).  The options
       are
              a) 'Kct' -  C(s) = K (K>0)              ('Constant gain K')
              b) 'LLC' -  C(s) = K (s-pole)/(s-zero)  ('Lead-Lag controller')
              c) 'PID' -  C(s) = Kp + Ki/s + Kds  or Kd(s+z1)(s+z2)/s    ('Proportional-Integral-Derivative Controller')

       You enter the 3-letter code as shown in the setup template file.

       2) do a manual design to start off with your controller parameters
        in the right "ballpark". Enter the manual parameters into your setup template file.

       3) Identify the region of the s-plane that may have the performance you want.

       4) run >python3 userSetupMY_PROB.py Rlocus

         You will see the root locus plot.    Click on the point you want and note
         the gain value (small font on top of the RL plot frame).

       5) Close the root locus plot.  The software will ask you for that gain value - enter it.

       6) Observe the computed step response for your new gain.   Is it somewhat close to meeting the specs?
            if so, go to step 8.
            if not, go back to step 4 and click a different spot - is the step response better?
            Settle for the best response you can get with a couple of tries.

       7) Once you have something reasonable, you are ready to optimize.
       Initialize your controller (in the setup file) to your best gain value from step 6.

       8) Set the search parameters in your userSetupMY_PROB.py file. (see Lecture and text for detail on this.)
       Typically you will start with a high range (e.g. 50) and a small number of values (~4) to quickly
       make sure things are working without errors.   Then raise nvals to say 40-50 and go have lunch.

       9) With the result of your long search, you can zoom in for fine tuning.  Selecting the set of gains
       you like best, reduce the range to say 1.5 and reduce the nvals to get at least 3%  change
       between search steps (smaller is a waste of time).

       10) Check for hitting range limits, and repeat searches from new starting points to perfect the results.


