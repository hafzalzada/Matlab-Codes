 4 files have been set up in \verb"MATLAB" which work together to produce the impulse-response functions required. These files include \verb"stateDynamics.m" \verb"lossFunction.m", \\ \verb"optimizationExpectedUtility.m" and \verb"runningFile". 

The \verb"stateDynamics.m" file sets up a function file which defines the time evolutions, with its input values being the variables in each of the equations. 

The \verb"lossFunction.m" file is also a function file which calculates the losses produced at each time for the hinge loss function. 

Both of these files are used in the \verb"optimizationExpectedUtility.m" file to find the least expected loss produced over time. We define the control variables, \verb"x,y,z" and \verb"l" as 4$ vectors with equally spaced values between 0 and 1, which each aim to find the optimal levels of \verb"C,I,A,K" that produce the least loss for a given complexity R from the deviations in \verb"C, I, A" and \verb"K". We use 4-dimensional matrices to find the lowest expected loss found for every combination of control variables. 

The final \verb"C,I, A, K" values which produced the least loss are then recorded in the \verb"runningFile.m" file to produce the impulse response function for each attribute. 

All the parameter values, shocks, and weightings, are defined in the \verb"runningFile.m" file and the impulse-response function for each of the equations are then given as outputs in the form of graphs.
