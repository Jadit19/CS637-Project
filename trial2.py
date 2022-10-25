import numpy as np
import matplotlib.pyplot as plt
import time, random
from pidcontroller import AltitudeController

# Quadcopter Constants
# Acceleration due to gravity
G = 9.80665
# Time Interval
DT = 0.001
# Vehicle params
MAXRPM = 30000
B      = 5.30216718361085E-05 # Thrust coefficient
M      = 50                  # Mass (kg)

# Important parameters
ALTITUDE_START  = 0
ALTITUDE_TARGET = 10
ALTITUDE_TOLERANCE = .0001 # level-off velocity

# PID parameters
ALT_P = 1.25
VEL_P = 1.5
VEL_I = 1.0
VEL_D = 0.05

# Plots results from CSV log file
def plot(logfilename):

    data = np.genfromtxt(logfilename, delimiter=',', skip_header=1)
    t = data[:,0]
    z = data[:,3]

    plt.plot(t, z)
    plt.xlabel('time (sec)')
    plt.ylabel('altitude (m)')
    plt.ylim([min(z)-1, max(z)+1])
    plt.grid()
    plt.show()

if __name__ == '__main__':
    # initial conditions
    t     = 0
    z     = ALTITUDE_START
    dzdt  = 0
    u     = 0

    # make CSV file name from these params
    filename = '%04.f-%04.f_%3.3f-%3.3f-%3.3f-%3.3f.csv' % (ALTITUDE_START, ALTITUDE_TARGET, ALT_P, VEL_P, VEL_I, VEL_D)
    logfile = open(filename, 'w')
    logfile.write('t, dzdt2, dzdt, z, u\n')

    # Create PID controller
    pid = AltitudeController(ALTITUDE_TARGET, ALT_P, VEL_P, VEL_I, VEL_D)

    # Loop until level-off
    while True:
        # If altitude has leveled off, halt
        if abs(z) != 0 and abs(dzdt) < ALTITUDE_TOLERANCE:
            break
        # Get correction from PID controller
        u = pid.u(z, dzdt, DT)
        u = max(0, min(1, u)) # Constrain correction to [0,1] to represent motor value
        thrust = B * (u * MAXRPM * np.pi / 30) ** 2 / M # Convert motor value to vertical thrust     
        dzdt2 = thrust - G # Subtract G from thrust to get net vertical acceleration
        dzdt = dzdt + dzdt2 * DT  # Integrate net vertical acceleration to get vertical velocity    
        z = z + dzdt*DT # Integrate vertical velocity to get altitude
        logfile.write('%3.3f,%3.3f,%3.3f,%3.3f,%3.3f\n' % (t, dzdt2, dzdt, z, u))
        t = t + DT

    logfile.close()

    plot(filename)