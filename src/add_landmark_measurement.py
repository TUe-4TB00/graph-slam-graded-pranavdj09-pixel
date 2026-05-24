import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # Determine the correct rotation (bearing) and distance from X(4) to L(2) 
    theta_X4 = math.pi/2   # The robot is oriented 90 degrees anti-clockwise at X4

    # The vector connecting the robot to the landmark (r), has to be l2x - x4 and l4y-y4, I wrote the variables wrong   
    x4_l2x = -(4+math.sqrt(2)-4)
    y4_l2y = -(math.sqrt(2)-2)

    # Measuring the angle made by r in the global cartesian coordinates (x1,y1)
    theta_global = math.atan2(y4_l2y,x4_l2x)

    # The orientation of the robot
    theta_local = theta_X4

    # Angle made by r with the robot's local x-axis at X4:
    rotation = theta_global-theta_local

    distance = math.sqrt((x4_l2x)**2+(y4_l2y)**2)
    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), gtsam.Rot2(rotation), distance, MEASUREMENT_NOISE))
    return graph