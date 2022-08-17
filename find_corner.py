import math
import numpy as np
import matplotlib.pyplot as plt

import track_waypoints
import reward_func_with_speed

testwaypoints = track_waypoints.get_waypoints()

def get_corners_and_sharpness_scores(interval_min, interval_max, waypoints, threshhold):
    corner_ranges = []
    waypoints_and_scores = []
    waypoint_cornering = []
    corner_factors = []
    for j in range(0, len(waypoints)-1):
        ##debug_output = \"C@ \" + str(j)
        corner_score = 0
        for steps in range(interval_min,interval_max):
            corner_check = reward_func_with_speed.identify_corner(waypoints, [j, j+1], steps)
            corner_factor = (corner_check[0]/corner_check[1])/steps
            corner_score = corner_score + corner_factor

        corner_factors.append(corner_factor)
        
        ##debug_output = debug_output + \" \" + str(corner_factor)[0:4] + \",\"
        ##print(debug_output)#\"Adding {} to range, score: {}\".format(str(j), str(corner_score)[0:4]))
        if corner_score >= threshhold :
        #print(\"Adding {} to range, score: {}\".format(str(j), str(corner_score)[0:4]))
            waypoints_and_scores.append([j, corner_score])
        elif len(waypoints_and_scores) > 2 :
           #print(\"creating new range from {} to {}\".format(waypoints_and_scores[0][0], j))
           corner_ranges.append(waypoints_and_scores)
           waypoints_and_scores = []
        else:
           waypoints_and_scores = []
        
    waypoint_corner_angles = []
    for current_waypoint_index in range(0, len(corner_factors)-1) :
        #negative - curving to left
        #positive - curve to right
        prev_index = (len(corner_factors)-1 if current_waypoint_index == 0 else current_waypoint_index) - 1
        corner_angle = corner_factors[current_waypoint_index] - corner_factors[prev_index]
        print("Waypoint {} (prev {}) factor: {} delta: {}".format(current_waypoint_index, prev_index, str(corner_factors[current_waypoint_index])[0:6], str(corner_angle)[0:6] ))
        waypoint_corner_angles.append([current_waypoint_index, corner_angle])
    return corner_ranges, waypoint_corner_angles

for waypoint in testwaypoints:
    plt.scatter(waypoint[0], waypoint[1], color='k')

corner_ranges, waypoint_corner_angles = get_corners_and_sharpness_scores(3,22,testwaypoints, 24)

for waypoint_corner_angle in waypoint_corner_angles :
    if (abs(waypoint_corner_angle[1])>3):
        waypoint_index = waypoint_corner_angle[0]
        size = round(abs(waypoint_corner_angle[1]),2)
        #print("Plotting: {}, {}, s={}\".format(testwaypoints[waypoint_index][0], testwaypoints[waypoint_index][1], size))
        plt.scatter(testwaypoints[waypoint_index][0], testwaypoints[waypoint_index][1], color='r', s=size)

plt.show()        
    
for waypoint_scores in corner_ranges:
    for waypoint in testwaypoints:
        plt.scatter(waypoint[0], waypoint[1], color='k')
    
    print("Corner Range from {} to {}".format(waypoint_scores[0][0], waypoint_scores[-1][0]))
    for k in waypoint_scores :
       plt.scatter(testwaypoints[k[0]][0], testwaypoints[k[0]][1], color='r', s=k[1]/1.5) 
    
    plt.show()