import math
import numpy as np
import matplotlib.pyplot as plt

import track_waypoints
import reward_func_with_speed

testwaypoints = track_waypoints.get_waypoints()

def get_corners_sharpness(interval_min, interval_max, waypoints, threshhold):
    corner_ranges = []
    waypoints_and_scores = []
    corner_sharpness_map = []
    for j in range(0, len(waypoints)-1):
        debug_output = "C@ " + str(j)
        corner_score = 0
        waypoint_interval_size = interval_max-interval_min + 1
        change_in_heading, distance = reward_func_with_speed.identify_corner(waypoints, [j, j+1], waypoint_interval_size)

        #print("WP {}, dH={} @ {}".format(j, change_in_heading, distance))
        if change_in_heading >= threshhold :
        #print(\"Adding {} to range, score: {}\".format(str(j), str(corner_score)[0:4]))
            waypoints_and_scores.append([j, change_in_heading])
        elif len(waypoints_and_scores) > 2 :
           #print(\"creating new range from {} to {}\".format(waypoints_and_scores[0][0], j))

           corner_ranges.append(waypoints_and_scores)
           waypoints_and_scores = []
        else:
            #to reduce "noise", reset collection if less than 2
           waypoints_and_scores = []
    
        curves_and_turn_direction = []
        #see if turning left or right by checing the first vs the last points in the curve
    for corner_range in corner_ranges :
        curve_direction = determine_turn_direction(waypoints, corner_range)
        curves_and_turn_direction.append([corner_range, curve_direction])

    return curves_and_turn_direction

def determine_turn_direction(waypoints, waypoint_range):
    first_waypoint = waypoints[waypoint_range[0][0]]
    last_waypoint = waypoints[waypoint_range[-1][0]]
    #get the point (Pc) in between the first and last points in the range
    Pc = [(last_waypoint[0] + first_waypoint[0])/2, (last_waypoint[1] + first_waypoint[1])/2]
    #get the median way point (Pm)
    median_waypoint_index = waypoint_range[0][0] + int(len(waypoint_range)/2)
    Pm = waypoints[median_waypoint_index]
    #get the points after PmNext) the median waypoint and use to determine heading
    PmNext = waypoints[median_waypoint_index+1]
    radius = reward_func_with_speed.dist(PmNext, Pm)
    #based on the heading, get a point on the left (Tl) and right (Tr) sides of the median waypoint on the track
    ##Tl = [radius * math.cos(math.radians(90)), radius * math.sin(math.radians(90))]
    ##Tr = [radius * math.cos(math.radians(270)), radius * math.sin(math.radians(270))]
    xDist = PmNext[0] - Pm[0]
    yDist = PmNext[1] - Pm[1]
    Tl = [PmNext[0] - (2*xDist), Pm[1] + yDist]
    Tr = [Pm[0] + xDist, PmNext[1]-(2*yDist)]
    #if the point on the left (Tl) is closer to Pc then the track is curving left, otherwise curving right
    dist_from_track_left_side = abs(reward_func_with_speed.dist(Pc, Tl))
    dist_from_track_right_side = abs(reward_func_with_speed.dist(Pc, Tr))
    curve_direction = "left" if dist_from_track_left_side < dist_from_track_right_side else "right"
    #print("Determined direction for wp {} to {} as {}".format(waypoint_range[0][0], waypoint_range[-1][0], curve_direction))
    return curve_direction

#corner_ranges, waypoint_corner_angles = get_corners_and_sharpness_scores(3,15,testwaypoints, 12)
corner_ranges = get_corners_sharpness(3,15, testwaypoints, 20)      
    
def test_show_plot():
    for waypoint in testwaypoints:
        plt.scatter(waypoint[0], waypoint[1], color='k')
    for waypoint_scores_with_turn in corner_ranges:
        for waypoint in testwaypoints:
            plt.scatter(waypoint[0], waypoint[1], color='k')
        waypoint_scores = waypoint_scores_with_turn[0]
        print("Corner Range from {} to {} turning to {}".format(waypoint_scores[0][0], waypoint_scores[-1][0], waypoint_scores_with_turn[1]))
        for k in waypoint_scores :
            plt.scatter(testwaypoints[k[0]][0], testwaypoints[k[0]][1], color='r', s=k[1]/1.5) 
        
        plt.show()