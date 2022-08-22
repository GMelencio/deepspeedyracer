import math
from operator import truediv
import numpy as np
import matplotlib.pyplot as plt

import track_waypoints
import reward_func_with_speed

testwaypoints = track_waypoints.get_waypoints()

def determine_turn_direction(waypoints, waypoint_range):
    first_waypoint = waypoints[waypoint_range[0][0]]
    last_waypoint = waypoints[waypoint_range[-1][0]]
    #get the point (Pc) in between the first and last points in the range
    Pc = [(last_waypoint[0] + first_waypoint[0])/2, (last_waypoint[1] + first_waypoint[1])/2]
    #get the median way point (Pm)
    median_waypoint_index = waypoint_range[0][0] + int(len(waypoint_range)/2)
    Pm = waypoints[median_waypoint_index]
    Pm2m = waypoints[waypoint_range[0][0] + int(len(waypoint_range)/4)]
    #get the points after PmNext) the median waypoint and use to determine heading
    ##PmNext = waypoints[median_waypoint_index+2]
    PmNext = waypoints[waypoint_range[-1][0] - 2]
    radius = reward_func_with_speed.dist(PmNext, Pm)
    #based on the heading, get a point on the left (Tl) and right (Tr) sides of the median waypoint on the track
    #degreeLeftDiff = math.acos(/radius)
    #Tl = [radius * math.cos(math.radians(90))+Pm[0], radius * math.sin(math.radians(90))+Pm[1]]
    #Sin(t) = Op/Hyp
    #oppositeSide = reward_func_with_speed.dist(PmNext, [Pm[0], radius + Pm[1]])
    #theta = math.degrees(math.asin(oppositeSide/radius))
    foo, angle = reward_func_with_speed.polar(Pm2m[0]-PmNext[0], Pm2m[1]-PmNext[1])
    theta=angle+270
    Tl = [radius * math.cos(math.radians(theta))+Pm[0], radius * math.sin(math.radians(theta))+Pm[1]]
    Tr = [radius * math.cos(math.radians(270))+Pm[0], radius * math.sin(math.radians(270))+Pm[1]]
    xDist = PmNext[0] - Pm[0]
    yDist = PmNext[1] - Pm[1]
    Tl2 = [(PmNext[0] - (2*xDist)), Pm[1] + yDist]
    Tr2 = [Pm[0] + xDist, PmNext[1]-(2*yDist)]
    #if the point on the left (Tl) is closer to Pc then the track is curving left, otherwise curving right
    dist_from_track_left_side = abs(reward_func_with_speed.dist(Pc, Tl2))
    dist_from_track_right_side = abs(reward_func_with_speed.dist(Pc, Tr2))
    curve_direction = "left" if dist_from_track_left_side < dist_from_track_right_side else "right"
    #print("Determined direction for wp {} to {} as {}".format(waypoint_range[0][0], waypoint_range[-1][0], curve_direction))
    #curve_waypoints = [waypoints[x[0]] for x in waypoint_range]
    #for point in curve_waypoints:
    #    plt.scatter(point[0], point[1], color='k')
    #plt.show()
    #plt.scatter(Pc[0], Pc[1], color='green')
    #plt.scatter(Pm[0], Pm[1], color='blue')
    #plt.scatter(Tl[0], Tl[1], color='orange')
    #plt.scatter(Tr[0], Tr[1], color='cyan')
    #plt.scatter(Tl2[0], Tl2[1], color='yellow')
    #plt.scatter(Tr2[0], Tr2[1], color='pink')
    #["red","green","blue","yellow","pink","black","orange","purple","beige","brown","gray","cyan","magenta"]
    return curve_direction, [Pm, Pc, Tl, Tr, Tl2, Tr2]

#corner_ranges, waypoint_corner_angles = get_corners_and_sharpness_scores(3,15,testwaypoints, 12)



def test_show_plot():
    for waypoint in testwaypoints:
        plt.scatter(waypoint[0], waypoint[1], color='k')
    for waypoint_scores_with_turn in corner_ranges:
        for waypoint in testwaypoints:
            plt.scatter(waypoint[0], waypoint[1], color='k')
        waypoint_scores = waypoint_scores_with_turn[0]
        turn_direction = waypoint_scores_with_turn[1][0]
        turn_data = waypoint_scores_with_turn[1][1]
        median = turn_data[0]
        center = turn_data[1]
        left1 = turn_data[2]
        right1 = turn_data[3]
        left2 = turn_data[4]
        right2 = turn_data[5]
        print("Corner Range from {} to {} turning to {}".format(waypoint_scores[0][0], waypoint_scores[-1][0], turn_direction))
        for k in waypoint_scores :
            plt.scatter(testwaypoints[k[0]][0], testwaypoints[k[0]][1], color='red', s=k[1]/1.5) 
        plt.scatter(median[0], median[1], color='green')
        plt.scatter(center[0], center[1], color='blue')
        plt.scatter(left1[0], left1[1], color='orange')
        plt.scatter(right1[0], right1[1], color='cyan')
        plt.scatter(left2[0], left2[1], color='yellow')
        plt.scatter(right2[0], right2[1], color='pink')

        plt.show()



def get_ideal_speed_and_heading(closest_waypoints, waypoints, corner_ranges):
    #if not in a corner
    is_in_corner = False
    
    current_corner_range = []
    for corner_range in corner_ranges:
        if (closest_waypoints[0] >= corner_range[0] and closest_waypoints[0] <= corner_range[-1]) or (closest_waypoints[1] >= corner_range[0] and closest_waypoints[1] <= corner_range[-1]) :
            is_in_corner = True
            current_corner_range = corner_range
            
    
    if is_in_corner :
        #check how far we are from corner exit
        is_close_to_corner_exit = False
        if is_close_to_corner_exit:
            return "TODO"
            #increase lookahead factor to 2x of standard
            #reinstate speed reward based on how far is the corner exit 
        #set target steering point to standard lookahead factor
        #reduce speed reward threshhold
    #if not in a corner
        #TO DO
    return False


profiles = [
            [3, 3, 10], #allows gap at big tunr on right but not much on lower left and upper left
            [3, 3, 12], # too many small gaps, does not detect upper left at all
            [3, 3, 9], # - similar to 0
            [3, 3, 8],  # - similar to 0
            [3, 4, 7.5],  # - small gap at right and top left but no covrage of lower left horizontal
            [3, 4, 6],  # -no gap at right and no covrage of lower left horizontal
            [4, 4, 7.5],  # - no gap at right, covers lower left in two parts, but also zigzag
            [4, 4, 8],  # - no gap at right, covers lower left in two parts, no zigzag
            [4, 4, 7.5],  # - no gap at right, covers lower left in two parts, covers zigzag
            [4, 4, 8.5],  # - tiny gap at right(1), no lower left horiz, no zigzag
            [5, 4, 8.5],  # - no gap at right, covers all lower left, zigzag(8)
            [4, 4, 9],   # - tiny gap at right(1), no lower left horiz, no zigzag
            [4, 4, 8.25],   # - no gap at right, no lower left horiz, no zigzag
            [4, 4, 10],   # - gap at right(3), no lower left horiz, no zigzag
            [4, 3, 10],   # - gap at right(3), no lower left horiz, no zigzag, marks low shallow curve at top
            [5, 3, 10],   # - gap at right, no lower left horiz in 2 parts, has zigzag, marks low shallow curve at top
            [5, 3, 10.8],   # - gap at right, no lower left horiz in 2 parts, has zigzag, marks low shallow curve at top
            [14, 2, 23],   # - default - covers turns in a long way
            [4, 4, 11],  # - gap at right, no left horizontal, no zigzag
            [4, 3, 11],  # - gap at right, no left horizontal, no zigzag
            [5, 3, 11],  # - no gap at right, left horizontal in two parts (6 & 4), has top shallow curve, no zigzag
            [5, 3, 12],  # - gap at right, left horizontal in two parts (5 & 4), has top shallow curve(4), no zigzag  # IDEAL
            [4, 3, 12],  # - gap at right but earlier detect, no left horizontal, no top shallow curve(4), no zigzag
            [7, 3, 12],  # - no gap at right, covers all lower left corner, has top shallow curve, full corner at rop, no zigzag early detect sharp turn
            [5, 3, 12],  # - no gap at right, covers all lower left corner, has top shallow curve, full corner at rop, no zigzag early detect sharp turn
            ]

# 18 score 7.159816682017379
# 19 score 9.018942006214132
# 20 score 11.72114630683103
# 21 score 15.321606054217114
# 22 score 10.268200320443839
# 23 score 8.078065319058737

turn_profile = profiles[-1]

corner_ranges, all_wp_scores = reward_func_with_speed.get_corners_sharpness(turn_profile[0], turn_profile[1], turn_profile[2], testwaypoints)      

def test_corner_detection():
    i_index = 0
    for wp_score in all_wp_scores:
        print("WP # {} score {}".format(i_index, wp_score))
        i_index = i_index + 1
    
    for waypoint in testwaypoints:
        plt.scatter(waypoint[0], waypoint[1], color='k')
    for waypoint_scores_in_corners in corner_ranges:
        first_index_in_corner = waypoint_scores_in_corners[0][0]
        last_index_in_corner = waypoint_scores_in_corners[-1][0]
        print("plotting corner from {} to {}".format(first_index_in_corner, last_index_in_corner))
        for index_and_score in waypoint_scores_in_corners :
            wp_index = index_and_score[0]
            wp_score = index_and_score[1]
            plt.scatter(testwaypoints[wp_index][0], testwaypoints[wp_index][1], color='red', s=wp_score/1.5)
    
    car_location_indices= [1, 3, 30, 51, 52, 58, 100, 120, 140]
    #car_location_indices= [58, 100, 120, 140]
    for i in car_location_indices:
        car_location = testwaypoints[i]
        is_in_corner, corner_points = reward_func_with_speed.get_current_or_next_corner(car_location, testwaypoints, corner_ranges)
        print("Test car @WP {}, in_corner={}, which corner or next={} ({} waypoints)".format(i, is_in_corner, corner_points[0], len(corner_points)))

    plt.show()

test_corner_detection()
#reward_func_with_speed.calculate_reward(testwaypoints[0])