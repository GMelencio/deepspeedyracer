import math
import numpy as np

# Parameters
FUTURE_STEP = 16
MID_STEP = 8
TURN_THRESHOLD = 12     # degrees
DIST_THRESHOLD = 1.2    # metres
SPEED_THRESHOLD = 1.9   # m/s

SPEED_INCENTIVE_FACTOR = 0.085 #Additional reward incentive for going faster
TURN_LOOKAHEAD_FACTOR = 1.21 #How far to look ahead for turns (relative to track width)

def identify_corner(waypoints, closest_waypoints, future_step):

    # Identify next waypoint and a further waypoint
    point_prev = waypoints[closest_waypoints[0]]
    point_next = waypoints[closest_waypoints[1]]
    point_future = waypoints[min(len(waypoints) - 1, 
                                 closest_waypoints[1] + future_step)]

    # Calculate headings to waypoints
    heading_current = math.degrees(math.atan2(point_prev[1] - point_next[1], 
                                             point_prev[0] - point_next[0]))
    heading_future = math.degrees(math.atan2(point_prev[1] - point_future[1], 
                                             point_prev[0]-point_future[0]))

    # Calculate the difference between the headings
    diff_heading = abs(heading_current - heading_future)

    # Check we didn't choose the reflex angle
    if diff_heading > 180:
        diff_heading = 360 - diff_heading

    # Calculate distance to further waypoint
    dist_future = np.linalg.norm([point_next[0] - point_future[0],
                                  point_next[1] - point_future[1]])  

    return diff_heading, dist_future


def total_angle_of_curve(waypoints, first_point_on_curve, track_length_of_curve):
    #NOTE: this expects that waypoints are ordered by distance from car (NOT linear distance as the car may be on a hairpin turn)

    distances = [dist(p, first_point_on_curve) for p in waypoints]
    min_dist = min(distances)
    i_closest = distances.index(min_dist)
    
    #my n00b way of keeping track of variables cause I have to re-learn python after many years
    i = i_closest
    track_distance_traversed = 0
    total_angle = 0
    
    #Iterate through waypoints, starting from i_closest, keep going until = track_distance_between
    while track_distance_traversed < track_length_of_curve:
        waypoint_current = waypoints[i]
        waypoint_next = waypoints[i+1]
        track_distance_traversed += dist(waypoint_current, waypoint_next)
        opposite = waypoint_next[1] - waypoint_current[1]
        adjacent = waypoint_next[0] - waypoint_current[0]
        angle = math.atan2(opposite, adjacent) #output to log to check value
        total_angle += angle

    return total_angle

#TODO: replace this with a function that takes into account how far is a sharp curve is ahead
def select_speed(waypoints, closest_waypoints, future_step, mid_step):

    # Identify if a corner is in the future
    diff_heading, dist_future = identify_corner(waypoints, 
                                                    closest_waypoints, 
                                                    future_step)

    if diff_heading < TURN_THRESHOLD:
        # If there's no corner encourage going faster
        go_fast = True
    else:
        if dist_future < DIST_THRESHOLD:
            # If there is a corner and it's close encourage going slower
            go_fast = False
        else:
            # If the corner is far away, re-assess closer points
            diff_heading_mid, dist_mid = identify_corner(waypoints, 
                                                         closest_waypoints, 
                                                         mid_step)

            if diff_heading_mid < TURN_THRESHOLD:
                # If there's no corner encourage going faster
                go_fast = True
            else:
                # If there is a corner and it's close encourage going slower
                go_fast = False

    return go_fast

def dist(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
    
def rect(r, theta):
    """
    theta in degrees
    returns tuple; (float, float); (x,y)
    """

    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x, y


def polar(x, y):
    """
    returns r, theta(degrees)
    """

    r = (x ** 2 + y ** 2) ** .5
    theta = math.degrees(math.atan2(y,x))
    return r, theta


def angle_mod_360(angle):
    """
    Maps an angle to the interval -180, +180.
    Examples:
    angle_mod_360(362) == 2
    angle_mod_360(270) == -90
    :param angle: angle in degree
    :return: angle in degree. Between -180 and +180
    """

    n = math.floor(angle/360.0)

    angle_between_0_and_360 = angle - n*360.0

    if angle_between_0_and_360 <= 180.0:
        return angle_between_0_and_360
    else:
        return angle_between_0_and_360 - 360


def get_waypoints_ordered_in_driving_direction(params):
    # waypoints are always provided in counter clock wise order
    if params['is_reversed']: # driving clock wise.
        return list(reversed(params['waypoints']))
    else: # driving counter clock wise.
        return params['waypoints']


def up_sample(waypoints, factor):
    """
    Adds extra waypoints in between provided waypoints
    :param waypoints:
    :param factor: integer. E.g. 3 means that the resulting list has 3 times as many points.
    :return:
    """
    p = waypoints
    n = len(p)

    return [[i / factor * p[(j+1) % n][0] + (1 - i / factor) * p[j][0],
             i / factor * p[(j+1) % n][1] + (1 - i / factor) * p[j][1]] for j in range(n) for i in range(factor)]


def get_target_point(params):
    waypoints = up_sample(get_waypoints_ordered_in_driving_direction(params), 20)

    car = [params['x'], params['y']]

    distances = [dist(p, car) for p in waypoints]
    min_dist = min(distances)
    i_closest = distances.index(min_dist)

    n = len(waypoints)

    waypoints_starting_with_closest = [waypoints[(i+i_closest) % n] for i in range(n)]

    r = params['track_width'] * TURN_LOOKAHEAD_FACTOR

    is_inside = [dist(p, car) < r for p in waypoints_starting_with_closest]
    i_first_outside = is_inside.index(False)

    if i_first_outside < 0:  # this can only happen if we choose r as big as the entire track
        return waypoints[i_closest]

    return waypoints_starting_with_closest[i_first_outside]


def get_target_steering_degree(params):
    tx, ty = get_target_point(params)
    car_x = params['x']
    car_y = params['y']
    dx = tx-car_x
    dy = ty-car_y
    heading = params['heading']

    _, target_angle = polar(dx, dy)

    steering_angle = target_angle - heading

    return angle_mod_360(steering_angle)


def score_steer_to_point_ahead(params):
    best_stearing_angle = get_target_steering_degree(params)
    steering_angle = params['steering_angle']

    error = (steering_angle - best_stearing_angle) / 60.0  # 60 degree is already really bad

    score = 1.0 - abs(error)

    return max(score, 0.01)  # optimizer is rumored to struggle with negative numbers and numbers too close to zero


def reward_function(params):
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    speed = params['speed']
    reward = 0
    
    #TODO: Improve on this by instead of returning a boolean value, return a float that defines how far off and how sharp the next the corner is
    go_fast = select_speed(waypoints, closest_waypoints, FUTURE_STEP, MID_STEP)

    # Implement speed incentive
    if go_fast and speed > SPEED_THRESHOLD:
        additional_reward_for_going_faster = (speed - SPEED_THRESHOLD)* SPEED_INCENTIVE_FACTOR
        reward = reward + 0.1 + additional_reward_for_going_faster

    #TODO: adjust reward for slowing down more the sharper and closer the next curve is
    elif not go_fast and speed < SPEED_THRESHOLD:
        reward += 0.1
 
        
    return float(score_steer_to_point_ahead(params))+reward

def get_corners_and_sharpness_scores(interval_min, interval_max, waypoints, threshhold):
    corner_ranges = []
    waypoints_and_scores = []
    for j in range(0, len(waypoints)-1):
        debug_output = "C@ " + str(j)
        corner_score = 0
        for steps in range(interval_min,interval_max):
            corner_check = identify_corner(waypoints, [j, j+1], steps)
            corner_factor = (corner_check[0]/corner_check[1])/steps
            corner_score = corner_score + corner_factor
            
        if corner_score >= threshhold :           
            if len(corner_ranges) > 1 and j - corner_ranges[-1][-1][0] >= interval_min :
                #previous range existed and is relatively close, use that one moving forward
                waypoints_and_scores = corner_ranges[-1]
                print("reconciled previous at {}".format(corner_ranges[-1][-1][0]))
                
            print("Adding {} to range, score: {}".format(str(j), str(corner_factor)[0:4]))
            waypoints_and_scores.append([j, corner_score])
        elif len(waypoints_and_scores) > 2 :
           print("creating new range from {} to {}".format(waypoints_and_scores[0][0], j))
           corner_ranges.append(waypoints_and_scores)
           waypoints_and_scores = []
        #debug_output = debug_output + " " + str(corner_factor)[0:4] + ","
        #print(debug_output + " = " + str(corner_score)[0:4])
    return corner_ranges

import matplotlib.pyplot as plt
import numpy as np

for waypoint in testwaypoints:
    plt.scatter(waypoint[0], waypoint[1], color='k')

corner_ranges = get_corners_and_sharpness_scores(3,12,testwaypoints, 17)
for waypoint_scores in corner_ranges:
    for k in waypoint_scores:
        plt.scatter(testwaypoints[k[0]][0], testwaypoints[k[0]][1], color='r', s=k[1]/1.5) 

plt.show()