import math
import numpy as np

# Parameters
FUTURE_STEP = 14
MID_STEP = 7
TURN_THRESHOLD = 10     # degrees
DIST_THRESHOLD = 1.2    # metres
SPEED_THRESHOLD = 1.8   # m/s

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

    r = params['track_width'] * 0.9

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
    
    go_fast = select_speed(waypoints, closest_waypoints, FUTURE_STEP, MID_STEP)

    # Implement speed incentive
    if go_fast and speed > SPEED_THRESHOLD:
        reward += 0.1

    elif not go_fast and speed < SPEED_THRESHOLD:
        reward += 0.1
    # Positive reward if the car is in a straight line going fast
    if abs(steering) < 0.1 and speed > SPEED_THRESHOLD:
        reward *= 1.2
        
    return float(score_steer_to_point_ahead(params))+reward
