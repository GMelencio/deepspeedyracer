import math
import numpy as np

# Parameters
MIN_SPEED = 1.2
TURN_LOOKAHEAD_FACTOR = 1.22 #How far to look ahead for turns (relative to track width)

TURN_DETECTION_INTERVAL = 5
MIN_TURN_LENGTH = 3
CORNER_SHARPNESS_THRESHHOLD = 12

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

def get_corners_sharpness(interval_size_to_check, min_count, threshhold, waypoints):
    corner_ranges = []
    waypoints_in_corner = []
    all_waypoints_scores = []

    for j in range(0, len(waypoints)-1):
        change_in_heading, distance = identify_corner(waypoints, [j, j+1], interval_size_to_check)
        all_waypoints_scores.append(change_in_heading)

        #print("WP {}, dH={} @ {}".format(j, change_in_heading, distance))
        if change_in_heading >= threshhold or ((change_in_heading + all_waypoints_scores[-1])/2) > threshhold:
        #print(\"Adding {} to range, score: {}\".format(str(j), str(corner_score)[0:4]))
            waypoints_in_corner.append([j, change_in_heading])
        elif len(waypoints_in_corner) > min_count :
           corner_ranges.append(waypoints_in_corner)
           waypoints_in_corner = []
        else:
            #to reduce "noise", reset collection if less than min_count
           waypoints_in_corner = []

    return corner_ranges, all_waypoints_scores

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


def get_nearby_target_point(car_location, closest_wp_index, waypoints, detection_distance):
    total_num_waypoints = len(waypoints)
    waypoints_starting_with_closest = [waypoints[(i+closest_wp_index) % total_num_waypoints] for i in range(total_num_waypoints)]

    for p in waypoints_starting_with_closest:
        if not dist(p, car_location) < detection_distance :
            return p

    print("WARNING - returned current index of car")
    # this can only happen if we choose r as big as the entire track
    return waypoints[closest_wp_index]


def get_target_steering_degree(car_location, car_heading, target_point):
    tx, ty = target_point
    car_x = car_location[0]
    car_y = car_location[1]
    dx = tx-car_x
    dy = ty-car_y

    _, target_angle = polar(dx, dy)

    steering_angle = target_angle - car_heading

    return angle_mod_360(steering_angle)


def score_steer_to_point_ahead(steering_angle, best_stearing_angle):
    error = (steering_angle - best_stearing_angle) / 60.0  # 60 degree is already really bad
    score = 1.0 - abs(error)

    return max(score, 0.01)  # optimizer is rumored to struggle with negative numbers and numbers too close to zero


def get_steering_score(car_location, car_heading, steering_angle, target_point) :
    best_steering_angle = get_target_steering_degree(car_location, car_heading, target_point)
    steering_score = float(score_steer_to_point_ahead(steering_angle, best_steering_angle))

    return steering_score


def get_closest_waypoint_index(car_location, all_waypoints) :
    distances = [dist(p, car_location) for p in all_waypoints]
    min_dist = min(distances)
    index_closest_waypoint = distances.index(min_dist)
    return index_closest_waypoint


def get_current_or_next_corner(closest_wp_index, all_waypoints, corner_ranges) :
    waypoints_to_next_corner = []

    n = len(all_waypoints)
    waypoints_indices_starting_with_closest = [(i+closest_wp_index) % n for i in range(n)]

    indecies_in_corner = []

    for indicies_with_scores in corner_ranges:
        indecies_in_corner = [k[0] for k in indicies_with_scores]
        if closest_wp_index in indecies_in_corner :
            return True, indicies_with_scores
        #get next closest corner
        else:    
            #count how many waypoints until next corner
            num_to_next_corner = 0
            for wp_index in waypoints_indices_starting_with_closest :
                num_to_next_corner = num_to_next_corner + 1
                if wp_index in indicies_with_scores[0]:
                    break

            waypoints_to_next_corner.append([num_to_next_corner, indicies_with_scores])

    #get the minimum # waypoints among those that were counted
    prev_count, next_corner_waypoints = 0, []
    for waypoint_count_and_points in waypoints_to_next_corner:
        if prev_count == 0 or waypoint_count_and_points[0] < prev_count:
            prev_count = waypoint_count_and_points[0]
            next_corner_waypoints = waypoint_count_and_points[1]
            
    return False, next_corner_waypoints


def get_speed_score_new(car_speed, car_location, closest_wp_index, is_in_corner, distance_from_center, corner_data, all_wp_scores, all_waypoints) :
    speed_reward = 0
    
    if is_in_corner :
        distance_to_exit = dist(all_waypoints[corner_data[-1][0]], car_location)
        divisor = distance_to_exit + all_wp_scores[closest_wp_index]
        speed_reward = car_speed/divisor
    else:
        # the further away from a corner, the faster it should go
        distance_to_next_corner = max(dist(all_waypoints[corner_data[0][0]], car_location), 0.01)
        speed_reward = ((car_speed - MIN_SPEED)/car_speed) * distance_to_next_corner

    #speed_reward = speed_reward + 1/max(distance_from_center, 0.01)

    return speed_reward


def calculate_reward(car_location, car_heading, steering_angle, car_speed, track_width, distance_from_center, is_direction_reversed, all_wheels_on_track, waypoints):
    if is_direction_reversed : # driving clock wise.
        waypoints = list(reversed(waypoints))

    closest_wp_index = get_closest_waypoint_index(car_location, waypoints)
    corner_ranges, all_waypoints_scores = get_corners_sharpness(TURN_DETECTION_INTERVAL, MIN_TURN_LENGTH, CORNER_SHARPNESS_THRESHHOLD, waypoints)
    is_in_corner, corner_data = get_current_or_next_corner(closest_wp_index, waypoints, corner_ranges)

    additional_reward = 0

    target_point = car_location
    if is_in_corner :
        #new: look a little bit further ahead if the corner is not as sharp
        detection_distance = track_width * (TURN_LOOKAHEAD_FACTOR + 1/all_waypoints_scores[closest_wp_index])
        target_point = get_nearby_target_point(car_location, closest_wp_index, waypoints, detection_distance)
        if all_wheels_on_track :
            additional_reward = (car_speed-MIN_SPEED)/(car_speed + max(distance_from_center, 0.01))
    else:
        next_corner_index = corner_data[0][0]
        target_point = waypoints[next_corner_index]
    
    distance_to_target = dist(car_location, target_point)

    speed_score = get_speed_score_new(car_speed, car_location, closest_wp_index, is_in_corner, distance_from_center, corner_data, all_waypoints_scores, waypoints)
    steering_score = get_steering_score(car_location, car_heading, steering_angle, target_point)

    reward_score = speed_score + steering_score + additional_reward
    print("@wp={} Spd={} hdng={} in_crnr={} corner@={} dist={} steer_sc={} spd_sc={} addtl={} REWARD={}".format(closest_wp_index, car_speed, car_heading, is_in_corner, corner_data[0][0], distance_to_target, steering_score, speed_score, additional_reward, reward_score))
    reward_score = speed_score + steering_score
    return reward_score

def reward_function(params):
    waypoints = params['waypoints']
    #closest_waypoints = params['closest_waypoints']
    car_speed = params['speed']
    steering_angle = params['steering_angle']
    car_heading = params['heading']
    car_location = [params['x'], params['y']]
    track_width = params['track_width']
    is_direction_reversed = params['is_reversed']
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    #print("Current Speed {}".format(car_speed))

    reward = calculate_reward(car_location, car_heading, steering_angle, car_speed, track_width, distance_from_center, is_direction_reversed, all_wheels_on_track, waypoints)

    return reward
