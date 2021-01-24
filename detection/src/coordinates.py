import math
from geometry_msgs.msg import Point

def distance(x1,y1,x2,y2):
	return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# Computes the y in pixels given a x in pixels given a line definition
def calc_y_pix(line_def,x_pix):
    slope=line_def[0]
    const=line_def[1]
    y_pix=const+slope*x_pix
    return (int)(y_pix)


# Computes the x in pixels given a y in pixels given a line definition
def calc_x_pix(line_def,y_pix):
    slope=line_def[0]
    const=line_def[1]
    x_pix=(y_pix-const)/slope
    return (int)(x_pix)


# Converts a coordinate in pixels into a coordinate in cm
# To do so, the vertical and horizontal lines adjacent to the point is evaluted
# Then, the cooridinates are interpolated
# If the point is outsie the range, the method returns None
# This is for near camera
def near_pixel_to_cm(x_pix, y_pix):
    y_cm_above=None
    y_cm_below=None
    x_cm_left=None
    x_cm_right=None

    for x_cm, line_def in near_lines_vertical.items():
        x_pix_line = calc_x_pix(line_def,y_pix)
        if x_pix_line >= x_pix:
            x_pix_right=x_pix_line
            x_cm_right=x_cm
            break
        else:
            x_pix_left=x_pix_line
            x_cm_left=x_cm

    for y_cm, line_def in near_lines_horizontal.items():
        y_pix_line = calc_y_pix(line_def, x_pix)
        if y_pix_line <= y_pix:
            y_pix_above = y_pix_line
            y_cm_above = y_cm
            break
        else:
            y_pix_below = y_pix_line
            y_cm_below = y_cm

    if y_cm_above is None or y_cm_below is None or x_cm_left is None or x_cm_right is None:
        # Cannot interpolate
        return None,None

    y_cm = (y_pix_below-y_pix)/(y_pix_below-y_pix_above)*(y_cm_above-y_cm_below)+y_cm_below
    x_cm = (x_pix-x_pix_left)/(x_pix_right-x_pix_left)*(x_cm_right-x_cm_left)+x_cm_left
    return x_cm, y_cm

# Converts a coordinate in pixels into a coordinate in cm
# To do so, the vertical and horizontal lines adjacent to the point is evaluted
# Then, the cooridinates are interpolated
# If the point is outsie the range, the method returns None
# This is for far camera
def far_pixel_to_cm(x_pix, y_pix):
    x_cm_left = None
    x_cm_right = None

    # y_cm is computed according to formula: y_pix = c0 + (c1 / (y_cm - c2))
    # where c0 is the y coordinate of the 'fluchtpunkt' and c1 / c2 were found to match what was measured
    y_cm = c1 / (y_pix - c0) + c2

    # Find adjacent lines for x:
    for x_cm, line_def in far_lines_vertical.items():
        x_pix_line = calc_x_pix(line_def,y_pix)
        if x_pix_line >= x_pix:
            x_pix_right = x_pix_line
            x_cm_right = x_cm
            break
        else:
            x_pix_left = x_pix_line
            x_cm_left = x_cm

    if x_cm_left is None or x_cm_right is None:
        # Cannot interpolate
        return None,y_cm

    x_cm = (x_pix-x_pix_left)/(x_pix_right-x_pix_left)*(x_cm_right-x_cm_left)+x_cm_left
    return x_cm, y_cm

# Computes the forward/backward difference for left and right wheels in cm to get the gripper to the desired position
def move_gripper_to_pos(x_cm, y_cm):
    x, y = to_turning_coordinates(x_cm, y_cm)
    x_gr, y_gr = to_turning_coordinates(gripper_x_cm, gripper_y_cm)
    forward_cm = distance(x, y, 0, 0) - distance(x_gr, y_gr, 0, 0)
    alpha_rad = math.atan2(x, y) - math.atan2(x_gr, y_gr) #in rad
    return move_robot(forward_cm, alpha_rad)


def move_robot(forward_cm,alpha_rad):
    # TODO: The angle_scaling might have to be less if the turn is smooth, i.e. when there is less friction
    turn_cm = turn_r_cm * alpha_rad * angle_scaling
    fwd_left_cm = forward_cm + turn_cm
    fwd_right_cm = forward_cm - turn_cm
    print("Moving forward {0:.1f} cm, turning right {1:.1f} deg".format(forward_cm, alpha_rad / math.pi * 180))
    return fwd_left_cm, fwd_right_cm


# Converts coordinates to coordinates relative to turning point
def to_turning_coordinates(x_cm,y_cm):
    return x_cm-turn_x_cm, y_cm-turn_y_cm

#Converts the coordinates to meters, from base link
#Also, the x/y are adjusted to ROS standard (x(ros)=y(detector), y(ros)= -x(detector)
def to_base_link(x_cm,y_cm):
    xt,yt = to_turning_coordinates(x_cm,y_cm)
    return yt/100.0,-xt/100

def to_base_link_point(x_cm,y_cm):
    x,y = to_base_link(x_cm,y_cm)
    return Point(x=x,y=y,z=0.0)

'''
#Adjusts the right side of the cube boundary to the height of the object using the camera angle
#x_cm: The x in cm that the camera sees
#h_cm: The height of the object (brick)
#returns the x_cm of the object as it is on the ground
def adjust_for_height(x_cm,h_cm):
    return x_cm*(cam_cm-h_cm)/cam_cm


#x_pix_min,y_pix_min: Upper left corner
#x_pix_max,y_pix_max: Lower right corner
#return the center of the brick in x_cm / y_cm
def find_brick_center_cm(x_pix_min,y_pix_min,x_pix_max,y_pix_max,h_cm):
    top_left_x_cm,top_left_y_cm=pixel_to_cm(x_pix_min,y_pix_min)
    bottom_left_x_cm,bottom_left_y_cm=pixel_to_cm(x_pix_min,y_pix_max)
    top_right_x_cm,top_right_y_cm=pixel_to_cm(x_pix_max,y_pix_min)
    bottom_right_x_cm,bottom_right_y_cm=pixel_to_cm(x_pix_max,y_pix_max)
    if top_left_x_cm is None:
        return None,None
    if bottom_left_x_cm is None:
        return None,None
    if top_right_x_cm is None:
        return None,None
    if bottom_right_x_cm is None:
        return None,None

    top_right_x_cm=adjust_for_height(top_right_x_cm,h_cm)
    bottom_right_x_cm=adjust_for_height(bottom_right_x_cm,h_cm)
    mid_left_cm=(top_left_x_cm+bottom_left_x_cm)/2
    mid_right_cm=(top_right_x_cm+bottom_right_x_cm)/2
    mid_x_cm=(mid_left_cm+mid_right_cm)/2

    mid_top_cm=(top_left_y_cm+top_right_y_cm)/2
    mid_bottom_cm=(bottom_left_y_cm+bottom_right_y_cm)/2
    mid_y_cm=(mid_top_cm+mid_bottom_cm)/2

    return mid_x_cm,mid_y_cm

#Height of the camera above ground
cam_cm = 14
'''

# Line data is obtained from coordinates.xls using linear regression on points on line. Assumption is that all lines are straight
# Maps from a X in cm to slope and const of the line defining that X: y (pixel) = const + slope * x(pixel)
near_lines_vertical={
    10: (-4.29957116602001, 524.843373493976),
    12: (-5.16057482036863, 1091.95470165573),
    14: (-6.2341935483871, 1832.29838709677),
    16: (-7.03805686142825, 2617.01119319454),
    18: (-7.82212581344902, 3441.75054229935),
    20: (-9.89784649364992, 4904.28492545555),
    22: (-11.0075585789872, 6051.39833711262),
    24: (-12.2910994764398, 7372.77172774869),
    26: (-14.3354430379747, 9168.70569620253),
    28: (-19.1481481481481, 12863.2222222222),
}

# Maps from a Y in cm to slope and const of the line defining that Y: y (pixel) = const + slope * x(pixel)
near_lines_horizontal={
    2: (0.165644171779141, 528.253987730061),
    4: (0.254012077659476, 382.665781503016),
    6: (0.34153324769924, 243.121140560641),
    8: (0.428020939474298, 104.639050412644),
    10: (0.507802769192242, -24.9849658299339),
    12: (0.604625954999213, -161.09216070211),
    14: (0.711484076342604, -298.06273774165),
    16: (0.804097853350272, -422.930225740252),
}

# Line data is obtained from coordinates.xls using linear regression on points on line. Assumption is that all lines are straight
# Maps from a X in cm to slope and const of the line defining that X: y (pixel) = const + slope * x(pixel)
far_lines_vertical={
    -34.5: (-0.282119609, 134.2247897),
    -9.5: (-0.645737569, 288.855908),
    15.5: (3.801401501, -1630.09601),
    40.5: (0.472210066, -190.3908096),
    65.5: (0.28, -116.24),
}

# far y_cm is computed according to formula: y_pix = c0 + (c1 / (y_cm - c2))
# where c0 is the y coordinate of the 'fluchtpunkt' (looked up on image) and c1 / c2 were found to match the measured coordinates of the lines
c0 = 12
c1 = 11704
c2 = -2.67

turn_r_cm = 18  # turning circle radius in cm
turn_x_cm = 14.5 # x coordinate of turning point in cm
turn_y_cm = -15.0 # y coordinate of turning point in cm
angle_scaling = 1.33 # Scale up factor for turning angle to compensate for friction of wheels (empirical value)
gripper_x_cm = 19 - 1.6 # x position of the gripper in cm (adjusted to lower left corner of brick)
gripper_y_cm = 5.5 - 1.7  # y position of the gripper in cm (adjusted to lower left corner of brick)

# x coordinate in cm