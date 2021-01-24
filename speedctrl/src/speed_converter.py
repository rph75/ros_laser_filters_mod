import math

#lookups for turn in place
#0. path speed (m/sec)
#1. angular speed (rad/sec)
#2. controller speed (m/sec)
#3. controller speed delta left/right (m/sec)
tip_lookups = [
(0,0,0,0),
(0,0.1637,0,0.05),
(0,-0.1952,0,-0.05),
(0,0.2504,0,0.075),
(0,-0.2964,0,-0.075),
]

#Tuples of
#0. path speed (m/sec)
#1. angular speed (rad/sec)
#2. controller speed (m/sec)
#3. controller speed delta left/right (m/sec)
lookups = [
(0.1,0,0.1,0),
(-0.1005,0,-0.1,0),
(0.2,0,0.2,0),
(-0.1988,0,-0.2,0),
(0.2973,0,0.3,0),
(-0.299,0,-0.3,0),
(0,0.1637,0,0.05),
(0,-0.1952,0,-0.05),
(0,0.2504,0,0.075),
(0,-0.2964,0,-0.075),
(0.0997,0.0338,0.1,0.01),
(-0.0988,-0.0277,-0.1,-0.01),
(0.097,0.0712,0.1,0.02),
(-0.0969,-0.0712,-0.1,-0.02),
(0.095,0.1011,0.1,0.03),
(-0.0961,-0.1102,-0.1,-0.03),
(0.0925,0.1388,0.1,0.04),
(-0.095,-0.1536,-0.1,-0.04),
(0.0907,0.1746,0.1,0.05),
(-0.0922,-0.1864,-0.1,-0.05),
(0.0878,0.2143,0.1,0.06),
(-0.0904,-0.2191,-0.1,-0.06),
(0.0859,0.2509,0.1,0.07),
(-0.0899,-0.2552,-0.1,-0.07),
(0.0832,0.2962,0.1,0.08),
(-0.0878,-0.2904,-0.1,-0.08),
(0.0758,0.3255,0.1,0.09),
(-0.0485,-0.1451,-0.1,-0.09),
(0.0181,0.0158,0.1,0.1),
(-0.0349,-0.0292,-0.1,-0.1),
(0.0998,0.0025,0.1,0.002),
(-0.1003,-0.0014,-0.1,-0.002),
(0.1001,0.0101,0.1,0.004),
(-0.0995,-0.0068,-0.1,-0.004),
(0.1001,0.0179,0.1,0.006),
(-0.0998,-0.0132,-0.1,-0.006),
(0.0991,0.0252,0.1,0.008),
(-0.0988,-0.0211,-0.1,-0.008),
(-0.1996,-0.0014,-0.2,-0.004),
(0.1999,0.021,0.2,0.008),
(-0.1976,-0.0115,-0.2,-0.008),
(0.2,0.0358,0.2,0.012),
(-0.1977,-0.0259,-0.2,-0.012),
(0.1987,0.0523,0.2,0.016),
(-0.1977,-0.0411,-0.2,-0.016),
(0.1998,0.0689,0.2,0.02),
(-0.1976,-0.0577,-0.2,-0.02),
(0.1971,0.1474,0.2,0.04),
(-0.1942,-0.1385,-0.2,-0.04),
(0.1903,0.2193,0.2,0.06),
(-0.1869,-0.1972,-0.2,-0.06),
(0.1813,0.2726,0.2,0.08),
(-0.1753,-0.2352,-0.2,-0.08),
(0.2521,0.0003,0.25,0.004),
(0.2512,0.0171,0.25,0.008),
(-0.2504,-0.0088,-0.25,-0.008),
(0.2503,0.0324,0.25,0.012),
(-0.2487,-0.0204,-0.25,-0.012),
(0.2496,0.0485,0.25,0.016),
(-0.2474,-0.0349,-0.25,-0.016),
(0.2485,0.0639,0.25,0.02),
(-0.2466,-0.0511,-0.25,-0.02),
(0.2445,0.1344,0.25,0.04),
(-0.2436,-0.1221,-0.25,-0.04),
(0.2324,0.1775,0.25,0.06),
(-0.228,-0.146,-0.25,-0.06),
(0.2187,0.2054,0.25,0.08),
(-0.2129,-0.1693,-0.25,-0.08),
(0.0997,-0.0338,0.1,-0.01),
(-0.0988,0.0277,-0.1,0.01),
(0.097,-0.0712,0.1,-0.02),
(-0.0969,0.0712,-0.1,0.02),
(0.095,-0.1011,0.1,-0.03),
(-0.0961,0.1102,-0.1,0.03),
(0.0925,-0.1388,0.1,-0.04),
(-0.095,0.1536,-0.1,0.04),
(0.0907,-0.1746,0.1,-0.05),
(-0.0922,0.1864,-0.1,0.05),
(0.0878,-0.2143,0.1,-0.06),
(-0.0904,0.2191,-0.1,0.06),
(0.0859,-0.2509,0.1,-0.07),
(-0.0899,0.2552,-0.1,0.07),
(0.0832,-0.2962,0.1,-0.08),
(-0.0878,0.2904,-0.1,0.08),
(0.0758,-0.3255,0.1,-0.09),
(-0.0485,0.1451,-0.1,0.09),
(0.0181,-0.0158,0.1,-0.1),
(-0.0349,0.0292,-0.1,0.1),
(0.0998,-0.0025,0.1,-0.002),
(-0.1003,0.0014,-0.1,0.002),
(0.1001,-0.0101,0.1,-0.004),
(-0.0995,0.0068,-0.1,0.004),
(0.1001,-0.0179,0.1,-0.006),
(-0.0998,0.0132,-0.1,0.006),
(0.0991,-0.0252,0.1,-0.008),
(-0.0988,0.0211,-0.1,0.008),
(-0.1996,0.0014,-0.2,0.004),
(0.1999,-0.021,0.2,-0.008),
(-0.1976,0.0115,-0.2,0.008),
(0.2,-0.0358,0.2,-0.012),
(-0.1977,0.0259,-0.2,0.012),
(0.1987,-0.0523,0.2,-0.016),
(-0.1977,0.0411,-0.2,0.016),
(0.1998,-0.0689,0.2,-0.02),
(-0.1976,0.0577,-0.2,0.02),
(0.1971,-0.1474,0.2,-0.04),
(-0.1942,0.1385,-0.2,0.04),
(0.1903,-0.2193,0.2,-0.06),
(-0.1869,0.1972,-0.2,0.06),
(0.1813,-0.2726,0.2,-0.08),
(-0.1753,0.2352,-0.2,0.08),
(0.2521,-0.0003,0.25,-0.004),
(0.2512,-0.0171,0.25,-0.008),
(-0.2504,0.0088,-0.25,0.008),
(0.2503,-0.0324,0.25,-0.012),
(-0.2487,0.0204,-0.25,0.012),
(0.2496,-0.0485,0.25,-0.016),
(-0.2474,0.0349,-0.25,0.016),
(0.2485,-0.0639,0.25,-0.02),
(-0.2466,0.0511,-0.25,0.02),
(0.2445,-0.1344,0.25,-0.04),
(-0.2436,0.1221,-0.25,0.04),
(0.2324,-0.1775,0.25,-0.06),
(-0.228,0.146,-0.25,0.06),
(0.2187,-0.2054,0.25,-0.08),
(-0.2129,0.1693,-0.25,0.08),

]

def dist_target(path_speed1,ang_speed1,path_speed2,ang_speed2):
    #TODO: we could give more weight to the path speed or the angular speed
    d = math.sqrt((path_speed1-path_speed2)**2+(ang_speed1-ang_speed2)**2)
    return d

#Special handling for turn-in-place
def turn_in_place(ang_speed):
    p = [None,None]
    for lookup in tip_lookups:
        # Tuples of
        # 0. path speed (m/sec)
        # 1. angular speed (rad/sec)
        # 2. controller speed (m/sec)
        # 3. controller speed delta left/right (m/sec)
        if lookup[1]<=ang_speed:
            if p[0] is None or lookup[1]>p[0][1]:
                p[0] = lookup
        if lookup[1]>ang_speed:
            if p[1] is None or lookup[1]<p[1][1]:
                p[1] = lookup
    controller_speed = 0 #Always 0 speed
    if p[0] is None:
        #extrapolation on lower side
        controller_delta = ang_speed/p[1][1]*p[1][3]
    elif p[1] is None:
        #extrap0liation on upper side
        controller_delta = ang_speed / p[0][1] * p[0][3]
    else:
        #interpolate
        portion = (ang_speed - p[0][1])/(p[1][1]-p[0][1])
        controller_delta=p[0][3]+portion*(p[1][3]-p[0][3])
    return controller_speed, controller_delta


# Converts a path_speed (in m/sec) and an angular speed (in rad/sec)
# To a controller speed (in m/sec) and a controller speed delta (in m/sec)
def convert_to_controller(path_speed, ang_speed):
    if abs(path_speed<0.0001):
        return turn_in_place(ang_speed)
    # Now look up the 4 points
    #
    # ^ ang speed
    # |
    # |  P3      P2
    # |       T
    # |  P0      P1
    # |
    # +---------------> path speed
    #
    # P0: tuple with max path speed and max ang speed, where path speed <= target and ang speed <= target
    # P1: tuple with min path speed and max ang speed, where path speed > target and ang speed <= target
    # P2: tuple with min path speed and min ang speed, where path speed > target and ang speed > target
    # P3: tuple with max path speed and min ang speed, where path speed <= target and ang speed > target
    p = [None,None,None,None]
    dist =[0,0,0,0]
    for lookup in lookups:
        # Tuples of
        # 0. path speed (m/sec)
        # 1. angular speed (rad/sec)
        # 2. controller speed (m/sec)
        # 3. controller speed delta left/right (m/sec)
        d = dist_target(path_speed, ang_speed, lookup[0], lookup[1])
        if lookup[0]<=path_speed and lookup[1]<=ang_speed:
            #lookup is a candidate for P0
            if p[0] is None or d<dist_target(p[0][0],p[0][1],lookup[0],lookup[1]):
                p[0]=lookup
                dist[0]=d
        if lookup[0]>path_speed and lookup[1]<=ang_speed:
            #lookup is a candidate for P1
            if p[1] is None or d<dist_target(p[1][0],p[1][1],lookup[0],lookup[1]):
                p[1]=lookup
                dist[1] = d
        if lookup[0]>path_speed and lookup[1]>ang_speed:
            #lookup is a candidate for P2
            if p[2] is None or d<dist_target(p[2][0],p[2][1],lookup[0],lookup[1]):
                p[2]=lookup
                dist[2]=d
        if lookup[0]<=path_speed and lookup[1]>ang_speed:
            #lookup is a candidate for P3
            if p[3] is None or d<dist_target(p[3][0],p[3][1],lookup[0],lookup[1]):
                p[3]=lookup
                dist[3]=d

    total_distance = 0
    for i in range(0,len(p)):
        x = p[i]
        if x is not None:
            total_distance = total_distance + dist[i]

    controller_speed = 0
    controller_delta = 0
    total_weight = 0
    for i in range(0,len(p)):
        x = p[i]
        if x is not None:
            d = dist[i]
            if d<0.0000001:
                #Avoid div 0
                d=0.0000001
            weight = total_distance / d
            total_weight = total_weight + weight
            controller_speed = controller_speed + weight * x[2]
            controller_delta = controller_delta + weight * x[3]
    controller_speed = controller_speed / total_weight
    controller_delta = controller_delta / total_weight
    return controller_speed, controller_delta

controller_speed, controller_delta = convert_to_controller(0, 0.2)
print("controller speed " + str(controller_speed))
print("controller delta " + str(controller_delta))
