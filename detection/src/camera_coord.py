import cv2
from geometry_msgs.msg import Point

#Area where we can trust a detection so we could use the gripper
STABLE_POLY = [Point(x=0.01, y=0.01), Point(x=0.01, y=-0.14), Point(x=0.05, y=-0.17), Point(x=0.18, y=-0.18),Point(x=0.12, y=-0.06), Point(x=2.0, y=-1.0), Point(x=2.0, y=1.25), Point(x=0.12, y=0.09),Point(x=0.08, y=0.02)]

# Line data is obtained from measures.ods. Assumption is that all lines are straight
# tuples are (k1,k2), where y_pix = k1 + k2 * x_pix
# horizontal lines are mapping from the x in meters (camera coordinate frame) to the tuples (k1,k2)
# vertical lines are mapping from the y in meters (camera coordinate frame) to the tuples (k1,k2)
NEAR_LINES_HORIZONTAL={
    0: (577.93493, 0.03253),
    0.02: (463.43946, 0.11211),
    0.04: (356.29582, 0.17605),
    0.06: (250.85528, 0.24381),
    0.08: (147.02941, 0.30882),
    0.1: (48.66803, 0.37022),
    0.12: (-43.97017, 0.427),
    0.14: (-137.25249, 0.48708),
    0.16: (-231, 0.54545),
    0.18: (-312.83333, 0.59524),
    0.2: (-387.42353, 0.63529),
    0.22: (-464.74766, 0.68224),
    0.24: (-568.93617, 0.76596),
}

NEAR_VERTICAL_LINES={
    0.04: (-39.17576, -2.79394),
    0.02: (303.67416, -3.33708),
    0: (769.01379, -3.95862),
    -0.02: (1290.09322, -4.5339),
    -0.04: (1908.875, -5.3125),
    -0.06: (2682.18072, -6.26506),
    -0.08: (3512.94521, -7.23288),
    -0.1: (4536.03175, -8.4127),
    -0.12: (5982.22642, -10.22642),
    -0.14: (7930.69767, -12.62791),
    -0.16: (11814.4, -17.8),
    -0.18: (15018.34783, -21.43478),
    -0.2: (20746.66667, -28.33333),
    -0.22: (32017, -42),
    -0.24: (122528.66667, -155.66667),
}

FAR_FP_X = 425
FAR_FP_Y = 125
# Constants to find distance in meters, given y pixels, found using wolfram-alpha on formula
# x_m = c1 / (y_pix - c0) + c2
FAR_C1 = 68.839
FAR_C2 = -0.034146

FAR_SCALE_0 = 2843.75 # Pixels per meter at 0.2m distance
FAR_Y0 = 420 # Y coordinate (pixels) of 0.2m distance line

class CameraCoordinates:
    def __init__(self):
        return

    # Computes the y in pixels given a x in pixels given a line definition:
    # y=k1+k2*x
    def calc_y_pix(self,line_def, x_pix):
        k1 = line_def[0]
        k2 = line_def[1]
        y_pix = k1 + k2 * x_pix
        return (int)(y_pix)

    # Computes the x in pixels given a y in pixels given a line definition
    # y=k1+k2*x
    def calc_x_pix(self,line_def, y_pix):
        k1 = line_def[0]
        k2 = line_def[1]
        x_pix = (y_pix - k1) / k2
        return (int)(x_pix)

    # This is for near camera
    # Converts a coordinate in pixels into a coordinate in meters (in camera coordiante frame)
    # To do so, the vertical and horizontal lines adjacent to the point is evaluted
    # Then, the coordinates are inter/extrapolated by inter/extrapolating the k1 and k2 in y=k1 * k2*x
    def near_pixel_to_m(self,x_pix, y_pix):
        line_above = None
        x_above = None
        for x, line_def in NEAR_LINES_HORIZONTAL.items():
            x_below = x_above
            x_above = x
            line_below = line_above
            line_above = line_def
            if line_below is not None and self.calc_y_pix(line_above, x_pix) < y_pix:
                break

        line_right = None
        y_right = None
        for y, line_def in NEAR_VERTICAL_LINES.items():
            y_left = y_right
            y_right = y
            line_left = line_right
            line_right = line_def
            if line_left is not None and self.calc_x_pix(line_right, y_pix) > x_pix:
                break

        s1 = (y_pix - x_pix*line_below[1] - line_below[0]) / (x_pix*(line_above[1]-line_below[1]) + (line_above[0] - line_below[0]))
        k1h=line_below[0]+(line_above[0]-line_below[0])*s1
        k2h=line_below[1]+(line_above[1]-line_below[1])*s1

        s2 = (x_pix * (k2h - line_left[1]) -line_left[0] +k1h ) / ((line_right[0]- line_left[0]) + x_pix * ( line_right[1] - line_left[1]) )
        x_m = s1 * (x_above - x_below) + x_below
        y_m = s2 * (y_right - y_left) + y_left
        return x_m, y_m

    # Convert from meters to pixel
    def near_m_to_pixel(self,x_m, y_m):
        line_above = None
        x_above = None
        for x, line_def in NEAR_LINES_HORIZONTAL.items():
            x_below = x_above
            x_above = x
            line_below = line_above
            line_above = line_def
            if line_below is not None and x >= x_m:
                break

        line_right = None
        y_right = None
        for y, line_def in NEAR_VERTICAL_LINES.items():
            y_left = y_right
            y_right = y
            line_left = line_right
            line_right = line_def
            if line_left is not None and y < y_m:
                break

        s1=(x_m-x_below)/(x_above-x_below)
        s2=(y_m-y_left)/(y_right-y_left)
        k1h=line_below[0]+(line_above[0]-line_below[0])*s1
        k2h=line_below[1]+(line_above[1]-line_below[1])*s1

        k1v=line_left[0]+(line_right[0]-line_left[0])*s2
        k2v=line_left[1]+(line_right[1]-line_left[1])*s2

        x_pix=(int)((k1v-k1h)/(k2h-k2v))
        y_pix=(int)(k1h+k2h*x_pix)
        return x_pix,y_pix


    # This is for far camera
    # Converts a coordinate in pixels into a coordinate in meters (in camera coordiante frame)
    # The distance is found using the approximation y_m = c1 / (y_pix - c0) + c2, where the constants
    # have been found solving three distances at 0.2m, 1.0m and 4.0m
    def far_pixel_to_m(self,x_pix, y_pix):
        if y_pix <= FAR_FP_Y:
            #Avoid div zero in area above fluchtpunkt (anyway illegal)
            x_m=float('inf')
            y_m=0.0
        else:
            # x_m is computed according to formula: y_pix = c0 + (c1 / (x_m - c2))
            # where c0 is the y pix coordinate of the 'fluchtpunkt' and c1 / c2 were found to match what was measured
            x_m = FAR_C1 / (y_pix - FAR_FP_Y) + FAR_C2

            # y_m is computed using triangles, and a given number of pixels per meter at a distance of 0.2m:
            # y_m=((x_fp-x)*(y_0-y_fp)/(y-y_fp))/scale
            # where y_0 is y in pixels of 0.2m distance
            # and scale is pixels per meter at 0.2m distance
            y_m = ((FAR_FP_X-x_pix)*(FAR_Y0-FAR_FP_Y)/(y_pix-FAR_FP_Y))/FAR_SCALE_0
        return x_m, y_m

    #Convert from meters to pixel
    def far_m_to_pixel(self,x_m, y_m):
        y_pix = (int) ((FAR_C1 - FAR_C2 * FAR_FP_Y + FAR_FP_Y * x_m) / (x_m - FAR_C2))
        x_pix = (int) (FAR_FP_X + FAR_SCALE_0 * y_m * (y_pix - FAR_FP_Y) / (FAR_FP_Y - FAR_Y0))
        return x_pix,y_pix




    def test(self):
        #Test code

        x_m,y_m = self.far_pixel_to_m(200,	600)
        print(self.far_m_to_pixel(x_m,y_m))

        #x_m,y_m = self.near_pixel_to_m(600,	170)



        x_pix,y_pix = self.near_m_to_pixel(0.055,0.075)
        x_m, y_m = self.near_pixel_to_m(x_pix, y_pix)
        print(x_m,y_m)
        print(self.near_pixel_to_m(183,254))
        print(self.near_pixel_to_m(600,170))
        print(self.near_pixel_to_m(7,99))

        print(self.far_pixel_to_m(180,	10))

        print(self.far_pixel_to_m(180,	281))
        print(self.far_pixel_to_m(19,	187))
        print(self.far_pixel_to_m(315,	143))
        print(self.far_pixel_to_m(765,	237))  #0.6 / -0.32

        blue = (250,0,0)
        yellow = (0,255,255)
        image_near=cv2.imread("/home/roman/measure/near.jpg")
        for y_m, line_def in NEAR_VERTICAL_LINES.items():
            y1=0
            y2=600
            x1=self.calc_x_pix(line_def,y1)
            x2=self.calc_x_pix(line_def,y2)
            cv2.line(image_near, (x1,y1), (x2,y2), blue, 1)

        for y_m, line_def in NEAR_LINES_HORIZONTAL.items():
            x1=0
            x2=800
            y1=self.calc_y_pix(line_def,x1)
            y2=self.calc_y_pix(line_def,x2)
            cv2.line(image_near, (x1,y1), (x2,y2), blue , 1)
            x=740
            y=self.calc_y_pix(line_def,x)+5
            cv2.putText(image_near, "{}cm".format((int)(y_m*100)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        image_far=cv2.imread("/home/roman/measure/far.jpg")
        for y_m in [0.96,0.8,0.64,0.48,0.32,0.16,0,-0.16,-0.32,-0.48,-0.64,-0.8,-0.96]:
            x_pix, y_pix = self.far_m_to_pixel(0, y_m)
            cv2.line(image_far, (x_pix, y_pix), (FAR_FP_X, FAR_FP_Y), blue, 1)

        for x_m in [0.12,0.14,0.16,0.18,0.2,0.22,0.24,0.26]:
            x_pix,y_pix = self.far_m_to_pixel(x_m,0)
            cv2.line(image_far, (0, y_pix), (800, y_pix), blue, 1)
            cv2.putText(image_far, "{}cm".format((int)(x_m*100)), (740, y_pix+5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        for i in range(0,len(STABLE_POLY)):
            p1=STABLE_POLY[i]
            p2=STABLE_POLY[(i+1)%len(STABLE_POLY)]
            x1_pix, y1_pix = self.far_m_to_pixel(p1.x, p1.y)
            x2_pix, y2_pix = self.far_m_to_pixel(p2.x, p2.y)
            cv2.line(image_far, (x1_pix, y1_pix), (x2_pix, y2_pix), yellow, 1)
            x1_pix, y1_pix = self.near_m_to_pixel(p1.x, p1.y)
            x2_pix, y2_pix = self.near_m_to_pixel(p2.x, p2.y)
            cv2.line(image_near, (x1_pix, y1_pix), (x2_pix, y2_pix), yellow, 1)

        image = cv2.vconcat([image_near, image_far])

        cv2.imshow('Image', image)
        cv2.waitKey(100000)

#CameraCoordinates().test()

