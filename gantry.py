import RPi.GPIO as GPIO
import time
import math
import numpy as np
from PIL import Image

def get_polyline(polyline_string, dpi):
    """
    Returns a polyline (list of coordinates with each coordinate being a list of
    [x,y]) from a string in "x1,y1,x2,y2,..." form
    Args:
        polyline_string (string): polyline coords in "x1,y1,x2,y2,..." form
        dpi (int): steps per inch value that the hpgl file was generated with.
        This is used for converting to inches global coordinate system.
    """
    unsorted_coords = polyline_string.split(",")
    polyline = []
    for i in range(len(unsorted_coords)//2):
        x_i = int(unsorted_coords[i*2])   / dpi
        y_i = int(unsorted_coords[i*2+1]) / dpi
        polyline.append([x_i, y_i])
    return polyline


def get_multiple_polylines(polyline_strings, dpi):
    """
    Returns a list of polylines (each polyline is a list of coordinates and
    each coordinate is a list [x,y])
    Args:
        polyline_strings (list of strings): list of strings in
        "x1,y1,x2,y2,..." form
        dpi (int): steps per inch value that the hpgl file was generated with.
        This is used for converting to inches global coordinate system.
    """
    return [get_polyline(polyline_string, dpi) for polyline_string in \
                                                   polyline_strings]

def get_polylines_from_hpgl(filepath, dpi):
    """
    Generates and returns a list of polylines from an hpgl file
    Args:
        filepath (string): path to the hpgl file
        dpi (int): steps per inch value that the hpgl file was generated with.
        This is used for converting to inches global coordinate system.
    """
    with open(filepath, "r") as file:
        # read the one-liner hpgl file into a string
        lines = file.readlines()
        # create list of lines in the hpgl file
        lines = lines[0].split(";")

        # list of all pen up commands
        pu_lines = [line[2:] for line in lines if line[0:2] == "PU"][1:-1]

        # list of all pen down commands
        pd_lines = [line[2:] for line in lines if line[0:2] == "PD"]

        full_lines = []

        for i in range(len(pd_lines)):
            full_lines.append(pu_lines[i] + "," + pd_lines[i])

        # create list of all unsorted poly lines, e.g. "x1,y1,x2,y2,x3,y3"
        # which in [x,y] form should be [[x1,y1],[x2,y2],[x3,y3]
        unsorted_polylines = [line for line in full_lines]

        return get_multiple_polylines(unsorted_polylines, dpi)

# def setkeypoints():
#     keypoints = []
#     # Line
#     # x = 5
#     # for y in np.linspace(3,-3,10):
#     #     keypoints.append([x,y])
#     #     x *= -1

#     # Circle
#     r = 3
#     for theta in range(360):
#             rad = math.radians(theta)
#             keypoints.append([r*math.sin(rad), r*math.cos(rad)])
#     return keypoints

def draw_poly(g,keys,speed,power):
    g.move_global(keys[0], 1)

    g.setlaser(power)
    for point in keys[1:]:
        g.move_global(point, speed)
    g.setlaser(0)


def draw_line(g, keys, speed, power, ppi):
    g.move_global(keys[0], 3)
    last_point = keys[0]

    g.setlaser(power)
    for point in keys[1:]:
        for step in extrap_line(np.array(last_point), np.array(point), ppi):
            g.move_global(step, speed)
        last_point = point
    g.setlaser(0)

def extrap_line(point1, point2, ppi):
    """Function extrapolates points in a line between two points with 'ppi' points per inch"""
    pol = cart2pol(point2-point1)
    vector_lengths = np.linspace(0, pol[0], math.ceil(pol[0]*ppi))
    line = []
    for rho in vector_lengths:
        line.append(pol2cart([rho, pol[1]]))
    return line+point1

def cart2pol(cart):
    return [np.sqrt(cart[0]**2 + cart[1]**2), np.arctan2(cart[1], cart[0])]

def pol2cart(pol):
    return [pol[0] * np.cos(pol[1]), pol[0] * np.sin(pol[1])]

class gantry:
    def __init__(self):
        self.setup_pi()

        # Global commanded position of the gantry in inches
        self.X = 0
        self.Y = 0

        # Stepper moter steps to inch scalar
        self.step2inch = 20000/9.75 # Found Experimentally

    def setup_pi(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.DIR1 = 6
        self.DIR2 = 19
        self.STEP1 = 13
        self.STEP2 = 26
        self.LAZ = 4

        GPIO.setup(self.LAZ, GPIO.OUT)
        GPIO.setup(self.DIR1, GPIO.OUT)
        GPIO.setup(self.DIR2, GPIO.OUT)
        GPIO.setup(self.STEP1, GPIO.OUT)
        GPIO.setup(self.STEP2, GPIO.OUT)

        self.laserpwm = GPIO.PWM(self.LAZ, 8000)

        # Setup boards for 1:16 microstepping
        Mode = (16, 20, 21)
        for pin in Mode:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 1)

    def setlaser(self, duty):
        self.laserpwm.start(duty)

    def move(self, x, y, sps):
        """Function moves gantry x, y steps at 'sps' Steps Per Second"""
        GPIO.output(self.DIR1, int(x>0))
        GPIO.output(self.DIR2, int(y<0))

        # Use given speed and distance to compute total time of move
        movtime =  math.sqrt(x**2 + y**2)/sps

        x_time = 0
        y_time = 0

        start = time.time()
        while True:
            elapsed = time.time() - start
            if elapsed>movtime:
                break

            if abs(x)>0 and elapsed - x_time > movtime/abs(x):
                GPIO.output(self.STEP1, 1)
                x_time = elapsed

            if abs(y)>0 and elapsed - y_time > movtime/abs(y):
                GPIO.output(self.STEP2, 1)
                y_time = elapsed

            GPIO.output(self.STEP1, 0)
            GPIO.output(self.STEP2, 0)

    def move_global(self, target, ips):
        """Function moves gantry to inch coordinates x,y at 'ips' Inches Per Second"""
        xdiff = (target[0] - self.X) * self.step2inch
        ydiff = (target[1] - self.Y) * self.step2inch

        if xdiff or ydiff:
            self.move(xdiff, ydiff, ips*self.step2inch)
            self.X = target[0]
            self.Y = target[1]

if __name__ == "__main__":
    g = gantry()
    just_laser = False

    if just_laser:
        g.setlaser(100)
        time.sleep(100)
    else:
        for path in get_polylines_from_hpgl('shapes.hpgl', 500):
            draw_line(g, np.array(path)/2, 2, 50, 2)
        
        g.move_global([0,0], 1)
    