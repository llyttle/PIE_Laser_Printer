import RPi.GPIO as GPIO
import time
import math
import numpy as np
from PIL import Image
import random

# def setkeypoints():
#     keypoints = []
    # Line
    # x = 5
    # for y in np.linspace(3,-3,10):
    #     keypoints.append([x,y])
    #     x *= -1

    # Circle
    # r = 3
    # for theta in range(360):
    #         rad = math.radians(theta)
    #         keypoints.append([r*math.sin(rad), r*math.cos(rad)])
    # return keypoints

class Gantry:
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

class ToolFile:
    def __init__(self, filename, res=500):
        filestat = filename.split('.')
        if filestat[-1] == 'hpgl':
            print('Recognized '+filestat[0]+' as .'+filestat[-1])
            self.I = np.ones((200,200))
            self.path = self.get_polylines_from_hpgl(filename, 500)

        elif filestat[-1] == 'jpg':
            print('Recognized '+filestat[0]+' as .'+filestat[-1])
            gray = Image.open(filename).convert('L')
            resized = gray.resize((200,200))
            self.I = np.asarray(resized)
            self.I = (255-self.I)/255  # Inverse light/dark, normalize over 255

            self.path = [self.gen_keypoints(res)]

        else:
            raise AttributeError('Cannot read .'+filestat[-1]+' files')
    
    def path_count(self):
        return len(self.path)
    
    def span(self):
        extreme_points = []
        for line in self.path:
            extreme_points.append(np.max(line,0))
            extreme_points.append(np.min(line,0))
        
        return max(np.ptp(np.array(extreme_points),axis=0))
    
    def MapPoint(self, point, sz):
        file2bed_scalar = sz/self.span()
        if self.I is not None:
            x = point[0]+np.size(self.I,0)/2
            y = point[1]+np.size(self.I,1)/2
            a = self.I[round(x)-1][round(y)-1]
            return [point[0]*file2bed_scalar, point[1]*file2bed_scalar, a]
        else:
            return [point[0]*file2bed_scalar, point[1]*file2bed_scalar, 1]
        
    def get_polyline(self, polyline_string, dpi):
        """Returns a polyline (list of coordinates with each coordinate being a list of
        [x,y]) from a string in "x1,y1,x2,y2,..." form"""
        unsorted_coords = polyline_string.split(",")
        polyline = []
        for i in range(len(unsorted_coords)//2):
            x_i = int(unsorted_coords[i*2])   / dpi
            y_i = int(unsorted_coords[i*2+1]) / dpi
            polyline.append([x_i, y_i])
        return polyline

    def get_multiple_polylines(self, polyline_strings, dpi):
        """Returns a list of polylines (each polyline is a list of coordinates and
        each coordinate is a list [x,y])"""
        return [self.get_polyline(polyline_string, dpi) for polyline_string in \
                                                    polyline_strings]

    def get_polylines_from_hpgl(self, filepath, dpi):
        """Generates and returns a list of polylines from an hpgl file"""
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

            return self.get_multiple_polylines(unsorted_polylines, dpi)

    def gen_keypoints(self, n):
        norm = n/np.sum(self.I)    # Scalar that maps from each pixel summing to 1 to entire array summing to n
        prob_matrix = norm*self.I

        sz = len(prob_matrix)
        r = np.random.random((sz,sz))   # Random number matrix reference

        keys = []
        for y,row in enumerate(r<prob_matrix):
            for x,point in enumerate(row):
                if point:
                    keys.append([x-(sz/2),(sz/2)-y])    # while appending, center points on 0,0 and flip y
        random.shuffle(keys)
        return keys

def populatePath(T, ppi):
    PP = []
    for line in T.path:
        refined_line = []
        last_point = np.array(line[0])
        for point in line[1:]:
            [refined_line.append(new_p) for new_p in extrap_line(last_point, np.array(point), ppi)]
            last_point = point
        PP.append(refined_line)

    return PP

def MasterPath(G, polyline, sz, I):
    for line in polyline:
        for point in line:
            p = np.array(point) * sz/I.span()
            a = I.I[round(p[0])-1][round(p[1])-1]
            G.move_global(p, .5)
            G.setlaser(100)
        G.setlaser(0)
            

def extrap_line(point1, point2, ppi):
    """Function extrapolates points in a line between two points with 'ppi' points per inch"""
    pol = cart2pol(point2-point1)
    vector_lengths = np.linspace(0, pol[0], math.ceil(pol[0]*ppi))

    line = [pol2cart([rho,pol[1]])+point1 for rho in vector_lengths]
    
    return line

def cart2pol(cart):
    return np.array([np.sqrt(cart[0]**2 + cart[1]**2), np.arctan2(cart[1], cart[0])])

def pol2cart(pol):
    return np.array([pol[0] * np.cos(pol[1]), pol[0] * np.sin(pol[1])])

if __name__ == "__main__":
    G = Gantry()
    # T = ToolFile('totoro.jpg', 800) # Import totoro.jpg, populate with n points
    T = ToolFile('addendum.hpgl')
    T = ToolFile('shapes.hpgl')
    # T = ToolFile('square.jpg', 500)

    just_laser = False

    if just_laser:
        G.setlaser(100)
        time.sleep(100)
    else:
        P = populatePath(T, ppi=10) # Ensures that all points are spaced out 'ppi' points per inch.

        print('finished populating')

        MasterPath(G, P, 7, T)

        G.move_global([0,0],2)


        # print([m for m in M])
        
        # for Line in M:
        #     G.move_global(Line[0], 3)

        #     G.setlaser(100)
        #     for point in Line[1:]:
        #         G.move_global(point, 2)
        #         print(point)
        #     G.setlaser(0)
    


    # G.move_global([0,0], 1)
    
        
        