import RPi.GPIO as GPIO
import time
import math
import numpy as np
from PIL import Image
import random

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
        self.laserpwm.start(duty) #provide duty cycle in the range 0-100

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

class ToolVector:
    def __init__(self, filename):
        filestat = filename.split('.')
        self.filename = filename
        if filestat[-1] == 'hpgl':
            print('Recognized '+filestat[0]+' as .'+filestat[-1])
            self.polyline = self.get_polylines_from_hpgl(500)
            self.formatPolyline()
            self.type = 'vector'
        else:
            raise AttributeError('Cannot read .'+filestat[-1]+' files')

    def formatPolyline(self):
        for line in self.polyline:
            for point in line:
                point.append(1)

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

    def get_polylines_from_hpgl(self, dpi):
        """Generates and returns a list of polylines from an hpgl file"""
        with open(self.filename, "r") as file:
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

class ToolImage:
    def __init__(self, filename, res=500):
        filestat = filename.split('.')
        self.filename = filename
        if filestat[-1] == 'jpg':
            print('Recognized '+filestat[0]+' as .'+filestat[-1])
            
            gray = Image.open(filename).convert('L')
            resized = gray.resize((200,200))
            self.I = np.asarray(resized)
            self.I = (255-self.I)/255  # Inverse light/dark, normalize over 255

            self.gen_keypoints(res)
            print('Generated Keypoints')
            self.populatePath()
            print('Populated Path')
            self.MapPoint(3)

            self.polyline = [self.path]
            self.type = 'image'
        else:
            raise AttributeError('Cannot read .'+filestat[-1]+' files')
    
    def path_count(self):
        return len(self.path)
    
    def span(self):
        return max(np.ptp(np.array(self.path), axis=0))

    def gen_keypoints(self, n):
        norm = n/np.sum(self.I)    # Scalar that maps from each pixel summing to 1 to entire array summing to n
        prob_matrix = norm*self.I

        sz = len(prob_matrix)
        r = np.random.random((sz,sz))   # Random number matrix reference

        self.keys = []
        for y,row in enumerate(r<prob_matrix):
            for x,point in enumerate(row):
                if point:
                    self.keys.append([x-(sz/2),(sz/2)-y])    # while appending, center points on 0,0 and flip y
        random.shuffle(self.keys)
    
    def MapPoint(self, sz):
        f2b = sz/self.span() #file to bed dimension scalar
        for point in self.path:
            print(np.asarray((np.shape(self.I)))/2)
            point += np.shape(self.I)/2
            # point[0] = (point[0]+np.size(self.I,0)/2)*f2b
            # point[1] = (point[1]+np.size(self.I,1)/2)*f2b

            point.append(a = self.I[round(point[0])-1][round(point[1])-1])

    def populatePath(self):
        self.path = []
        last_point = np.array(self.keys[0])
        for point in self.keys[1:]:
            [self.path.append(new_p) for new_p in self.extrap_line(last_point, np.array(point))]
            last_point = point
    
    def extrap_line(self, point1, point2):
        """Function extrapolates points in a line between two points with 'ppi' points per inch"""
        pol = self.cart2pol(point2-point1)
        vector_lengths = list(range(round(pol[0])))
        line = [self.pol2cart([rho,pol[1]])+point1 for rho in vector_lengths]
        return line

    def cart2pol(self, cart):
        return np.array([np.sqrt(cart[0]**2 + cart[1]**2), np.arctan2(cart[1], cart[0])])

    def pol2cart(self, pol):
        return np.array([pol[0] * np.cos(pol[1]), pol[0] * np.sin(pol[1])])

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

def DrawToolpath(G, P, speed, power):
    """
    Commands the gantry to draw the hpgl file located at [filename]

    Args:
        G (Gantry): the gantry object
        P (Toolpath): tool file containing the hpgl code
        speed_draw (float): speed of gantry while drawing in in/sec
        speed_jump (float): speed of gantry while jumping between 
                            polylines in in/sec
    """
    if P.type == 'image':
        print('Printing Image Toolpath...')
    elif P.type == 'vector':
        print('Printing Vector Toolpath...')
    else:
        pass
    
    # Split up Path into lines 
    for line in P.polyline:
        G.move_global(line[0], speed)
        
        for point in line:
        #     p = np.array(point) * sz/I.span()
        #     # a = I.I[round(p[0])-1][round(p[1])-1]
            G.setlaser(power)
            G.move_global(point, .5)
        #     G.setlaser(100)
        G.setlaser(0)
    G.move_global([0,0],2)

if __name__ == "__main__":
    G = Gantry()
    # T = ToolImage('totoro.jpg', res=800) # Import totoro.jpg, populate with n points
    # T = ToolVector('max_tshirt_line.hpgl')
    # T = ToolVector('shapes.hpgl')
    
    # T = ToolFile('mini_square.hpgl')
    T = ToolImage('square.jpg', 100)

    just_laser = 0

    if just_laser:
        G.setlaser(100)
        while 1:
            pass
    else:
        print(T.polyline[0])
        # DrawToolpath(G, T, 1, 10)
