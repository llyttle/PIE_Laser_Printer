import serial 
import os
from pyo import *
import RPi.GPIO as GPIO
import time
import math
import pygame

class gantry:
    def __init__(self):
        self.setup_pi()

        # Global current position of the gantry in inches
        self.X = 0
        self.Y = 0

        # Stepper moter steps to inch scalar
        self.step2inch = 20000/9.75 # Found Experimentally
    def idle(self):
      while True:
        update_system(ser,surface)

    def joystick(self):
      while True:
        update_system(ser,surface)
        self.move(joystick_control.unit_valx, joystick_control.unit_valy, 1)

    def draw_circle(self):
        for theta in range(360):
            update_system(ser,surface)
            rad = math.radians(theta)
            self.move_global(math.sin(rad), math.cos(rad), 1)
            

    def setup_pi(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.DIR1 = 6
        self.DIR2 = 19
        self.STEP1 = 13
        self.STEP2 = 26

        GPIO.setup(self.DIR1, GPIO.OUT)
        GPIO.setup(self.DIR2, GPIO.OUT)
        GPIO.setup(self.STEP1, GPIO.OUT)
        GPIO.setup(self.STEP2, GPIO.OUT)
        
        # Setup boards for 1:16 microstepping
        Mode = (16, 20, 21)
        for pin in Mode:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 1)

    def move_global(self, target_x, target_y, ips):
        """Function moves gantry to inch coordinates x,y at 'ips' Inches Per Second"""
        xdiff = (target_x - self.X) * self.step2inch
        ydiff = (target_y - self.Y) * self.step2inch

        if xdiff or ydiff:
            self.move(xdiff, ydiff, ips*self.step2inch)
            self.X = target_x
            self.Y = target_y

    def move(self, x, y, sps):
        """Function moves gantry x, y steps at 'sps' Steps Per Second"""
        GPIO.output(self.DIR1, x<0)
        GPIO.output(self.DIR2, y>0)

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
def poll_serial(ser_serv):
  input_dict = dict()
  line = []
  while len(line) != 17:
    line = []
    if ser_serv.in_waiting > 0:
      line = ser_serv.readline().decode('utf-8', errors='replace').split(",")
  for i in line:
    rip_dict(i, input_dict)
  return input_dict
def rip_dict(entry, dic):
  print(entry)
  stuff = entry.split(":")
  dic[stuff[0]] = int(stuff[1])
  return dic
class digital_input():
  def __init__(self, flag, state):
    self.flag = flag
    self.state = state
class switch(digital_input):
  def change_state(self,input):
    self.state = input
class tap_button(digital_input):
  def change_state(self,input):
    self.state = input
class sound_switch(switch):
  def __init__(self, flag, state):
    self.flag = flag
    self.state = state
    self.on = 0
    self.signal = None
  def change_state(self, input):
    self.state = input
    if self.state:
      self.turn_on()
    else:
      self.turn_off()
  def turn_on(self):
      pass
  def turn_off(self):
      pass
class white_noise_switch(sound_switch):
  def turn_on(self):
    if self.on:
      return
    else:
      self.signal = Noise(0.3).out()
      self.on = 1
      return
  def turn_off(self):
    if self.on:
      self.signal.stop()
      self.on = 1
    else:
      return
class pink_noise_switch(sound_switch):
  def turn_on(self):
    if self.on:
      return
    else:
      self.signal = PinkNoise(0.3).out()
      self.on = 1
      return
  def turn_off(self):
    if self.on:
      self.signal.stop()
      self.on = 1
    else:
      return
class brown_noise_switch(sound_switch):
  def turn_on(self):
    if self.on:
      return
    else:
      self.signal = BrownNoise(0.3).out()
      self.on = 1
      return
  def turn_off(self):
    if self.on:
      self.signal.stop()
      self.on = 1
    else:
      return
class off_switch(switch):
  def change_state(self, input):
    self.state = input
    if self.state:
      os.system('echo 1 > /proc/sys/kernel/sysrq && echo b > /proc/sysrq-trigger')
    else:
      return

class brown_noise_switch(sound_switch):
  def turn_on(self):
    if self.on:
      return
    else:
      self.signal = BrownNoise(0.3).out()
      self.on = 1
      return
  def turn_off(self):
    if self.on:
      self.signal.stop()
      self.on = 1
    else:
      return
class Rossler_switch(sound_switch):
  def turn_on(self):
    if self.on:
      return
    else:
      self.signal = Sine(Rossler(0.003, chaos=0.7, mul=250, add=500), mul=0.3).out()
      self.on = 1
      return
  def turn_off(self):
    if self.on:
      self.signal.stop()
      self.on = 1
    else:
      return
class ChenLee_switch(sound_switch):
  def turn_on(self):
    if self.on:
      return
    else:
      self.signal = Sine(ChenLee(0.003, chaos=0.7, mul=250, add=500), mul=0.3).out()
      self.on = 1
      return
  def turn_off(self):
    if self.on:
      self.signal.stop()
      self.on = 1
    else:
      return
class Lorenz_switch(sound_switch):
  def turn_on(self):
    if self.on:
      return
    else:
      self.signal = Sine(Lorenz(0.005, chaos=0.7, mul=250, add=500), mul=0.3).out()
      self.on = 1
      return
  def turn_off(self):
    if self.on:
      self.signal.stop()
      self.on = 1
    else:
      return
class analog_input():
  def __init__(self, flag, val):
    self.flag = flag
    self.val = val
class knob(analog_input):
  def __init__(self, flag, val, map_low, map_hi):
    self.flag = flag
    self.val = val
    self.map_low = map_low
    self.map_hi = map_hi
  def set_val(self,val):
    self.val = map(val, 0, 1023, self.map_low, self.map_hi)
    return self.val
  def map_val(self,val):
    return map(val, 0, 1023, self.map_low, self.map_hi)
class joystick_controller():
  def __init__(self, flagx, flagy):
    self.flagx = flagx
    self.flagy = flagy
    self.unit_valx = 0
    self.unit_valy = 0
    self.x_pot = knob(flagx, 0, -512, 512)
    self.y_pot = knob(flagy, 0, -512, 512)
  def set_unit_val(self, inputx, inputy):
    x_vec = self.x_pot.map_val(inputx)
    y_vec = self.y_pot.map_val(inputy)
    self.unit_valx = x_vec / math.sqrt(x_vec ** 2 + y_vec **2)
    self.unit_valy = y_vec / math.sqrt(x_vec ** 2 + y_vec **2)
    return
class master_vol_knob(knob):
  def update_volume(self,ain):
    s.amp = self.set_val(ain)
    return
class mech_control_knob(knob):
    # map_hi is num_states here
    def __init__(self, flag, val, state, map_low, map_hi, button_flag):
      self.flag = flag
      self.val = val
      self.state = state
      self.map_low = map_low
      self.map_hi = map_hi
      self.button = tap_button(button_flag, 0)
    def set_state(self, val_in, din):
      new_state = int(self.map_val(val_in))
      din_last_state = self.button.state
      self.button.change_state(din)
      if self.state != new_state:
        if din_last_state == 1:
          self.state = new_state
          self.run_state()
    def run_state(self):
      if self.state == 0:
        g.idle()
      elif self.state == 1:
        g.draw_circle()
      elif self.state == 2:
        g.joystick()
      return
class freq_knob(knob):
  def __init__(self, flag, val, state, map_low, map_hi, switch_flag1, switch_flag2):
    self.flag = flag
    self.val = val
    self.state = state
    self.map_low = map_low
    self.map_hi = map_hi
    self.on_switch = switch(switch_flag1, 0)
    self.state_switch = switch(switch_flag2, 0)
    self.freq1 = val
    self.freq2 = val
    self.sine1_on = 0
    self.sine2_on = 0
    self.sine1 = Sine()
    self.sine2 = Sine()
  def set_freq(self, ain, din_on, din_state):
    frequency = self.map_val(ain)
    self.on_switch.change_state(din_on)
    self.state_switch.change_state(din_state)
    if self.on_switch.state:
      if self.state_switch.state == 0:
        if self.sine1_on == 0:
          self.freq1 = frequency
          self.sine1 = Sine(freq = frequency)
          self.sine1.out()
          self.sine1_on = 1
        else:
          self.freq1 = frequency
          self.sine1.setFreq(frequency)
          self.sine1.out()
      if self.state_switch.state == 1:
        if self.sine2_on == 0:
          self.freq2 = frequency
          self.sine2 = Sine(freq = frequency)
          self.sine2.out()
          self.sine2_on = 1
        else:
          self.freq2 = frequency
          self.sine2.setFreq(frequency)
          self.sine2.out()
    if self.on_switch.state == 0:
      if self.state_switch.state == 0:
        if self.sine1_on:
          self.sine1.stop()
          self.sine1_on = 0
      if self.state_switch.state == 1:
        if self.sine2_on:
          self.sine2.stop()
          self.sine2_on = 0
class frequency_mod_knob(knob):
  def __init__(self, flag, val, state, map_low, map_hi, switch_flag1):
    self.flag = flag
    self.val = val
    self.map_low = map_low
    self.map_hi = map_hi
    self.on_switch = switch(switch_flag1, state)
    self.carrier = val
    self.signal_on = 0
    self.singal = None
  def set_carrier(self, ain, din_on):
    self.carrier = self.map_val(ain)
    self.on_switch.change_state(din_on)
    if self.on_switch.state:
        if self.signal_on == 0:
          self.singal = CrossFM(carrier=self.carrier, ratio=[1.5, 1.49], ind1=10, ind2=2, mul=0.3).out()
          self.signal_on = 1
        else:
          self.singal.setCarrier(carrier).out()
    elif self.signal_on:
        self.singal.stop()
        self.signal_on = 0
def map(val, in_low, in_hi, out_low, out_hi):
  return (val-in_low) * (out_hi - out_low) / (in_hi - in_low) + out_low
class TextBox:
    """
    Defines a Static Text Box

    Class works similar to the InputBox class but
    simply displays given text and has no user
    interaction

    Code adapted from: https://bit.ly/3m5Nq03

    Attributes:
        rect: defines as a rectangle
        x: int X position of text box
        y: int Y position of text box
        width: int width of text box
        height: int height of text box
        text: string input text the text box holds
        rect: Defines as text box as a
            rectangle using x, y, width & height
        color: Color of button
        txt_surface = renders text directly on
            text box
    """

    def __init__(self, x, y, w, h, text=''):
        self.rect = pygame.Rect(x, y, w, h)
        self.color = COLOR_INACTIVE
        self.text = text
        self.txt_surface = FONT.render(text, True, self.color)
        self.val = 0
        self.knob = knob
        self.texti = text

    def update(self):
        # Resize the box if the text is too long.
        width = max(200, self.txt_surface.get_width() + 10)
        self.rect.w = width
    
    def draw_switch(self, surface, s):
      self.handle_color(s)
      surface.blit(self.txt_surface, (self.rect.x + 5, self.rect.y + 5))
      pygame.draw.rect(surface, self.color, self.rect, 2)

    def draw_knob(self, surface, val, s):
      # Blit the text.
      self.handle_color(s)
    
      self.text= self.texti + str(int(val))
      self.txt_surface = FONT.render(self.text, True, self.color)
      surface.blit(self.txt_surface, (self.rect.x + 5, self.rect.y + 5))

        # Draw the rect.
      pygame.draw.rect(surface, self.color, self.rect, 2)
    def handle_color(self, s):
      if s:
        self.color = COLOR_ACTIVE
      else:
        self.color = COLOR_INACTIVE

class user_interface:
  def __init__(self,surface):
    #pygame.init()
    self.clock = pygame.time.Clock()

    # Starts up a bunch of the text boxes and buttons
    
    self.vol_knob_b = TextBox(5, 15, 86, 25, "Master Vol 1: ")
    self.f1_knob_b = TextBox(5, 56.5, 86, 25, "Freq 1:")
    self.f2_knob_b = TextBox(5, 98, 86, 25, "Freq 2:")
    self.fm_knob_b = TextBox(5, 139.5, 86, 25, "Carrier:")
    self.wnoise_switch_b = TextBox(100, 15, 55, 35, "White")
    self.pnoise_switch_b = TextBox(170, 15, 55, 35, "Pink")
    self.bnoise_switch_b = TextBox(230, 15, 55, 35, "Brown")
    self.chenlee_switch_b = TextBox(100, 56.5, 55, 35, "ChenLee")
    self.lorenz_switch_b = TextBox(170, 56.5, 55, 35, "Lorenz")
    self.rossler_switch_b = TextBox(230, 56.5, 55, 35, "Rossler")
    self.m_knob_b = TextBox(100, 98, 170, 29, "Mode:")

    

    # Put all the text boxes and buttons into lists for
    # easy drasurfaceg
    self.boxes = [ self.vol_knob_b,
    self.f1_knob_b,
    self.f2_knob_b,
    self.fm_knob_b,
    self.wnoise_switch_b,
    self.pnoise_switch_b,
    self.bnoise_switch_b,
    self.chenlee_switch_b,
    self.lorenz_switch_b,
    self.rossler_switch_b,
    self.m_knob_b]
    
    self.vol_knob_b.draw_knob(surface, 0, 1)
    
    self.f1_knob_b.draw_knob(surface,0, 0)
        
    self.f2_knob_b.draw_knob(surface,0, 0)
    
    self.fm_knob_b.draw_knob(surface,0, 0)
    self.wnoise_switch_b.draw_switch(surface,0)
    self.pnoise_switch_b.draw_switch(surface,0)
    self.bnoise_switch_b.draw_switch(surface,0)
    self.chenlee_switch_b.draw_switch(surface,0)
    self.lorenz_switch_b.draw_switch(surface,0)
    self.rossler_switch_b.draw_switch(surface,0)
    self.m_knob_b.draw_knob(surface,0, 1)
    
    pygame.display.flip()
    self.clock.tick(30)
  def update_ui(self, surface, vol_val, f_s1val, f_s2val, f_s1on, f_s2on, fm_val, fm_s,  wn_s, pn_s, bn_s, chen_s, lor_s, ros_s, m_val):
    surface.fill((30, 30, 30))
    self.vol_knob_b.draw_knob(surface,vol_val, 1)
    if f_s1on:
        self.f1_knob_b.draw_knob(surface,f_s1val, 1)
    else:
        self.f1_knob_b.draw_knob(surface,f_s1val, 0)
    if f_s2on:
      self.f2_knob_b.draw_knob(surface,f_s2val, 1)
    else:
      self.f2_knob_b.draw_knob(surface,f_s2val, 0)
    self.fm_knob_b.draw_knob(surface,fm_val, 1)
    self.wnoise_switch_b.draw_switch(surface,wn_s)
    self.pnoise_switch_b.draw_switch(surface,pn_s)
    self.bnoise_switch_b.draw_switch(surface,bn_s)
    self.chenlee_switch_b.draw_switch(surface,chen_s)
    self.lorenz_switch_b.draw_switch(surface,lor_s)
    self.rossler_switch_b.draw_switch(surface,ros_s)
    self.m_knob_b.draw_knob(surface,m_val, 1)
    
    pygame.display.flip()
    self.clock.tick(30)
    return

def update_system(ser,surface):
  input = poll_serial(ser)
  vol_knob.update_volume(input[vol_knob.flag])
  f_knob.set_freq(input[f_knob.flag], input[f_knob.on_switch.flag], input[f_knob.state_switch.flag])
  fm_knob.set_carrier(input[fm_knob.flag], input[fm_knob.on_switch.flag])
  wnoise_switch.change_state(input[wnoise_switch.flag])
  pnoise_switch.change_state(input[pnoise_switch.flag])
  bnoise_switch.change_state(input[bnoise_switch.flag])
  chenlee_switch.change_state(input[chenlee_switch.flag])
  lorenz_switch.change_state(input[lorenz_switch.flag])
  rossler_switch.change_state(input[rossler_switch.flag])
  e_shutdown_switch.change_state(input[e_shutdown_switch.flag])
  if m_knob.state == 2:
    joystick_control.set_unit_val(input[joystick_control.flagx],input[joystick_control.flagy])
  ui.update_ui(surface,vol_knob.val*100, f_knob.freq1, f_knob.freq2, f_knob.sine1_on, f_knob.sine2_on, fm_knob.carrier, fm_knob.signal_on, wnoise_switch.on, 
  pnoise_switch.on,
  bnoise_switch.on,
  chenlee_switch.on,
  lorenz_switch.on,
  rossler_switch.on,
  m_knob.state)
  m_knob.set_state(input[m_knob.flag], input[m_knob.button.flag])


  return 
if __name__ == "__main__":
    pygame.init()

    COLOR_INACTIVE = pygame.Color('lightskyblue3')
    COLOR_ACTIVE = pygame.Color('dodgerblue2')
    surface = pygame.display.set_mode((320, 200))

    FONT = pygame.font.Font(None, 16)
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
    ser.flush()
    s = Server(duplex = 0, audio="jack").boot()
    s.start()
    vol_knob = master_vol_knob('ain0',0, 0, 1)
    f_knob = freq_knob("ain1", 0, 0,300, 1200, "din1", "din2")
    m_knob = mech_control_knob("ain2", 0,0, 0, 3, "din0")
    joystick_control = joystick_controller("ain3", "ain4")
    fm_knob = frequency_mod_knob("ain5", 0, 0, 10, 500, "din10")
    wnoise_switch = white_noise_switch("din3", 0)
    pnoise_switch = pink_noise_switch("din4", 0)
    bnoise_switch = brown_noise_switch("din5", 0)
    lorenz_switch = Lorenz_switch("din6", 0)
    rossler_switch = Rossler_switch("din7", 0)
    chenlee_switch = ChenLee_switch("din8", 0)
    e_shutdown_switch = off_switch("din9", 0)
    g = gantry()
    ui = user_interface(surface)
    g.idle()



