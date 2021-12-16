from pyo import *
s = Server(duplex=0).boot()
s.start()
a = Sine(mul=0.01).out()