from pyo import *

s = Server(duplex=0).boot()
s.start()
### Oscilloscope ###

# LFO applied to the `chaos` attribute
lfo = Sine(0.2).range(0, 1)

## Rossler attractor
#n1 = Rossler(pitch=0.5, chaos=lfo).out()
#fm1 = FM(carrier=250, ratio=[1.5, 1.49], index=10, mul=0.3).out()
freq = Rossler(0.005, chaos=0.7, stereo=True, mul=250, add=500)
a = Sine(freq, mul=0.8).out()
#n2 = Lorenz(pitch=0.5, chaos=lfo, stereo=True).out()
while True:
    pass
# Lorenz attractor
#n2 = Lorenz(pitch=0.5, chaos=lfo, stereo=True)

# ChenLee attractor
#n3 = ChenLee(pitch=0.5, chaos=lfo, stereo=True)


