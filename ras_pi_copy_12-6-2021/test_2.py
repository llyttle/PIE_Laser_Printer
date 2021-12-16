from pyo import *

s = Server(duplex=0).boot()

### Oscilloscope ###

# LFO applied to the `chaos` attribute
lfo = Sine(0.2).range(0, 1)

# Rossler attractor
n1 = Rossler(pitch=0.5, chaos=lfo, stereo=True)

# Lorenz attractor
n2 = Lorenz(pitch=0.5, chaos=lfo, stereo=True)

# ChenLee attractor
n3 = ChenLee(pitch=0.5, chaos=lfo, stereo=True)

# Interpolates between input objects to produce a single output
sel = Selector([n1, n2, n3]).out()
sel.ctrl(title="Input interpolator (0=Rossler, 1=Lorenz, 2=ChenLee)")

# Displays the waveform of the chosen attractor
sc = Scope(sel)

### Audio ###

# Lorenz with very low pitch value that acts as a LFO
#freq = Lorenz(0.005, chaos=0.7, stereo=True, mul=250, add=500)
#a = Sine(freq, mul=0.3).out()

s.gui(locals())
