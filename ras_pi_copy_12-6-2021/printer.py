from pyo import *

print("Audio host APIS:")
pa_list_host_apis()
pa_list_devices()
print("Default input device: %i" % pa_get_default_input())
print("Default output device: %i" % pa_get_default_output())
