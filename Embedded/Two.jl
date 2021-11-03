using SerialPorts
#using PyPlot
using Plots
plotly() 
using FFTW
using Statistics

list_serialports() 
sp = SerialPort("/dev/ttyACM0", 9600) 


