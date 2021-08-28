using Pkg
using Plots
plotly()
using FFTW
using JLD2
using FileIO


#Specify parameters of chirp pulse
f0 = 39000; # Centre frequency is 40 kHz
B = 2000; # Chirp bandwidth
#f0 = fc-B/2; # Inital frequency
T =  7E-3; # Chirp pulse length Change back to 5E-3 if nec
K = B/T; # Chirp rate

#Define sampling and range parameters
c = 343; # speed of sound in air in m/s
fs = 400000; # sample rate of sonar, 44100 original 100 000
dt = 1/fs; # sample spacing
r_max = 10; # maximum range in metres
t_max = 2*r_max/c + T; # time delay to max range

# Create an array containing the time values of the samples
t= 0:dt:t_max
# Convert from time axis to range axis
r = c*t/2;

# Define a simple a rect() function which returns 1 for -0.5<=t<=0.5 only
rect(t) = (abs.(t) .<= 0.5)*1.0

blackman(f,B) = abs.((0.42 + 0.5*cos((2π*f)/B) + 0.08*cos((4π*f)/B))*rect(f/B))

chirp(t) = cos( 2*pi*(f0*(t) + 0.5*K*(t)^2) ) * rect(t/T) * sqrt(blackman(t,T))

# Delay the chirp pulse so that it starts after t=0.
td = 0.6*T; # Chirp delay
# Define shifted chirp pulse
v_tx = chirp.(t.-td)


#Save to dictionary
dict = Dict{Any,Any}("transmit" => v_tx)
#figure()


fig = plot(v_tx,label="transmitted pulse")
xlabel!("time")
display(fig)

V_TX = fft(v_tx); # Fourier transform chirp pulse
display(plot( abs.(V_TX), label="Fourier transform of the transmitted pulse" ));

transmitter = [0,1,4]
#receiver_1 = [0,0,3]
#receiver_2 = [0,1,3]
#receiver_3 = [0,2,3]
#receiver_4 = [0,3,3]

point_target = [6,6,6]

rows=1
numReceiversCols = 5

function calcV_RX(x,y)
    r = x-y
    R = sqrt(sum(r.*r))
    td = 2*R/c
    A = 1/R^2
    v_rx = A*chirp.(t.-td)
    return v_rx
end
key_label = 0

for z in 3:(rows-1+3)
    print("starting")
    for y in 0:(numReceivers-1)
        receiver_position = [0,y,z]
        v = calcV_RX(point_target, receiver_position)
        pulse_label = string("Received pulse ",key_label)
        push!(dict,key_label => v)
        key_label=key_label+1
        display(plot(r,v,label=pulse_label))
    end
end


save("chirps.jld2", "chirps",dict)
dict




