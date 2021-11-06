using OrderedCollections
using Plots
plotly()
using FFTW
using JLD2
using FileIO

mutable struct PointTargets
    coordinates
    reflection::Float64
end

mutable struct Receivers
    coordinates
    pulse
end


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
dict = OrderedDict{Any,Any}("transmit" => v_tx)
#figure()


fig = plot(v_tx,label="transmitted pulse")
xlabel!("time")
display(fig)

@show length(v_tx)

V_TX = fft(v_tx); # Fourier transform chirp pulse
display(plot( abs.(V_TX), label="Fourier transform of the transmitted pulse" ));

#Generate the targets
numOfTargets = 20
#Generate the receiving arrays
numOfReceivers = 8
ArrayOfTargets = Array{PointTargets,1}(undef, numOfTargets)

TargetXRange = 1
for targets in 1:numOfTargets
    targetCoordinate = [rand(-TargetXRange/2:0.05:TargetXRange/2),rand(1:1:10),0]
    #targetCoordinate = [0.5, 2,0.0]
    targetStruct = PointTargets(targetCoordinate,10)
    ArrayOfTargets[targets] = (targetStruct)

    
    
end

 #=   targetCoordinate1 = [0.2,6.0,0.0]
    targetStruct1 = PointTargets(targetCoordinate1,10)
    ArrayOfTargets[2] = (targetStruct1)

 targetCoordinate2 = [-0.5,8.0,0.0]
    targetStruct2 = PointTargets(targetCoordinate2,10)
    ArrayOfTargets[3] = (targetStruct2)
=#

@show ArrayOfTargets

transmitter = [0,0,3] #z=4
#receiver_1 = [0,0,3]
#receiver_2 = [0,1,3]
#receiver_3 = [0,2,3]
#receiver_4 = [0,3,3]

receivers = []
ArrayOfReceivers = Array{Receivers,1}(undef, numOfReceivers)

function calcV_RX(target,receiver)
    r = target-receiver
    R = sqrt(sum(r.*r))
   # println(R)
    td = 2*R/c
    A = 1/R^2
   # println(A)
    v_rx = A*chirp.(t.-td)
    return v_rx
end
key_label = 0

i=0
println("starting")
receiverSpacing = 0.016
x1 = -((numOfReceivers-1)/2*receiverSpacing)
for n in 1:numOfReceivers
    x = x1+(n-1)*receiverSpacing
    receiver_position = [x,0,0]
    push!(receivers,receiver_position)

    #println(receiver_position)
     v_receiver = zeros(size(v_tx))
    for pointTarget in 1:numOfTargets
        point = ArrayOfTargets[pointTarget].coordinates
        receiving_rx = calcV_RX(point, receiver_position)
        v_receiver = v_receiver + receiving_rx
    end
    
    receivingStructElement = Receivers(receiver_position,v_receiver) 
    pulse_label = string("Received pulse ",key_label)
    push!(dict,key_label => receivingStructElement)
    key_label=key_label+1
    display(plot(r,v_receiver,label=pulse_label))
end



save("chirps.jld2", "chirps",dict)
dict



X=[]
Y=[]
Z=[]




for r in 1:length(receivers)
    push!(X, receivers[r][1])
    push!(Y, receivers[r][2])
    push!(Z, receivers[r][3])
end

for p in 1:numOfTargets
    push!(X, ArrayOfTargets[p].coordinates[1])
    push!(Y, ArrayOfTargets[p].coordinates[2])
    push!(Z, ArrayOfTargets[p].coordinates[3])
end

plt3d= Plots.plot(X,Y, Z,
     seriestype=:scatter, markersize = 4, xlabel = "x", ylabel = "y", zlabel = "z")







