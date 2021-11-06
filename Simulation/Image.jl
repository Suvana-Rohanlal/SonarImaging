f0 = 39000;

using JLD2
using FileIO

mutable struct Received_pulses
    ReceivedPulse::ComplexF64
end

d = load("BasebandedSignal.jld2", "baseband")

v_tx = d["transmit"]
lengthOfTransmitted = length(v_tx)

r_tx = d[0].pulse;

lengthOfReceived = length(r_tx)

c = 343;
fs = 400000; # sample rate of sonar, 44100 original 100 000
dt = 1/fs; # sample spacing
TargetXRange=2
k =0
for i in 0:0.005:TargetXRange
     k=k+1
end

TargetYRange=10
j =0
for s in 1:0.005:TargetYRange
     j=j+1
end

j



function calc_td(target,receiver)
    r = target-receiver
    R = sqrt(sum(r.*r))
    if(R== 2.0007838463962067)
        print("true")
    end
    td = 2*R/c
    return td
end


numOfReceivers = 8
ImageArray = zeros(Complex,k,j)
#reshape(Complex.(ImageArray),k,j)

receiverSpacing = 0.016
x1 = -((numOfReceivers-1)/2*receiverSpacing)
indexOfReceived =0
for n in 1:numOfReceivers
    xn = x1+(n-1)*receiverSpacing
    receiver_position = d[indexOfReceived].coordinates
    r_tx = d[indexOfReceived].pulse
    indexOfReceived = indexOfReceived+1
    x1=1
    y1=1
    
    for x in -TargetXRange/2:0.005:TargetXRange/2
        for y in 1:0.005:10
            td_calculated = calc_td([x,y,0],receiver_position)
            tdArray_index = Int(round(td_calculated/dt))
            if tdArray_index <= lengthOfReceived
                i=im
                ImageArray[CartesianIndex.( x1,y1)] += (r_tx[tdArray_index]) * exp(i*2*pi*f0*td_calculated)
                
            end 
            y1+=1
        end
        x1=x1+1
        y1=1
    end
    
end


#ImageTimeArray[CartesianIndex.(3, 3)]= ImageTimeArray[CartesianIndex.(3, 3)]*1000
@show j
@show k

using Plots; gr()



@show length(ImageArray)
y=0:0.005:TargetXRange
x=1:0.005:TargetYRange
Plots.heatmap(x,y,(abs.(ImageArray)).^0.25, xlabel="Y", ylabel="X")
#Plots.heatmap(x,y,(abs.(ImageArray)), xlabel="Y", ylabel="X")
#Plots.heatmap((10*log10.(abs.(ImageArray))), xlabel="Y", ylabel="X")

using PyPlot
imshow(abs.(ImageArray).^0.4)

using Statistics
B = mean(abs.(ImageArray), dims=3)[:,:,1]

function imshow_scale(A)
    # Like imshow(A) but scales the values to [0,1] and supports grayscale
    
    A .-= minimum(A)            # Scale and shift to [0,1]
    A ./= maximum(A)
    if ndims(A) < 3
        A = reshape(A, size(A,1), size(A,2), 1)
    end
    if size(A,3) == 1
        A = repeat(A, 1, 1, 3)  # Set R=G=B for grayscale
    end
    imshow(A)
end


function image_threshold(A, th)
    return Float64.(A .> th)
end

imshow_scale(B);
#imshow_scale(1.0 .- image_threshold(B, 0.8));




