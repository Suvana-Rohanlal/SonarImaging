using JLD2
using FileIO
using Plots; gr()
include("./fourChannels.jl")

signalProcessing()

d = load("BasebandedSignal.jld2", "baseband")

v_tx = d["transmit"]
lengthOfTransmitted = length(v_tx)

r_tx = d[1];

lengthOfReceived = length(r_tx)
coordinates = [[-0.015,0,0], [-0.005,0,0], [0.005,0,0], [0.015,0,0]]
################ image ###############
	c = 343;
	fs = 500000; # sample rate of sonar, 44100 original 100 000
	dt = 1/fs; # sample spacing
	TargetXRange=3
	global k =0
	for i in 0:0.005:TargetXRange
     		global k=k+1
	end

	TargetYRange=6
	global j =0
	for s in 1:0.005:TargetYRange
	     global j=j+1
	end
	
	numOfReceivers = 4
	ImageArray = zeros(Complex,k,j)
	#reshape(Complex.(ImageArray),k,j)

	receiverSpacing = 0.01
       global x1 = -((numOfReceivers-1)/2*receiverSpacing)

	for n in 1:numOfReceivers
    		global xn = x1+(n-1)*receiverSpacing
    		receiver_position = coordinates[n]
    		r_tx = d[n]#received[n]#d[indexOfReceived].pulse

   		global x1=1
    		global y1=1
    
    		for x in -TargetXRange/2:0.005:TargetXRange/2
      	  		for y in 1:0.005:6
            		td_calculated = calc_td([x,y,0],receiver_position)
            		tdArray_index = Int(round(td_calculated/dt))
            		if tdArray_index <= lengthOfReceived
                		i=im
                		ImageArray[CartesianIndex.( x1,y1)] += (r_tx[tdArray_index]) * exp(i*2*pi*f0*td_calculated)
                
            		end 
            		global y1+=1
        	end
        	global x1=x1+1
        	global y1=1
    	end
    	end

	

y=-1:0.005:TargetXRange
x=1:0.005:TargetYRange
Plots.heatmap((x,y,(abs.(ImageArray))), xlabel="Y", ylabel="X")
    
