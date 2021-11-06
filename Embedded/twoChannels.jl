#module twoChannels
using SerialPorts
#using PyPlot
using Plots
plotly() 
using FFTW
using Statistics


list_serialports() 
sp = SerialPort("/dev/ttyACM0", 9600) 

################# constants ############################
c = 343 # speed of sound in air
r_max = 10

fc=40000;   # center freq of chirp
T=5E-3      # chirp pulse length
B = 4000    # chirp bandwidth
K = B/T     # chirp rate [Hz/s]
f0 = fc-B/2
 #fs = 4*(1.6*B)
fs = 500000#400000#250000 #100000 400000
λ = c/fc #waveform

####################### Rect ####################################
rect(t) = (abs.(t) .<= 0.5)*1.0

################### blackman window ################################ 
blackman(f,B) = (0.42 + 0.5*cos((2π*f)/B) + 0.08*cos((4π*f)/B))*rect(f/B)

#################### Chirp ########################################## 
chirp(t) = cos( 2*pi*(f0*(t) + 0.5*K*(t)^2) ) * rect(t/T) * sqrt(Complex(blackman(t,T)))

	
################## Matched Filter #####################################

function matchedFilter(V_TX,v_rx)
    V_RX = fft(v_rx) # Fourier transform received signal
    H = conj(V_TX) 
    V_MF = H.*V_RX     # Apply matched filter in frequency domain
   # v_mf = ifft(V_MF)  # Go back to time domain
end

################  Analytic signal ###################################
function analytic(V_IN)
   # V_IN = fft(v_mf)
    V_ANALYTIC = 2*V_IN;
    N1= length(V_IN)

    if mod(N1,2)==0
        neg_freq_range1 = Int(N1/2):N1
    else
        neg_freq_range1 = Int((N1+1)/2):N1
    end

    V_ANALYTIC[neg_freq_range1] .= 0
    v_analytic = ifft(V_ANALYTIC)    
end

##################### Inverse Filter ############################
function inverse(v_rx, V_TX,f)
    H = (1 ./ V_TX).*rect((f.-f0)/(B*1.0))
    V_RX = fft(v_rx)
    V_INVERSE = (V_RX.*H)
   # v_inverse = ifft(V_INVERSE)
end

#################### Windowing ######################################
function window(v,f)
    #b_axis = -3*B/2:df:3*B/2
    V = fft(v)
    w = blackman.(f,B) + blackman.(f.-fs,B)
    v_window = V.*w   
end

######################## baseband ###############################
function baseband(v_window,t)
    j=im
    
    v_baseband = v_window .* exp.(-j*2*pi*f0*t) #change V_ANALYTIC1 to v_window1
    V_baseband = fft(v_baseband)
end

############### calculate dist ############

function calc_td(target,receiver)
    r = target-receiver
    R = sqrt(sum(r.*r))
    if(R== 2.0007838463962067)
        print("true")
    end
    td = 2*R/c
    return td
end


############################# global variables #####################################

z=""
l = "" #replace z
d =""
m= "" #replace d

function signalProcessing()
c = 343 # speed of sound in air
r_max = 10

fc=40000;   # center freq of chirp
T=5E-3      # chirp pulse length
B = 4000    # chirp bandwidth
K = B/T     # chirp rate [Hz/s]
f0 = fc-B/2
 #fs = 4*(1.6*B)
fs = 500000#400000#250000 #100000 400000
λ = c/fc #waveform
    println("Emptying buffer")
    h = readavailable(sp) 
    println("Number of bytes emptied from buffer ",h)
    
    println("Initiating chirp pulse")
    println("Sending c command to teensy")
    write(sp, "c") #writing to serial 
    sleep(0.2)
    
    s = readavailable(sp)
    if(s != "0")
        println("Unexpected response from teensy: Restart code")
        println("The value of unexpected s is ",s)
    end
    
    println("Received from c ", s)
    sleep(1) #delay
    
    println("Reading from the buffer by sending p command to teensy")
    write(sp, "p") #writing to serial
    global count=0
    global n=0
    global z= 0
    
    while true
    
	x=readavailable(sp)
	if (length(x) == 0)
	   sleep(0.0001)
	   global n = n+1
	else
	   global z = string(z, x)#append to a string
	   global n = 0
	   global count = count+1
	end

    	if n>1000
 	   break
    	end
       
    end
    
    println("count = ",count)
    println("Number of bytes received from the teensy ",length(z))
    Z = split(z, "\r\n")

    #println(Z)
    len = length(Z)-1;
    v = Vector{Int64}(undef,len)

    for n=1:len
    	#if n<4000
    	#	v[n] = 33170
    	#else
		v[n] = parse(Int64, Z[n])
	#end	
    end
    println("Number of ADC samples ",length(v))
    
    println("Emptying buffer for the 2nd time")
    h1 = readavailable(sp) 
    println("Number of bytes emptied from buffer on the 2nd time ",h1)
    
    println("Reading from the buffer by sending z command to teensy")
    write(sp, "z") #writing to serial
    global count1=0
    global n1=0
    global l =0

    while true
    
	x1=readavailable(sp)
	if (length(x1) == 0)
	   sleep(0.0001)
	   global n1 = n1+1
	else
	   global l = string(l, x1) #append to array
	   global n1 = 0
	   global count1 = count1+1
	end

    	if n1>1000
	  break
    	end
       
    end


    println("count = ",count1)
    println("Number of bytes received from the teensy ",length(l))
    L = split(l, "\r\n") # making an array of values

    len1 = length(L)-1;
    v2 = Vector{Int64}(undef,len1)
    for k=1:len1
    	v2[k] = parse(Int64, L[k])
    end
    println("Number of ADC samples from the second receiver is ",length(v2))
   # v_fill = Vector{Int64}(undef,4000) #fill(1.67, (1,4000))
    v = v[2500:end-100] #ignore the first 50 samples and last sample    
    v =  (v .* 3.3) / 65535 
    v = v .- 1.67
    
    v= v .- mean(v)
    nl = zeros(2450)	
    append!(nl,v)
    v= nl
    #v = [v_fill; v];
     v2 = v2[2500:end-100] #ignore the first 50 samples and last sample    
    v2 =  (v2 .* 3.3) / 65535 
    v2 = v2 .- 1.67
    
    v2= v2 .- mean(v2)
    nl2 = zeros(2450)	
    append!(nl2,v2)
    v2= nl2

        numSamples = length(v)
	Δt = 1/fs
    	t_max = Δt*(numSamples-1)
    	t = 0:Δt:t_max
    	N=length(t)
    	
  
	# distance axis
    	s = 0.5 * t * c
    	s_max = 0.5 * t_max * c
  	#println(s)
    	
    	#init frequency axis
    	Δω = 2*pi/(N*Δt)   # Sample spacing in freq domain in rad/s
    	Δf = Δω/(2*pi)
    	ω = 0:Δω:(N-1)*Δω
    	f = ω/(2*π)

	#create array of freq values stored in f_axis. First element maps to 0Hz
    	if mod(N,2)==0    # case N even
        	f_axis = (-N/2:N/2-1)*Δf;
    	else   # case N odd
        	f_axis = (-(N-1)/2 : (N-1)/2)*Δf;
    	end



       ########### Chirp pulse #####################
	v_tx =  chirp.(t)
	V_TX = fft(v_tx)
	
	
	#display(plot(real(v_tx)))
	println()
	println("Signal processing array 1")
	
	V_RX = fft(v)
	
	############### matched/inverse filter ###################
	println("Inverse Filter")
	#println("Matched filter")
	#v_m = matchedFilter(V_TX,v)
	v_i=inverse(v, V_TX,f)
	v_i2=inverse(v2, V_TX,f)
	############### analytic signal #################
	
	println("Analytic")
	v_an = analytic(v_i)
	v_an2 = analytic(v_i2)

	##################### Baseband ################
	println("Baseband")
	v_bb = baseband(v_an,t)
	v_bb = ifft(v_bb)
	
	v_bb2 = baseband(v_an2,t)
	v_bb2 = ifft(v_bb2)
	###################### window #################
	
	println("window")
	v_w = window(v_bb,f)
	
	v_w2 = window(v_bb2,f)
	
	#v_fill = Vector{ComplexF64}(undef, 5000)#fill(0.1, (1,5000))#Vector{Int64}(undef,5000)	
	#display(plot(v))
	#display(plot(v2))
        v_b = ifft(v_w)
        v_b2 = ifft(v_w2)
        #v_out = [v_fill;(v_b)]
        
        #append!(v_fill, v_b)
	display(plot(s, v, label="Received 1"))
	display(plot(s, v2, label="Received 2"))
	#display(plot( v2))
	#display(plot(s,real.(v_i), label="Inverse filter-frequency domain"))
	#display(plot(s,abs.(v_an), label="v_analytic - time domain"))
	#display(plot(real(fft(v_an)), label="v_analytic - freq domain"))
	#display(plot(real.(v_in), label="v_inverse - freq"))
	display(plot(s,abs.(v_bb),label="v_baseband 1- time domain"))
	display(plot(s,abs.(v_bb2),label="v_baseband 2- time domain"))
	display(plot(s,abs.(v_b),label="output 1- time domain"))
	display(plot(s,abs.(v_b2),label="output 2- time domain"))
	display(plot(angle.(v_b),label="Angle of baseband 1"))
	display(plot(angle.(v_b2),label="Angle of baseband 2"))
	#display(plot(s,abs.(v_b2),label="output - time domain"))
	#display(plot(s,(v_fill),label="output - time domain"))
	#println(typeof(v_b))
	#	println(typeof(v_fill))
	
	
	
	
	
	#=################ image ###############
	c = 343;
	fs = 500000; # sample rate of sonar, 44100 original 100 000
	dt = 1/fs; # sample spacing
	TargetXRange=1 #2
	global k =0
	for i in -1:0.005:1#TargetXRange
     		global k= k+1
	end

	TargetYRange=6
	global	j =0
	for s in 1:0.005:TargetYRange
	     global j= j+1
	end
	
	numOfReceivers = 2
	ImageArray = zeros(Complex, k,j)
	#reshape(Complex.(ImageArray),k,j)

	receiverSpacing = 0.016
	x1 = -((numOfReceivers-1)/2*receiverSpacing)
	indexOfReceived =0
	##################### receiver 1 ###########################################
	#for n in 1:numOfReceivers
    		xn = x1+(1-1)*receiverSpacing
    		receiver_position = [-0.008, 0.0, 0.0]
    		r_tx = v#d[indexOfReceived].pulse
    		indexOfReceived = indexOfReceived+1
    		lengthOfReceived = length(v)
   		x1=1
    		y1=1
    
    		for x in -TargetXRange/2:0.005:TargetXRange/2
      	  		for y in 1:0.005:6
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
    	#end
    	
    	
    	######################## receiver 2 #############################
    		xn = x1+(1-1)*receiverSpacing
    		receiver_position = [0.008, 0.0, 0.0]
    		r_tx = v2#d[indexOfReceived].pulse
    		indexOfReceived = indexOfReceived+1
    		lengthOfReceived = length(v2)
   		x1=1
    		y1=1
    
    		for x in -TargetXRange/2:0.005:TargetXRange/2
      	  		for y in 1:0.005:6
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

#might not work
    	
    	return ImageArray=#
end


signalProcessing()

signalProcessing() 
