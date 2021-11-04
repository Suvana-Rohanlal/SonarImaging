using SerialPorts
#using PyPlot
using Plots
plotly() 
using FFTW
using Statistics
using JLD2
using FileIO
using OrderedCollections

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
fs = 500000#250000 #100000 400000
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
    v_mf = ifft(V_MF)  # Go back to time domain
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
  ############################ V1 #############################################  
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
	v[n] = parse(Int64, Z[n])
    end
    println("Number of ADC samples ",length(v))
    
    println("Emptying buffer for the 2nd time")
    h1 = readavailable(sp) 
    println("Number of bytes emptied from buffer on the 2nd time ",h1)
  ################################ v2 #######################################  
    println("Reading from the buffer by sending x command to teensy")
    write(sp, "x") #writing to serial
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
    L2 = split(l, "\r\n") # making an array of values

    len1 = length(L2)-1;
    v2 = Vector{Int64}(undef,len1)
    for k=1:len1
    	v2[k] = parse(Int64, L2[k])
    end
    println("Number of ADC samples from the third receiver is ",length(v2))
    

    
    ################## retransmit #################################
     println("Emptying buffer")
    h = readavailable(sp) 
    println("Number of bytes emptied from buffer ",h)
    
    println("Initiating chirp pulse")
    println("Sending d command to teensy")
    write(sp, "d") #writing to serial 
    sleep(0.2)
    
    s = readavailable(sp)
    if(s != "0000")
        println("Unexpected response from teensy: Restart code")
        println("The value of unexpected s is ",s)
    end
    
    println("Received from c ", s)
    sleep(1) #delay
    
 
      ############################ V3 #############################################  
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
    v3 = Vector{Int64}(undef,len)

    for n=1:len
	v3[n] = parse(Int64, Z[n])
    end
    println("Number of ADC samples ",length(v3))
    
    println("Emptying buffer for the 2nd time")
    h1 = readavailable(sp) 
    println("Number of bytes emptied from buffer on the 2nd time ",h1)
  ################################ v4 #######################################  
    println("Reading from the buffer by sending x command to teensy")
    write(sp, "x") #writing to serial
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
    L2 = split(l, "\r\n") # making an array of values

    len1 = length(L2)-1;
    v4 = Vector{Int64}(undef,len1)
    for k=1:len1
    	v4[k] = parse(Int64, L2[k])
    end
    println("Number of ADC samples from the third receiver is ",length(v4))
    
     ################## retransmit #################################
     println("Emptying buffer")
    h = readavailable(sp) 
    println("Number of bytes emptied from buffer ",h)
    
    println("Initiating chirp pulse")
    println("Sending d command to teensy")
    write(sp, "e") #writing to serial 
    sleep(0.2)
    
    s = readavailable(sp)
    if(s != "0")
        println("Unexpected response from teensy: Restart code")
        println("The value of unexpected s is ",s)
    end
    
    println("Received from e ", s)
    sleep(1) #delay
    
 
      ############################ V5 #############################################  
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
    v5 = Vector{Int64}(undef,len)

    for n=1:len
	v5[n] = parse(Int64, Z[n])
    end
    println("Number of ADC samples ",length(v5))
    
    println("Emptying buffer for the 2nd time")
    h1 = readavailable(sp) 
    println("Number of bytes emptied from buffer on the 2nd time ",h1)
  ################################ v6 #######################################  
    println("Reading from the buffer by sending x command to teensy")
    write(sp, "x") #writing to serial
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
    L2 = split(l, "\r\n") # making an array of values

    len1 = length(L2)-1;
    v6 = Vector{Int64}(undef,len1)
    for k=1:len1
    	v6[k] = parse(Int64, L2[k])
    end
    println("Number of ADC samples from the third receiver is ",length(v6))
    
     ################## retransmit #################################
     println("Emptying buffer")
    h = readavailable(sp) 
    println("Number of bytes emptied from buffer ",h)
    
    println("Initiating chirp pulse")
    println("Sending d command to teensy")
    write(sp, "f") #writing to serial 
    sleep(0.2)
    
    s = readavailable(sp)
    if(s != "0")
        println("Unexpected response from teensy: Restart code")
        println("The value of unexpected s is ",s)
    end
    
    println("Received from f ", s)
    sleep(1) #delay
    
 
      ############################ V7 #############################################  
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
    v7 = Vector{Int64}(undef,len)

    for n=1:len
	v7[n] = parse(Int64, Z[n])
    end
    println("Number of ADC samples ",length(v7))
    
    println("Emptying buffer for the 2nd time")
    h1 = readavailable(sp) 
    println("Number of bytes emptied from buffer on the 2nd time ",h1)
  ################################ v8 #######################################  
    println("Reading from the buffer by sending x command to teensy")
    write(sp, "x") #writing to serial
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
    L2 = split(l, "\r\n") # making an array of values

    len1 = length(L2)-1;
    v8 = Vector{Int64}(undef,len1)
    for k=1:len1
    	v8[k] = parse(Int64, L2[k])
    end
    println("Number of ADC samples from the third receiver is ",length(v8))
    
   #=     println("Reading from the buffer by sending z command to teensy")
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
    
     println("Emptying buffer for the 2nd time")
    h1 = readavailable(sp) 
    println("Number of bytes emptied from buffer on the 2nd time ",h1)
    
    ################################ V4 #####################################
        println("Reading from the buffer by sending s command to teensy")
    write(sp, "s") #writing to serial
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
    L3 = split(l, "\r\n") # making an array of values

    len1 = length(L3)-1;
    v4 = Vector{Int64}(undef,len1)
    for k=1:len1
    	v4[k] = parse(Int64, L3[k])
    end
    println("Number of ADC samples from the fourth receiver is ",length(v4))
    
     println("Emptying buffer for the 5th time")
    h1 = readavailable(sp) 
    println("Number of bytes emptied from buffer on the 5th time ",h1)
    
=#
    ##############################################################################
  #=  display(plot(v))
    display(plot(v2))
    display(plot(v3))
    display(plot(v4))=#

    #remove first few samples
   	v = v[2500:end-100] #ignore the first 50 samples and last sample    
   	v =  (v .* 3.3) / 65535 
    	v = v .- 1.67
    
    	v= v .- mean(v)
    	nl = zeros(2450)	
    	append!(nl,v)
    	v= nl
    	###########
    	v2 = v2[2500:end-100] #ignore the first 50 samples and last sample    
   	v2 =  (v2 .* 3.3) / 65535 
    	v2 = v2 .- 1.67
    
    	v2= v2 .- mean(v2)
    	nl2 = zeros(2450)	
    	append!(nl2,v2)
    	v2= nl2
    	###########
    	v3 = v3[2500:end-100] #ignore the first 50 samples and last sample    
   	v3 =  (v3 .* 3.3) / 65535 
    	v3 = v3 .- 1.67
    
    	v3= v3 .- mean(v3)
    	nl3 = zeros(2450)	
    	append!(nl3,v3)
    	v3= nl3
    	###########
    	v4 = v4[2500:end-100] #ignore the first 50 samples and last sample    
   	v4 =  (v4 .* 3.3) / 65535 
    	v4 = v4 .- 1.67
    
    	v4= v4 .- mean(v4)
    	nl4 = zeros(2450)	
    	append!(nl4,v4)
    	v4= nl4
    	
    	#########
    	v5 = v5[2500:end-100] #ignore the first 50 samples and last sample    
   	v5 =  (v5 .* 3.3) / 65535 
    	v5 = v5 .- 1.67
    
    	v5= v5 .- mean(v5)
    	nl5 = zeros(2450)	
    	append!(nl5,v5)
    	v5= nl5
    	###########
    	v6 = v6[2500:end-100] #ignore the first 50 samples and last sample    
   	v6 =  (v6 .* 3.3) / 65535 
    	v6 = v6 .- 1.67
    
    	v6= v6 .- mean(v6)
    	nl6 = zeros(2450)	
    	append!(nl6,v6)
    	v6= nl6
    	###########
    	v7 = v7[2500:end-100] #ignore the first 50 samples and last sample    
   	v7 =  (v7 .* 3.3) / 65535 
    	v7 = v7 .- 1.67
    
    	v7= v7 .- mean(v7)
    	nl7 = zeros(2450)	
    	append!(nl7,v7)
    	v7= nl7
    	###########
    	v8 = v8[2500:end-100] #ignore the first 50 samples and last sample    
   	v8 =  (v8 .* 3.3) / 65535 
    	v8 = v8 .- 1.67
    
    	v8= v8 .- mean(v8)
    	nl8 = zeros(2450)	
    	append!(nl8,v8)
    	v8= nl8
    	
    	
    	######### new constants #########################################################
    	
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
	#v_m = matchedFilter(V_TX,v1)
	v_m=inverse(v, V_TX,f)
	v_m2=inverse(v2, V_TX,f)
	v_m3=inverse(v3, V_TX,f)
	v_m4=inverse(v4, V_TX,f)
	############### analytic signal #################
	
	println("Analytic")
	v_an = analytic(v_m)
	v_an2 = analytic(v_m2)
	v_an3 = analytic(v_m3)
	v_an4 = analytic(v_m4)


	##################### Baseband ################
	println("Baseband")
	v_bb = baseband(v_an,t)
	v_bb = ifft(v_bb)
	v_bb2 = baseband(v_an2,t)
	v_bb2 = ifft(v_bb2)
	v_bb3 = baseband(v_an3,t)
	v_bb3 = ifft(v_bb3)
	v_bb4 = baseband(v_an4,t)
	v_bb4 = ifft(v_bb4)
	###################### window #################
	
	println("window")
	v_w = window(v_bb,f)
	v_w2 = window(v_bb2,f)
	v_w3 = window(v_bb3,f)
	v_w4 = window(v_bb4,f)
		

        v_b = ifft(v_w)
        v_b2 = ifft(v_w2)
        v_b3 = ifft(v_w3)
        v_b4 = ifft(v_w4) 
        
            
	display(plot(s, v))
	display(plot(s, v2))
	display(plot(s, v3))
	display(plot(s, v4))
	display(plot(s, v5))
	display(plot(s, v6))
	display(plot(s, v7))
	display(plot(s, v8	))
	#display(plot(s,real.(v_m), label="Inverse filter-freq domain"))
	#display(plot(s,abs.(v_an), label="v_analytic - time domain"))
	#display(plot(ifft(abs.(v_an)), label="v_analytic - time domain"))
	#display(plot(real.(v_in), label="v_inverse - freq"))
	#display(plot(s,abs.(v_bb),label="v_baseband - time domain"))
	#display(plot(s,abs.(v_b),label="output - time domain"))
	
	
	ImagingDict = OrderedDict{Any,Any}("transmit" => v_tx)
	push!(ImagingDict,1 => v)
	push!(ImagingDict,2 => v2)
	push!(ImagingDict,3 => v3)
	push!(ImagingDict,4 => v4)
	
	save("BasebandedSignal.jld2", "baseband",ImagingDict)
	#=arrRec[4]
	arrRec[1] = v
	arrRec[2] = v2
	arrRec[3] = v3
	arrRec[4] = v4
	
	return arrRec
	=#
    	
    

end

signalProcessing()
