using Pkg
using Plots
plotly()
using FFTW
using JLD2
using FileIO
using OrderedCollections
using LibSerialPort


################# constants ############################
c = 343 # speed of sound in air
r_max = 10

fc=40000;   # center freq of chirp
T=5E-3      # chirp pulse length
B = 4000    # chirp bandwidth
K = B/T     # chirp rate [Hz/s]
f0 = fc-B/2
 #fs = 4*(1.6*B)
fs = 250000 #100000 400000
λ = c/fc #waveform

####################### Rect ####################################
rect(t) = (abs.(t) .<= 0.5)*1.0

################### blackman window ################################ 
blackman(f,B) = (0.42 + 0.5*cos((2π*f)/B) + 0.08*cos((4π*f)/B))*rect(f/B)

#################### Chirp ########################################## 
chirp(t) = cos( 2*pi*(f0*(t) + 0.5*K*(t)^2) ) * rect(t/T) #* sqrt(Complex(blackman(t,T)))

	
################## Matched Filter #####################################

function matchedFilter(h,v)
    V_RX = fft(v) # Fourier transform received signal
    H = conj(h) 
    V_MF = H.*V_RX     # Apply matched filter in frequency domain
    v_mf = ifft(V_MF)  # Go back to time domain
end

################  Analytic signal ###################################
function analytic(v_mf)
    V_IN = fft(v_mf)
    V_ANALYTIC = 2*V_IN;
    N1= length(V_IN)

    if mod(N1,2)==0
        neg_freq_range1 = Int(N1/2):N1
    else
        neg_freq_range1 = Int((N1+1)/2):N1
    end

    V_ANALYTIC[neg_freq_range1] .= 0
    V_ANALYTIC = ifft(V_ANALYTIC)    
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


############################# global variables #####################################
global z=""
global l = "" #replace z
global d =""
global m= "" #replace d

############################### main function ####################################################
function signalProcessing()
	println("Starting...")
	

   	#send chirp command to Teensy
    	println("connecting to port")
    	ports = get_port_list()
    	sp=open(ports[1],9600)#("/dev/cu.usbmodem58714801",9600) # Or whatever in Linux, Windows or Mac. //38400
    
    	#clear buffer
   	
    	while(bytesavailable(sp) > 0)
        readline(sp);
    end
    write(sp, "s\n") # Sends chirp (DAC)
    println("chirp sent")
    write(sp, "c\n") # This writes out the ASCII codes for H, e, l, l and o.
    write(sp, "p\n")

    sleep(0.5) # Give time for a response from the micro

    #read in signal from serial port here
    BytesAvailable = bytesavailable(sp) # Number of bytes available in the buffer
   # v1=zeros(Int32, 17006) # Create an Uint8 array into which to read the
v1 = fill(1.67, (1,17006))
    n = 1;
    #print("BytesAvailableAtStart: ",bytesavailable(sp))
    println("reading array 1")
    while(bytesavailable(sp) > 0) #divide this by 4?? 4 bytes in a float?
         v1[n] = parse(Int32, (readline(sp)))* 3.3/65535;
         #println(v[n]);
         n = n+1;

    end
	println("Reading 1 done")
	
	#different symbol for second buffer
    	write(sp, "a\n") #ask for second buffer
    	sleep(0.5) # Give time for a response from the micro
    
    	# Check if some data is now in the receive buffer:
    	BytesAvailable = bytesavailable(sp) # Number of bytes available in the buffer
    	
    	#println("Bytes available:",BytesAvailable)
    	v2= fill(1.67, (1,17006))#zeros(Int32, 17006)
    	n = 1;
    	#print("BytesAvailableAtStart: ",bytesavailable(sp))
    	println("reading array 2")
    	while(bytesavailable(sp) > 0) #divide this by 4?? 4 bytes in a float?
     	#try
        #global n
        	v2[n] = parse(Int32, (readline(sp)))* 3.3/65535;#parse(Int32, (readline(sp))); #Int16
         	n = n+1;

	end
	println("Reading 2 done")
	
	v1 = v1[10:end-10] #ignore the first 50 samples and last sample
   	#v1 =  (v1 .* 3.3) / 65535
   	v2 = v2[50:end-10] #ignore the first 50 samples and last sample

	#display(plot(real(v1)))
	#display(plot(real(v2)))
	
	numSamples = length(v1)
	Δt = 1/fs
    	t_max = Δt*(numSamples-1)
    	t = 0:Δt:t_max
    	N=length(t)
    	
    	
	# distance axis
    	s = 0.5 * t * c
    	s_max = 0.5 * t_max * c

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
	
	
	
	println()
	println("Signal processing array 1")
	############# Filter #########################
	#filter = rect.((f.-fc)/(2*B)) .+ rect.((f.- (N*Δf-fc))/(2*B))
	
	V_1 = fft(v1)
	#v_1 = V_1.*filter
	#v1 = (ifft(V_1))
	#display(plot(real(v1)))
	############### matched/inverse filter ###################
	println("Inverse Filter")
	#println("Matched filter")
	#v_m = matchedFilter(V_TX,v1)
	v_m=inverse(v1, V_TX,f)
	############### analytic signal #################
	
	println("Analytic")
	v_an = analytic(v_m)
	#V_IN = inverse(v_an, V_TX, f)
	#v_in = ifft(V_IN)
	##################### Baseband ################
	println("Baseband")
	v_bb = baseband(v_an,t)
	v_bb = ifft(v_bb)
	###################### window #################
	
	println("window")
	v_w = window(v_bb,f)
	
	
	v_b = ifft(v_w)
	
	display(plot(abs.(v_m), label="Matched filter-time domain"))
	display(plot(abs.(v_an), label="v_analytic - time domain"))
	#display(plot(real.(v_in), label="v_inverse - freq"))
	display(plot(abs.(v_bb),label="v_baseband - time domain"))
	display(plot(abs.(v_b),label="output - time domain"))
	
	#println("Inverse Filter")
	#v_in=inverse(v1, v_tx)
	
	
	
	#v_in = inverse(v_an1,v_tx)
	
	
	
	
	#v_in = ifft(v_in)
	
	#display(plot(real(ff(v1))))
	
	#display(plot(abs.(v_an)))
	
	#display(plot(abs.(v_bb)))
	
#	display(plot(abs.(v_b)))
	
	
		
end

signalProcessing()
