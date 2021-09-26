using Pkg
using Plots
plotly()
using FFTW
using JLD2
using FileIO
using OrderedCollections

#Rect
rect(t) = (abs.(t) .<= 0.5)*1.0

#blackman 
blackman(f,B) = (0.42 + 0.5*cos((2π*f)/B) + 0.08*cos((4π*f)/B))*rect(f/B)

#Matched Filter

function matchedFilter(h,v)
    V_RX = fft(v) # Fourier transform received signal
    V_MF = h.*V_RX     # Apply matched filter in frequency domain
    v_mf = ifft(V_MF)  # Go back to time domain
    return v_mf
end

#Analytic signal
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
    return V_ANALYTIC
end

#Inverse Filter
function inverse(v_rx, V_TX)
    H = (1 ./ V_TX).*rect((f.-f0)/(B*1.0))
    V_RX = fft(v_rx)
    V_INVERSE = (V_RX.*H)
    v_inverse = ifft(V_INVERSE)
    return v_inverse
end

#Windowing
function window(v)
    #b_axis = -3*B/2:df:3*B/2
    v = ifft(v)
    w = blackman.(f,B) + blackman.(f.-fs,B)
    v_window = v.*w   
    return v_window
end

#baseband
function baseband(v_window)
    j=im
    v_baseband = v_window .* exp.(-j*2*pi*f0*t) #change V_ANALYTIC1 to v_window1
    return v_baseband
end

function signalProcessing()
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

	N=length(t)
	dw = 2*pi/(N*dt)
	df = dw/(2*pi)
	w = 0:dw:(N-1)*dw
	f = w/(2*π)
	fc = 40000;
	
	#list_ports()
	#send chirp command to Teensy
	println("connecting to port")
	sp=open("/dev/cu.usbmodem58714801",9600) # Or whatever in Linux, Windows or Mac. //38400
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
	v1=zeros(Int32, 17006) # Create an Uint8 array into which to read the

	n = 1;
	#print("BytesAvailableAtStart: ",bytesavailable(sp))
	println("reading array 1")
	while(bytesavailable(sp) > 0) #divide this by 4?? 4 bytes in a float?
        	v1[n] = parse(Int32, (readline(sp)));
         	#println(v[n]);
         	n = n+1;

    	end
	
end
