using Plots
plotly()
using FFTW

function chirp()
	
	#Specify parameters of chirp pulse
	f0 = 39000; # Centre frequency is 40 kHz
	B = 2000; # Chirp bandwidth
	#f0 = fc-B/2; # Inital frequency
	T =  5E-3; # Chirp pulse length Change back to 5E-3 if nec
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
	R = c*T/2; # this gives you the closest distance that the object can be
	r = c*t/2;
	println("deadband =", R)
	# Define a simple a rect() function which returns 1 for -0.5<=t<=0.5 only
	rect(t) = (abs.(t) .<= 0.5)*1.0

	blackman(f,B) = abs.((0.42 + 0.5*cos((2π*f)/B) + 0.08*cos((4π*f)/B))*rect(f/B))

	chirp(t) = cos( 2*pi*(f0*(t) + 0.5*K*(t)^2) ) * rect(t/T) #* sqrt(blackman(t,T))

	# Delay the chirp pulse so that it starts after t=0.
	td = 0.6*T; # Chirp delay
	# Define shifted chirp pulse
	v_tx = chirp.(t.-td)
	v_n = zeros(Float64, length(v_tx))

	#print values of v_tx to a file
	#write header file 
	write_to_file_path = "./Chirp.h";
	N = length(v_tx);
	rangeMultiplier = (0.4*4095)/3.3#(desiredAmplitude*DACresolution)/refVoltage;  #works with 0.5
	 DACoffset = (2.5*4095)/3.3; ## works with 2.5
	output_file = open(write_to_file_path, "w+");
	println(output_file, "#ifndef _Chirp_h_");
	println(output_file, "#define _Chirp_h_");
	println(output_file, "#define maxWaveform 1");
	print(output_file, "#define maxSamplesNum ");
	println(output_file, N);
	println(output_file, "static float waveformsTable[maxWaveform][maxSamplesNum] = {");
	println(output_file, "{");
	
	print(output_file, (v_tx[1]*rangeMultiplier)+DACoffset);
	v_n[1] = (v_tx[1]*rangeMultiplier)+DACoffset
	for n=2:N;
		#write(output_file, v_tx[n]);
		#print(output_file, v_tx[n]);
		print(output_file, ", ");
		val = v_tx[n];
               out = val*rangeMultiplier; #1860 - range.amplitude of 1.5V, 1240 - range up to 1V, 2480 - range up to 2V 
              
               newOut = out + DACoffset; #add offset (1860 = 1.5V) (1240 = 1V)
		print(output_file, newOut);
		v_n[n] = newOut;
	end
	println(output_file, "}");
	println(output_file, "};");
	println(output_file, "#endif");


		display(plot(real(v_n)))
end

chirp()
