using SerialPorts
#using PyPlot
using Plots
plotly() 
using FFTW

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
fs = 400000#250000 #100000 400000
λ = c/fc #waveform

####################### Rect ####################################
rect(t) = (abs.(t) .<= 0.5)*1.0

################### blackman window ################################ 

window(w) = ( abs.(w).<0.5 ) * 1.0
blackman(f,B) = (0.42 + 0.5*cos((2π*f)/B) + 0.08*cos((4π*f)/B))*rect(f/B)

#################### Chirp ########################################## 
chirp(t) = cos( 2*pi*(f0*(t) + 0.5*K*(t)^2) ) * rect(t/T) #* sqrt(Complex(blackman(t,T)))


#calculate required shift for basebanding
function calc_bb_shift(f_c,f_s)
    if(f_c < f_s/2)
        return f_c
    end
    if(f_c < f_s)
        return f_s - f_c
    end
    temp = f_c%f_s
    if(temp > fs/2)
        temp = abs(temp-fs)
    end

    return temp

end

########## serial communication ########################
z=""
l = "" #replace z
d =""
m= "" #replace d

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
v1 = Vector{Int64}(undef,len)

for n=1:len
    v1[n] = parse(Int64, Z[n])	
end

println("Number of ADC samples ",length(v1))


v1 = v1[2500:end-100] #ignore the first 50 samples and last sample    
v1 =  (v1 .* 3.3) / 65535 
v1 = v1 .- 1.67
nl = zeros(2500)
append!(nl,v1)	
v1= nl




numSamples = length(v1)
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


# Define chirp pulse
v_tx = rect((t .- T/2)/T).*cos.(2*pi*(f0*t+0.5*K*t.^2))

V_TX = fft(v_tx)


#prepare recieved signal

HPF =  window((f .- fc)/(2*B)) .+ window((f.- (N*Δf-fc))/(2*B))

#apply BPF to signal 1
v1_prefilter = v1;
V1 = fft(v1)
V1 = V1.* HPF
v1 = real(ifft(V1))

#deadtime comp
t_deadtime = 0.0065 #0.0055
s_deadtime = 2.2295
    
#INVERSE FILTER
wind = window((f .-fc % (N * Δf))/B)

Y1 = V1./V_TX .* (wind .+ wind[end:-1:1]) # to account for the ='ve and -'ve freq
Y1[isnan.(Y1)] .= 0 #replace any Nan with 0 (arrises from zero-padding)    

 #Output of filter
y1 = ifft( Y1 ) # go back to time domain.

 #Create analytic signal
    Y1_an = 2*Y1
    for i = 1:length(Y1)
        if i > length(Y1)/2
            Y1_an[i] =0
        end
    end
    
  y1_an = ifft(Y1_an)
    
   H_window = window((f.-fc)/(1.5*B))  .* cos.(pi*(f.+700)/(0.4*N)) .^2

    Y1_window = Y1_an .* H_window
    y1_window = ifft(Y1_window)


#Basebanding
    #shift_to_bb = min(-fc + ceil(fc/fs)*fs, fc*1) # to account for when sampled below nyquist rate
    shift_to_bb = calc_bb_shift(fc*1.01,fs) # multiply by 1% for correction
    y1_bb = y1_window .* exp.(-im*2*pi*shift_to_bb*t) #need the % f_highest/2 for the case when sampling = 4B
    Y1_bb = fft(y1_bb)
    
        bb_zoom_upper = convert(Int,round(length(s)/8))
    bb_zoom_lower = convert(Int,round(length(s)/12))
    
    #account for dead time by moving the last bit of the array to the beginning
    cut_index = Int(round(N-(t_deadtime/t_max * N)))
    y1_out_comp = vcat(y1_bb[cut_index:end-1],y1_bb[1:cut_index])
    
    display(plot(abs.(y1_out_comp)))

    
    
