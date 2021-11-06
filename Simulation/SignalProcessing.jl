using Pkg
using Plots
plotly()
using FFTW
using JLD2
using FileIO
using OrderedCollections

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

N=length(t)
dw = 2*pi/(N*dt)
df = dw/(2*pi)
w = 0:dw:(N-1)*dw
f = w/(2*π)
fc = 40000;

#Rect
rect(t) = (abs.(t) .<= 0.5)*1.0

#blackman 
blackman(f,B) = (0.42 + 0.5*cos((2π*f)/B) + 0.08*cos((4π*f)/B))*rect(f/B)
#Matched Filter

function matchedFilter(h,v)
    V_RX = fft(v) # Fourier transform received signal
    H = conj(h) 
    V_MF = H.*V_RX     # Apply matched filter in frequency domain
    v_mf = ifft(V_MF)  # Go back to time domain
end

#Analytic signal
function analytic(V_IN)
  #  V_IN = fft(v_mf)
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

#Inverse Filter
function inverse(v_rx, V_TX)
    H = (1 ./ V_TX).*rect((f.-f0)/(B*1.0))
    V_RX = fft(v_rx)
    V_INVERSE = (V_RX.*H)
   # v_inverse = ifft(V_INVERSE)
end

#Windowing
function window(v)
    #b_axis = -3*B/2:df:3*B/2
    V = fft(v)
    w = blackman.(f,B) + blackman.(f.-fs,B)
    v_window = V.*w   
end

#baseband
function baseband(v_window)
    j=im
    v_baseband = v_window .* exp.(-j*2*pi*f0*t) #change V_ANALYTIC1 to v_window1
    V_baseband = fft(v_baseband)
end

d = load("chirps.jld2","chirps")

v_tx = d["transmit"]
V_TX = fft(v_tx);
ImagingDict = OrderedDict{Any,Any}("transmit" => v_tx)
d = delete!(d,"transmit")

count = 1


#plt = plot(title="MyPlot", layout=2)
for (key,value) in d
    v_rx= value.pulse#value.pulse# d[0].pulse
   # V_M=matchedFilter(v_rx, V_TX)
   # v_m = ifft(V_M)
    v_in = inverse(v_rx, V_TX)
    v_an=analytic(v_in)
    v_bb = baseband(v_an)
    v_bb = ifft(v_bb)
    v_w = window(v_bb)
    v_b = ifft(v_w)
    
    #labels for the plots
    #base = string("Absolute baseband :",key)
    #log = string("Log of baseband :",key)
    #b_angle = string("Angle of baseband :",key)
    
    #Plotting of baseband
    #display(plot(r,real(v_w)))
   #= display(plot((r,v_rx), label="V_rx"))
    display(plot(real.(v_in), label="Inverse filter in the frequency domain"))
    #display(plot(r,abs.(ifft(v_in), label="Inverse filter in the time domain")))
    display(plot(r,real.(v_an),label="Analytic signalin the time domain"))
    display(plot(r,abs.((v_bb)), label="basebanded signal in the time domain"))
   # display(plot(r,abs.(ifft(v_bb)), label="basebanded signal in the time domain"))
    display(plot(abs.((v_w)), label="Windowed signal in the frequency domain"))
   # plt = plot!(r,abs.(v_b), label="output in the time domain")
    display(plot(r,abs.(v_b), label="output in the time domain"))
    display(plot(20*log10.(abs.(v_bb)), label="Log of baseband"))
    display(plot(angle.(v_b), label="Angle of baseband"))
    =#
    receivingStructElement = Receivers(value.coordinates,v_b) 
    pulse_label = string("Baseband : ",key)
    push!(ImagingDict,key => receivingStructElement)
end
#display(plt)


save("BasebandedSignal.jld2", "baseband",ImagingDict)




