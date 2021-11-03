 #module twoChannels
using SerialPorts
#using PyPlot
using Plots
plotly() 
using FFTW
using Statistics


list_serialports() 
sp = SerialPort("/dev/ttyACM0", 9600) 
 while(true)
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
    
 end
