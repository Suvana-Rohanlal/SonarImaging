using SerialPorts
#using PyPlot
using Plots
plotly() 
using FFTW


list_serialports() 
sp = SerialPort("/dev/ttyACM0", 9600) 

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
    if(s != "0000")
        println("Unexpected response from teensy: Restart code")
        println("The value of unexpected s is ",s)
    end
    
    println("Received from c ", s)
    sleep(1) #delay
    
     #println("Reading from the buffer by sending p command to teensy")
    #write(sp, "p") #writing to serial
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
    
    sleep(0.1)
    
    global count1=0
    global n1=0
    global z1= 0
    
    while true
    
	x1=readavailable(sp)
	if (length(x1) == 0)
	   sleep(0.0001)
	   global n1 = n1+1
	else
	   global z1 = string(z1, x1)#append to a string
	   global n1 = 0
	   global count1 = count1+1
	end

    	if n1>1000
 	   break
    	end
       
    end
    
     sleep(0.1)
    
    global count2=0
    global n2=0
    global z2= 0
    
    while true
    
	x2=readavailable(sp)
	if (length(x2) == 0)
	   sleep(0.0001)
	   global n2 = n2+1
	else
	   global z2 = string(z2, x2)#append to a string
	   global n2 = 0
	   global count2 = count2+1
	end

    	if n2>1000
 	   break
    	end
       
    end
    
      sleep(0.1)
    
    global count3=0
    global n3=0
    global z3= 0
    
    while true
    
	x3=readavailable(sp)
	if (length(x3) == 0)
	   sleep(0.0001)
	   global n3 = n3+1
	else
	   global z3 = string(z3, x3)#append to a string
	   global n3 = 0
	   global count3 = count3+1
	end

    	if n3>1000
 	   break
    	end
       
    end
    
    ############## end of reading string ##########################
    
    ############## ADC samples reading ###############
    
    println("count = ",count)
    println("Number of bytes received from the teensy ",length(z))
    Z = split(z, "\r\n")

    #println(Z)
    len = length(Z)-1;
    v = Vector{Int64}(undef,len)

    for n=1:len
	v[n] = parse(Int64, Z[n])
    end
    println("Number of ADC samples v = ",length(v))
    
     ####
    println("count1 = ",count1)
    println("Number of bytes received from the teensy ",length(z1))
    Z1 = split(z1, "\r\n")

    #println(Z)
    len1 = length(Z1)-1;
    v1 = Vector{Int64}(undef,len1)

    for n1=1:len1
	v1[n1] = parse(Int64, Z1[n1])
    end
    println("Number of ADC samples v1 = ",length(v1))
    
    
     ####
    println("count2 = ",count2)
    println("Number of bytes received from the teensy ",length(z2))
    Z2 = split(z2, "\r\n")

    #println(Z)
    len2 = length(Z2)-1;
    v2 = Vector{Int64}(undef,len2)

    for n2=1:len2
	v2[n2] = parse(Int64, Z2[n2])
    end
    println("Number of ADC samples v2 = ",length(v2))
    
     ####
    println("count3 = ",count3)
    println("Number of bytes received from the teensy ",length(z3))
    Z3 = split(z3, "\r\n")

    #println(Z)
    len3 = length(Z3)-1;
    v3 = Vector{Int64}(undef,len3)

    for n3=1:len3
	v3[n3] = parse(Int64, Z3[n3])
    end
    println("Number of ADC samples v3 = ",length(v3))
    
    
    println("END")
