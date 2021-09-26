using SerialPorts
list_serialports() # show available ports
sp = SerialPort("/dev/ttyACM0", 9600) # Linux example to open a port
#sp = SerialPort("COM4:", 9600) # On windows try "COM4:"
write(sp, "Hello") # write a string to the port (or use a binary data type)
s = readavailable(sp) # read from the port (s is now of type String)
println(s)
x = Vector{UInt8}(s) # Convert string to an array of Uint8 integers.
println(x)
close(sp) # close the port
