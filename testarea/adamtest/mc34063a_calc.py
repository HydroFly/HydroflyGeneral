Vin = 12.0 #input voltage of a 3Cell lipo battery in our case
Vout = 5.0 #output voltage we want for Raspberry PI
Vsat = 1.0 #saturation voltage, based on pg3 of datasheet, darlington connection
f = 100000 #100kHz based on website suggestion
VF = 1.1 #voltage drop of IN001 diode, based on datasheet
Iout = 2.5 #raspberry pi
Vripple = 0.1 #based on website

T_ONtoT_OFF = (Vout + VF)/(Vin - Vsat - Vout) #Vin(min)
T_ONplusT_OFF = 1/f
T_OFF = T_ONplusT_OFF/(T_ONtoT_OFF + 1)
T_ON = (T_ONplusT_OFF) - T_OFF
CT = (4.0*10**(-5))*T_ON
Ipk = 2*Iout#Tpkswitch. Ioutmax
Rsc = 0.3/Ipk 
Lmin = ((Vin - Vsat - Vout)/Ipk)*T_ON #T_ON max?
CO = Ipk*(T_ONplusT_OFF)/(8*Vripple) #Vripple(pp)

print("Capacitor for pin 3 (pF): ", CT)
print("Sense current Resistor (ohms): ", Rsc)
print("Minimum value of inductor (uH): ", Lmin)
print("Output Capacitor (uF): ", CO)

R2_to_R1 = (Vout/1.25)-1
print("R2:R1 : ", R2_to_R1)

