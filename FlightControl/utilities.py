def volt_to_pressure(voltage):
    """ converts voltage from ADC to PSIA value"""
    maxPress = 2500.0
    minPress = 0.0
    vSource = 5.0 #voltage source to ADC
    vSupplyPress = 5.0 #voltage supply to pressure sensor

    psi = (maxPress-minPress)*voltage/.8/vSource - (maxPress-minPress)/8 + minPress
    return psi


def val_to_volt(value, gain):
    """ converts value returned from ADC into an actual voltage depending on gain setting"""
    maxNum = 32768
    v = ((4.096/gain) / float(maxNum)) * value
    return v
