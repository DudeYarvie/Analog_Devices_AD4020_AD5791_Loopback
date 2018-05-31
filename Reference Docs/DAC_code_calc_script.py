Vrefn =-10
Vrefp = 10
Vo = 3
dac_bit_size = 20

D = int(((Vo - Vrefn)*(2**dac_bit_size - 1))/float(Vrefp - Vrefn))
D_binary = '{0:08b}'.format(D)
print ("Dac code = %d (0b%s)" % (D, D_binary))
