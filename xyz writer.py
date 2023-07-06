import numpy as np
import pickle
f1="flash.xyz"
flash=np.genfromtxt("flash.xyz",usecols=(0,1,2))
flash=flash.tolist()

print(flash[0][2])
with open("flash2.pts","w") as txt:
    for row in flash:
        txt.write(' '.join([str(item) for item in row]))
        txt.write('\n')
