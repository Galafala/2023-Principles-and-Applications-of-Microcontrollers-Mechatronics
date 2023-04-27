import random
import numpy as np

y = list(np.arange(0,31,1))
x = random.sample(y, 15)
x.sort()
z = []
for i in y:
    if i in x:
        continue
    z.append(i)
print(x, z)