import numpy as np
a = [0.108329, 0.170974, 0.066496, 0.101726, 0.104655, 0.105323]
scope = np.asarray(a)/np.linalg.norm(a)
r = np.round(scope*len(a), 0)
labels=[]
l = np.unique(r).tolist()
for i in r:
    labels.append(l.index(i))
print(labels)