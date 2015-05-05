import math
import numpy as np

def f(x, u, k, fprev):
    if k == 1:
        return 0.0
    else:
        return ((x-u)**2)/(k-1) + (float(k-1)/k)*fprev


a = np.random.rand(1)
fa = np.var(a)

fprev = 0.0
k=1
meanprev = 0.0
for ak in a:
    currmean = (meanprev*(k-1)+ak)/k
    currf = f(ak, currmean, k, fprev)
    meanprev = currmean
    fprev = currf
    k+=1



farec = fprev
print 'Variance of A: Recursive=', farec, 'Numpy=', fa






