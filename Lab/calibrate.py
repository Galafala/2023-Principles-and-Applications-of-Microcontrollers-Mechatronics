"""
the code for estimating the constants k1 and k2 in the equation:
     
voltage=k1*distance**(k2)
 
"""
import numpy as np
import matplotlib.pyplot as plt

def generate_random(length):
    import random

    index = list(np.arange(0,length,1))
    random.shuffle(index)

    x = index[:int(length/2)]
    y = index[int(length/2):]

    x.sort()
    y.sort()
    
    return x, y
 
# data for estimation
distances = np.array([10, 11, 12, 13, 14, 20, 25, 30 ,35, 40])
raw_measurements_total = np.array([2.313281298
,2.198046923
,2.053417921
,1.893701077
,1.655859375





,1.437988281




,1.237011671




,1.103222609




,1.044921875
,1.009521484])

plt.plot(distances, raw_measurements_total)
plt.xlabel('distance [cm]')
plt.ylabel('voltage [V]')
plt.savefig('general_shape.png')
plt.cla()

k1 = 0
k2 = 0

times = 5000
for _ in range(times):
    sample1, sample2 = generate_random(len(distances))
    distances_trimed=distances[sample1]
    raw_measurements_total_trimed=raw_measurements_total[sample1]
    
    # validation data 
    distances2=distances[sample2]
    raw_measurements2=raw_measurements_total[sample2]
    
    # print(distances_trimed)
    # print(distances2)
    n=distances_trimed.shape[0]
    y=np.zeros(shape=(n,1))
    A=np.zeros(shape=(n,2))
    
    for i in range(n):
        y[i,0]=np.log(raw_measurements_total_trimed[i])
        A[i,0]=1
        A[i,1]=np.log(distances_trimed[i])
    
    # solution    
    c=np.matmul(np.matmul(np.linalg.inv(np.matmul(A.T,A)),A.T),y)
    k1+=np.exp(c[0])
    k2+=c[1]
    
k1 /= times
k2 /= times
print(k1, k2)

# test on the estimation data
raw_measurements_total_trimed_prediction=np.zeros(shape=(n,1))
for i in range(n):
    raw_measurements_total_trimed_prediction[i]=k1*distances_trimed[i]**(k2)

plt.plot(distances_trimed,raw_measurements_total_trimed_prediction,'xr',label='least-squares prediction')
plt.plot(distances_trimed,raw_measurements_total_trimed,'k',label='real data')
plt.xlabel('distance [cm]')
plt.ylabel('voltage [V]')
plt.legend()
plt.savefig('estimation_curve.png')
plt.cla()

# test on the validation data 
n1=distances2.shape[0]
raw_measurements2_prediction=np.zeros(shape=(n1,1))
for i in range(n1):
    raw_measurements2_prediction[i]=k1*distances2[i]**(k2)

plt.plot(distances2,raw_measurements2_prediction,'xr',label='least-squares prediction')
plt.plot(distances2,raw_measurements2,'k',label='real data')
plt.xlabel('distance [cm]')
plt.ylabel('voltage [V]')
plt.legend()
plt.savefig('validation_curve.png')
plt.cla()