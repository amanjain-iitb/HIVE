import matplotlib.pyplot as plt
Kp=1
Kd=0.5
Ki=3
f_dev=0
f_integ=0
f=[5000,]
speed=[1000,]
t=0.05         #seconds

while f[-1]>=5:
    f.append(f[-1]-speed[-1]*t)
    f_dev=(f[-1]-f[-2])/t
    f_integ+=f[-1]*t
    speed.append(f[-1]*Kp + Kd*f_dev + Ki*f_integ)

n=len(speed)
time=[i*t for i in range(n)]
plt.subplot(211)
plt.plot(time,speed,'ro')
plt.subplot(212)
plt.plot(time,f)
plt.show()