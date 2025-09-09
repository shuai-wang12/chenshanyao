import pandas as pd
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

fig,ax = plt.subplots(subplot_kw={"projection":"3d"})

ax.set_xlim([0,100])
ax.set_ylim([0,100])
ax.set_zlim([0,1000])

voxelarray=np.zeros((60,60,1000),dtype=bool)
cararray=np.empty(shape=(0,2))
cw=2
lb=1
lf=3

for i in range(-lb,lf):
    for j in range(-1,1):
        cararray=np.r_[cararray,np.array([[j,i]])]
print(cararray)


# v1=np.array([-lb,cw/2])
# v2=np.array([lf,cw/2])
# v3=np.array([lf,-cw/2])
# v4=np.array([-lb,-cw/2])

path0 = 'solutions0.csv'
path1 = 'solutions1.csv'
path2 = 'solutions2.csv'
path3 = 'solutions3.csv'
path4 = 'solutions4.csv'
path5 = 'solutions5.csv'
# path6 = 'solutions6.csv'
data0 = pd.read_csv(path0)
data1 = pd.read_csv(path1)
data2 = pd.read_csv(path2)
data3 = pd.read_csv(path3)
data4 = pd.read_csv(path4)
data5 = pd.read_csv(path5)
# data6 = pd.read_csv(path6)

x0=data0['x']
y0=data0['y']
theta0=data0['theta']
t0=data0['t']
x1=data1['x']
y1=data1['y']
theta1=data1['theta']
t1=data1['t']
x2=data2['x']
y2=data2['y']
theta2=data2['theta']
t2=data2['t']
x3=data3['x']
y3=data3['y']
theta3=data3['theta']
t3=data3['t']
x4=data4['x']
y4=data4['y']
theta4=data4['theta']
t4=data4['t']
x5=data5['x']
y5=data5['y']
theta5=data5['theta']
t5=data5['t']
#x6=data6['x']
#y6=data6['y']
#theta6=data6['theta']
#t6=data6['t']
# print(t[0])

# plant1x=np.array([])
# plant1y=np.array([])
# Z=np.array([])

# for i in range(len(theta)):
#     R =np.array([[math.cos(theta[i]),-math.sin(theta[i])],[math.sin(theta[i]),math.cos(theta[i])]])
#     # p1 = np.dot(R,v1)+np.array([x[i],y[i]])
#     # p2 = np.dot(R,v2)+np.array([x[i],y[i]])
#     # p3 = np.dot(R,v3)+np.array([x[i],y[i]])
#     # p4 = np.dot(R,v4)+np.array([x[i],y[i]])
#     for j in cararray:
#         p = np.dot(R,j)+np.array([x[i],y[i]])
#         # print(p)
#         voxelarray[int(p[0]),int(p[1]),int(t[i])]=True
#     # plant1x=np.insert(plant1x,len(plant1x),p1[0])
#     # plant1y=np.insert(plant1y,len(plant1y),p1[1])
#     # Z=np.append(Z,t[i])
        
# X,Y=np.meshgrid(plant1x,plant1y)
ax.scatter3D(x0,y0,t0,cmap='rainbow')
ax.scatter3D(x1,y1,t1,cmap='rainbow')
ax.scatter3D(x2,y2,t2,cmap='rainbow')
ax.scatter3D(x3,y3,t3,cmap='rainbow')
ax.scatter3D(x4,y4,t4,cmap='rainbow')
ax.scatter3D(x5,y5,t5,cmap='rainbow')
#ax.scatter3D(x6,y6,t6,cmap='rainbow')
# print(X)
# print(Y)
# print(Z)
# ax.voxels(voxelarray,edgecolor='k')

plt.show()

