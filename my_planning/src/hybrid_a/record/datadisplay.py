import pandas as pd
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Rectangle

cw=2
lb=1.3
lf=3.4

# path = 'coarse.csv'
path = 'guess0.csv'
# path = 'guess1.csv'
# path = 'guess2.csv'
# path = 'guess3.csv'
# path = 'guess9.csv'
data = pd.read_csv(path)

def displayCar():
    fig=plt.figure()
    fig.canvas.set_window_title("XY")
    x=data['x']
    y=data['y']
    plt.scatter(x,y,s=2,alpha=0.5,c='black')
    ax=plt.gca()
    ax.set_aspect(1)
    theta=data['theta']
    for i in range(len(x)):
        rec=Rectangle((x[i]-math.sqrt(cw/2*cw/2+lb*lb) * math.sin(math.atan2(lb, cw/2)-theta[i]),
                       y[i] - math.sqrt(cw/2*cw/2+lb*lb) * math.cos(math.atan2(lb, cw/2)-theta[i])),
                       lb+lf, cw, theta[i] / math.pi * 180,
                       facecolor='none',edgecolor='green',alpha=0.7,lw=1,linestyle=":")
        ax.add_patch(rec)
    plt.axis([-4,60,-4,60])
    plt.xlabel("x"),plt.ylabel("y")
    
def displayTheta():
    fig=plt.figure()
    fig.canvas.set_window_title("Theta")
    theta=data['theta']
    plt.subplot(231)
    plt.xlabel("t")
    plt.ylabel("theta")
    plt.plot(theta,'.')

def displayV():
    # fig=plt.figure()
    # fig.canvas.set_window_title("V")
    v=data['v']
    plt.subplot(232)
    plt.xlabel("t")
    plt.ylabel("v")
    plt.plot(v,'.')

def displayPhi():
    # fig=plt.figure()
    # fig.canvas.set_window_title("Phi")
    phi=data['phi']
    plt.subplot(233)
    plt.xlabel("t")
    plt.ylabel("phi")
    plt.plot(phi,'.')

def displayA():
    # fig=plt.figure()
    # fig.canvas.set_window_title("A")
    a=data['a']
    plt.subplot(234)
    plt.xlabel("t")
    plt.ylabel("a")
    plt.plot(a,'.')

def displayOmega():
    # fig=plt.figure()
    # fig.canvas.set_window_title("2")
    omega=data['omega']
    plt.subplot(235)
    plt.xlabel("t")
    plt.ylabel("omega")
    plt.plot(omega,'.')
    sqrt_omega=[num**2 for num in omega]
    print(sum(sqrt_omega))

def displayJerk():
    # fig=plt.figure()
    # fig.canvas.set_window_title("Jerk")
    jerk=data['jerk']
    plt.subplot(236)
    plt.xlabel("t")
    plt.ylabel("jerk")
    plt.plot(jerk,'.')

    sqrt_jerk=[num**2 for num in jerk]
    print(sum(sqrt_jerk))

if __name__ == "__main__":
    displayCar()
    displayTheta()
    displayV()
    displayPhi()
    displayA()
    displayOmega()
    displayJerk()
    plt.show()
    
