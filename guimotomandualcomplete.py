import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import sympy as sp
from matplotlib.widgets import Slider,CheckButtons,Button,TextBox,RadioButtons
from robolink import *
import serial
import threading
import time
import sys

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.37,bottom=0.28,right=1,top=1)
ax = plt.axes(projection = "3d")


def mmatrix(*matrices):
    n=0
    for m in matrices:
        if (n==0):
            ma=m
            n=n+1
        elif (n==1):
            r=np.dot(ma,m)
            n=n+1
        else:
            r=np.dot(r,m)
    return r
    


def dibujar():
    plt.draw()
    plt.pause(0.001)


def sind(t):
    res=np.sin(t*np.pi/180)
    return res

def cosd(t):
    res=np.cos(t*np.pi/180)
    return res

def setaxis(lim=2):
    x1=-lim
    x2=lim
    y1=-lim
    y2=lim
    z1=-lim
    z2=lim
    ax.set_xlim3d(x1,x2)
    ax.set_ylim3d(y1,y2)
    ax.set_zlim3d(z1,z2)
    ax.view_init(elev=30,azim=40)
    ax.grid(True)

def sistemafijo(rango=1):
    x=[0,1*rango]
    y=[0,1*rango]
    z=[0,1*rango]
    ax.plot3D(x,[0,0],[0,0],color='red')
    ax.plot3D([0,0],y,[0,0],color='green')
    ax.plot3D([0,0],[0,0],z,color='blue')


def rotax(t):
    Rx=np.array(([1,0,0,0],[0,cosd(t),-sind(t),0],[0,sind(t),cosd(t),0],[0,0,0,1]))
    return Rx

def rotay(t):
    Ry=np.array(([cosd(t),0,sind(t),0],[0,1,0,0],[-sind(t),0,cosd(t),0],[0,0,0,1]))
    return Ry

def rotaz(t):
    Rz=np.array(([cosd(t),-sind(t),0,0],[sind(t),cosd(t),0,0],[0,0,1,0],[0,0,0,1]))
    return Rz

def rotaxf(t,r):
    px=r[0,3]
    py=r[1,3]
    pz=r[2,3]
    Rx=np.array(([1,0,0,0],[0,cosd(t),-sind(t),0],[0,sind(t),cosd(t),0],[0,0,0,1]))
    Rx=np.dot(Rx,r)
    Rx[0,3]=px
    Rx[1,3]=py
    Rx[2,3]=pz
    return Rx

def rotayf(t,r):
    px=r[0,3]
    py=r[1,3]
    pz=r[2,3]
    Ry=np.array(([cosd(t),0,sind(t),0],[0,1,0,0],[-sind(t),0,cosd(t),0],[0,0,0,1]))
    Ry=np.dot(Ry,r)
    Ry[0,3]=px
    Ry[1,3]=py
    Ry[2,3]=pz
    return Ry

def rotazf(t,r):
    px=r[0,3]
    py=r[1,3]
    pz=r[2,3]
    Rz=np.array(([cosd(t),-sind(t),0,0],[sind(t),cosd(t),0,0],[0,0,1,0],[0,0,0,1]))
    Rz=np.dot(Rz,r)
    Rz[0,3]=px
    Rz[1,3]=py
    Rz[2,3]=pz
    return Rz


def trasx(Dx):
    Tx=np.array(([[1,0,0,Dx],[0,1,0,0],[0,0,1,0],[0,0,0,1]]))
    return Tx

def trasy(Dy):
    Ty=np.array(([[1,0,0,0],[0,1,0,Dy],[0,0,1,0],[0,0,0,1]]))
    return Ty

def trasz(Dz):
    Tz=np.array(([[1,0,0,0],[0,1,0,0],[0,0,1,Dz],[0,0,0,1]]))
    return Tz

def ur5movej(p1,p2):
    n=1
    tetar=[p1[0],p1[1],p1[2],p1[3],p1[4],p1[5]]
    paso=[0,0,0,0,0,0]
    paso[0]=(p2[0]-p1[0])/30
    paso[1]=(p2[1]-p1[1])/30
    paso[2]=(p2[2]-p1[2])/30
    paso[3]=(p2[3]-p1[3])/30
    paso[4]=(p2[4]-p1[4])/30
    paso[5]=(p2[5]-p1[5])/30
    while n<31:
        tetar[0]=tetar[0]+paso[0]
        tetar[1]=tetar[1]+paso[1]
        tetar[2]=tetar[2]+paso[2]
        tetar[3]=tetar[3]+paso[3]
        tetar[4]=tetar[4]+paso[4]
        tetar[5]=tetar[5]+paso[5]
        n=n+1
        ax.cla()
        setaxis(1000)
        print(tetar)
        ur5(tetar[0],tetar[1],tetar[2],tetar[3],tetar[4],tetar[5])
        dibujar()
        valores1=tetar.copy()
        valores1[1]=valores1[1]-90
        valores1[2]=valores1[2]-90
        valores1[4]=valores1[4]+90
        valores1[4]=valores1[4]%360
        robot.MoveJ(valores1)

def ur5movel(p1,p2,sem):
    con=0
    n=0
    pn=p1
    p1x=p1[0,3]
    p1y=p1[1,3]
    p1z=p1[2,3]
    p2x=p2[0,3]
    p2y=p2[1,3]
    p2z=p2[2,3]
    dx=p2x-p1x
    dy=p2y-p1y 
    dz=p2z-p1z
    p1ea=mrot2eangle(p1)
    angrot=obtangrot(p2,p1ea)
    angrot[0]=angrot[0]/30
    angrot[1]=angrot[1]/30
    angrot[2]=angrot[2]/30
    angroti=[0,0,0]
    while n<1:
        pn=p1@rotaz(angroti[2])@rotay(angroti[1])@rotax(angroti[0])
        angroti[0]=angroti[0]+angrot[0]
        angroti[1]=angroti[1]+angrot[1]
        angroti[2]=angroti[2]+angrot[2]
        x=p1x+dx*n
        y=p1y+dy*n
        z=p1z+dz*n
        n+=1/30
        con+=1
        pn[0,3]=x
        pn[1,3]=y
        pn[2,3]=z
        print(pn)
        print(x)
        print(y)
        print(z)
        tetas=ur5newton(pn,sem)
        sem=tetas.copy()
        ax.cla()
        setaxis(1000)
        ur5(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        valores1=tetas.copy()
        valores1[1]=valores1[1]-90
        valores1[2]=valores1[2]-90
        valores1[4]=valores1[4]+90
        valores1[4]=valores1[4]%360
        robot.MoveJ(valores1)
        dibujar()

def ur5movec(p1,p2,sem):
    print(p1)
    print(p2)
    x,y,z=sp.symbols('x,y,z')
    con=0
    n=0
    pn=p1
    x1=p1[0,3]
    y1=p1[1,3]
    z1=p1[2,3]
    x2=p2[0,3]
    y2=p2[1,3]
    z2=p2[2,3]
    p1x=p1[0,3]
    p1y=p1[1,3]
    p1z=p1[2,3]
    p2x=p2[0,3]
    p2y=p2[1,3]
    p2z=p2[2,3]
    dx=x2-x1
    dy=y2-y1
    dz=z2-z1
    xm=x1+dx*0.5
    ym=y1+dy*0.5
    zm=z1+dz*0.5
    r=np.sqrt((x1-xm)**2+(y1-ym)**2+(z1-zm)**2)
    esfera=(x-xm)**2+(y-ym)**2+(z-zm)**2-r**2
    nd=[dy,-dx,0]
    plano=nd[0]*(x-x1)+nd[1]*(y-y1)+nd[2]*(z-z1)
    cir=sp.nonlinsolve([esfera,plano],[x,y,z])
    var=cir.free_symbols
    cirnp1=sp.lambdify(var, cir.args[0], "numpy")
    cirnp2=sp.lambdify(var, cir.args[1], "numpy")
    p1ea=mrot2eangle(p1)
    angrot=obtangrot(p2,p1ea)
    angrot[0]=angrot[0]/50
    angrot[1]=angrot[1]/50
    angrot[2]=angrot[2]/50
    angroti=[0,0,0]
    if np.around(cirnp1(z1)[0],4)== x1:
        rango1=np.arange(start=np.around(zm+r,4), stop=z2, step=-(zm+r-z2)/50)
        puntos1=cirnp2(rango1)
        rango2=np.arange(start=z1, stop=zm+r, step=(zm+r-z1)/50)
        puntos2=cirnp1(rango2)
        print(puntos1)
        print(puntos2)
        while n<puntos1[0].size:
            pn=p1@rotaz(angroti[2])@rotay(angroti[1])@rotax(angroti[0])
            angroti[0]=angroti[0]+angrot[0]
            angroti[1]=angroti[1]+angrot[1]
            angroti[2]=angroti[2]+angrot[2]
            px=puntos2[0][n]
            py=puntos2[1][n]
            pz=puntos2[2][n]
            n=n+1
            pn[0,3]=px
            pn[1,3]=py
            pn[2,3]=pz
            tetas=ur5newton(pn,sem)
            sem=tetas.copy()
            ax.cla()
            setaxis(1000)
            ur5(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
            valores1=tetas.copy()
            valores1[1]=valores1[1]-90
            valores1[2]=valores1[2]-90
            valores1[4]=valores1[4]+90
            valores1[4]=valores1[4]%360
            robot.MoveJ(valores1)
            dibujar()
        n=0
        while n<puntos2[0].size:
            pn=p1@rotaz(angroti[2])@rotay(angroti[1])@rotax(angroti[0])
            angroti[0]=angroti[0]+angrot[0]
            angroti[1]=angroti[1]+angrot[1]
            angroti[2]=angroti[2]+angrot[2]
            px=puntos1[0][n]
            py=puntos1[1][n]
            pz=puntos1[2][n]
            n=n+1
            pn[0,3]=px
            pn[1,3]=py
            pn[2,3]=pz
            tetas=ur5newton(pn,sem)
            sem=tetas.copy()
            ax.cla()
            setaxis(1000)
            ur5(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
            valores1=tetas.copy()
            valores1[1]=valores1[1]-90
            valores1[2]=valores1[2]-90
            valores1[4]=valores1[4]+90
            valores1[4]=valores1[4]%360
            robot.MoveJ(valores1)
            dibujar()
    else:
        rango1=np.arange(start=zm+r, stop=z2, step=-(zm+r-z2)/50)
        puntos1=cirnp1(rango1)
        rango2=np.arange(start=z1, stop=zm+r, step=(zm+r-z1)/50)
        puntos2=cirnp2(rango2)
        
        while n<puntos1[0].size:
            pn=p1@rotaz(angroti[2])@rotay(angroti[1])@rotax(angroti[0])
            angroti[0]=angroti[0]+angrot[0]
            angroti[1]=angroti[1]+angrot[1]
            angroti[2]=angroti[2]+angrot[2]
            px=puntos2[0][n]
            py=puntos2[1][n]
            pz=puntos2[2][n]
            n=n+1
            pn[0,3]=px
            pn[1,3]=py
            pn[2,3]=pz
            tetas=ur5newton(pn,sem)
            sem=tetas.copy()
            ax.cla()
            setaxis(1000)
            ur5(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
            valores1=tetas.copy()
            valores1[1]=valores1[1]-90
            valores1[2]=valores1[2]-90
            valores1[4]=valores1[4]+90
            valores1[4]=valores1[4]%360
            robot.MoveJ(valores1)
            dibujar()
        n=0
        while n<puntos2[0].size:
            
            pn=p1@rotaz(angroti[2])@rotay(angroti[1])@rotax(angroti[0])
            angroti[0]=angroti[0]+angrot[0]
            angroti[1]=angroti[1]+angrot[1]
            angroti[2]=angroti[2]+angrot[2]
            px=puntos1[0][n]
            py=puntos1[1][n]
            pz=puntos1[2][n]
            n=n+1
            pn[0,3]=px
            pn[1,3]=py
            pn[2,3]=pz
            tetas=ur5newton(pn,sem)
            sem=tetas.copy()
            ax.cla()
            setaxis(1000)
            ur5(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
            valores1=tetas.copy()
            valores1[1]=valores1[1]-90
            valores1[2]=valores1[2]-90
            valores1[4]=valores1[4]+90
            valores1[4]=valores1[4]%360
            robot.MoveJ(valores1)
            dibujar()
    
    

def mrot2eangle(r):
    teta=[0,0,0]
    if r[0,2]==1 or r[0,2]==-1:
        teta[0]=0
        teta[1]=r[0,2]*90
        teta[2]=np.degrees(np.arctan2(r[1,0],r[1,1]))
    else:
        teta[0]=np.degrees(np.arctan2(-r[1,2],r[2,2]))
        teta[1]=np.degrees(np.arcsin(r[0,2]))
        teta[2]=np.degrees(np.arctan2(-r[0,1],r[0,0]))
    return teta

def minv(R):
    r=np.zeros((4,4))
    a=np.zeros((3,3))
    p=np.zeros((3,1))
    a[0,0]=R[0,0]
    a[0,1]=R[0,1]
    a[0,2]=R[0,2]
    a[1,0]=R[1,0]
    a[1,1]=R[1,1]
    a[1,2]=R[1,2]
    a[2,0]=R[2,0]
    a[2,1]=R[2,1]
    a[2,2]=R[2,2]
    a=np.transpose(a)
    r[0,0]=a[0,0]
    r[0,1]=a[0,1]
    r[0,2]=a[0,2]
    r[1,0]=a[1,0]
    r[1,1]=a[1,1]
    r[1,2]=a[1,2]
    r[2,0]=a[2,0]
    r[2,1]=a[2,1]
    r[2,2]=a[2,2]
    a=-1*a
    p[0,0]=R[0,3]
    p[1,0]=R[1,3]
    p[2,0]=R[2,3]
    p1=np.dot(a,p)
    r[0,3]=p1[0,0]
    r[1,3]=p1[1,0]
    r[2,3]=p1[2,0]
    r[3,3]=1
    return r
    


def sistemamovil(r,rango=1):
    ux=r[0,0]
    uy=r[1,0]
    uz=r[2,0]
    vx=r[0,1]
    vy=r[1,1]
    vz=r[2,1]
    wx=r[0,2]
    wy=r[1,2]
    wz=r[2,2]

    px=r[0,3]
    py=r[1,3]
    pz=r[2,3]
    
    ax.plot3D([px,px+ux*rango],[py,py+uy*rango],[pz,pz+uz*rango],color='red') #Dibuja eje movil u
    ax.plot3D([px,px+vx*rango],[py,py+vy*rango],[pz,pz+vz*rango],color='green') #Dibuja eje movil v
    ax.plot3D([px,px+wx*rango],[py,py+wy*rango],[pz,pz+wz*rango],color='blue') #Dibuja eje movil w
    
def ppp(d1,d2,d3):
    t0=np.eye(4)
    t01=trasz(d1)@rotax(-90)
    t12=trasz(d2)@rotax(-90)@rotay(90)
    t23=trasz(d3)@rotaz(180)
    t02=t01@t12
    t03=t02@t23
    sistemafijo()
    sistemamovil(t01)
    sistemamovil(t02)
    sistemamovil(t03)
    ax.plot3D([t0[0,3],t01[0,3]],[t0[1,3],t01[1,3]],[t0[2,3],t01[2,3]],color='red')
    ax.plot3D([t01[0,3],t02[0,3]],[t01[1,3],t02[1,3]],[t01[2,3],t02[2,3]],color='red')
    ax.plot3D([t02[0,3],t03[0,3]],[t02[1,3],t03[1,3]],[t02[2,3],t03[2,3]],color='red')


def rpp(t1,d2,d3):
    t0=np.eye(4)
    t01=rotaz(t1)
    t12=trasz(d2)
    t23=rotay(90)@trasz(d3)
    t02=t01@t12
    t03=t02@t23
    sistemafijo()
    sistemamovil(t01)
    sistemamovil(t02)
    sistemamovil(t03)
    ax.plot3D([t0[0,3],t01[0,3]],[t0[1,3],t01[1,3]],[t0[2,3],t01[2,3]],color='red')
    ax.plot3D([t01[0,3],t02[0,3]],[t01[1,3],t02[1,3]],[t01[2,3],t02[2,3]],color='red')
    ax.plot3D([t02[0,3],t03[0,3]],[t02[1,3],t03[1,3]],[t02[2,3],t03[2,3]],color='red')


def rrp(t1,t2,d3):
    t0=np.eye(4)
    t01=rotaz(t1)
    t12=trasz(5)@rotay(90)@rotaz(90)@rotaz(t2)
    t23=rotay(90)@rotaz(-90)@trasz(d3)
    t02=t01@t12
    t03=t02@t23
    sistemafijo()
    sistemamovil(t01)
    sistemamovil(t02)
    sistemamovil(t03)
    ax.plot3D([t0[0,3],t01[0,3]],[t0[1,3],t01[1,3]],[t0[2,3],t01[2,3]],color='red')
    ax.plot3D([t01[0,3],t02[0,3]],[t01[1,3],t02[1,3]],[t01[2,3],t02[2,3]],color='red')
    ax.plot3D([t02[0,3],t03[0,3]],[t02[1,3],t03[1,3]],[t02[2,3],t03[2,3]],color='red')



def rrr(t1,t2,t3):
    t0=np.eye(4)
    t01=rotaz(t1)
    t12=trasz(4)@rotax(90)@rotaz(t2)
    t23=trasx(4)@rotaz(t3)
    t34=trasx(4)@rotay(90)@rotaz(-90)
    t02=t01@t12
    t03=t02@t23
    t04=t03@t34
    sistemafijo()
    sistemamovil(t01)
    sistemamovil(t02)
    sistemamovil(t03)
    sistemamovil(t04)
    ax.plot3D([t0[0,3],t01[0,3]],[t0[1,3],t01[1,3]],[t0[2,3],t01[2,3]],color='red')
    ax.plot3D([t01[0,3],t02[0,3]],[t01[1,3],t02[1,3]],[t01[2,3],t02[2,3]],color='red')
    ax.plot3D([t02[0,3],t03[0,3]],[t02[1,3],t03[1,3]],[t02[2,3],t03[2,3]],color='red')
    ax.plot3D([t03[0,3],t04[0,3]],[t03[1,3],t04[1,3]],[t03[2,3],t04[2,3]],color='red')


def scara(t1,t2,d3,t4):
    t0=np.eye(4)
    t01=rotaz(t1)@trasz(4)
    t12=trasx(4)
    t23=rotaz(t2)@trasz(-1)
    t34=trasx(4)@rotax(180)@rotaz(-90)
    t45=trasz(d3)
    t56=rotaz(t4)@trasz(1)
    t02=t01@t12
    t03=t02@t23
    t04=t03@t34
    t05=t04@t45
    t06=t05@t56
    sistemafijo()
    sistemamovil(t01)
    sistemamovil(t02)
    sistemamovil(t03)
    sistemamovil(t04)
    sistemamovil(t05)
    sistemamovil(t06)
    ax.plot3D([t0[0,3],t01[0,3]],[t0[1,3],t01[1,3]],[t0[2,3],t01[2,3]],color='red')
    ax.plot3D([t01[0,3],t02[0,3]],[t01[1,3],t02[1,3]],[t01[2,3],t02[2,3]],color='red')
    ax.plot3D([t02[0,3],t03[0,3]],[t02[1,3],t03[1,3]],[t02[2,3],t03[2,3]],color='red')
    ax.plot3D([t03[0,3],t04[0,3]],[t03[1,3],t04[1,3]],[t03[2,3],t04[2,3]],color='red')
    ax.plot3D([t04[0,3],t05[0,3]],[t04[1,3],t05[1,3]],[t04[2,3],t05[2,3]],color='red')
    ax.plot3D([t05[0,3],t06[0,3]],[t05[1,3],t06[1,3]],[t05[2,3],t06[2,3]],color='red')


def cobras800(t1,t2,d3,t4):
    t0=np.eye(4)
    t01=rotaz(t1)@trasz(342)
    t12=trasx(425)
    t23=rotaz(t2)@trasz(56)
    t34=trasx(375)
    t45=trasz(-210)@trasz(d3)
    t56=rotax(180)@rotaz(-180)@rotaz(t4)
    t02=t01@t12
    t03=t02@t23
    t04=t03@t34
    t05=t04@t45
    t06=t05@t56
    sistemafijo(100)
    #sistemamovil(t01,100)
    #sistemamovil(t02,100)
    sistemamovil(t03,100)
    #sistemamovil(t04,100)
    #sistemamovil(t05,100)
    sistemamovil(t06,100)
    ax.plot3D([t0[0,3],t01[0,3]],[t0[1,3],t01[1,3]],[t0[2,3],t01[2,3]],color='red')
    ax.plot3D([t01[0,3],t02[0,3]],[t01[1,3],t02[1,3]],[t01[2,3],t02[2,3]],color='red')
    ax.plot3D([t02[0,3],t03[0,3]],[t02[1,3],t03[1,3]],[t02[2,3],t03[2,3]],color='red')
    ax.plot3D([t03[0,3],t04[0,3]],[t03[1,3],t04[1,3]],[t03[2,3],t04[2,3]],color='red')
    ax.plot3D([t04[0,3],t05[0,3]],[t04[1,3],t05[1,3]],[t04[2,3],t05[2,3]],color='red')
    ax.plot3D([t05[0,3],t06[0,3]],[t05[1,3],t06[1,3]],[t05[2,3],t06[2,3]],color='red')


def ur5(t1,t2,t3,t4,t5,t6,mb=np.eye(4)):
    t0=np.eye(4)@mb
    t01=mb@rotaz(t1)@trasz(89.2)#
    t12=trasy(-134.2)@rotax(90)@rotaz(t2)#
    t23=trasy(425)
    t34=trasz(-118.45)@rotaz(t3)#
    t45=trasx(392.25)@rotaz(t4)#
    t56=trasz(94.75)@rotax(-90)@rotaz(t5)#
    t67=trasz(94.75)
    t78=trasx(82.5)@rotay(90)@rotaz(-90)@rotaz(t6)#
    t02=t01@t12
    t03=t02@t23
    t04=t03@t34
    t05=t04@t45
    t06=t05@t56
    t07=t06@t67
    t08=t07@t78
    print(t08)
    sistemamovil(t0,100)
    #sistemamovil(t01,100)
    sistemamovil(t02,100)
    sistemamovil(t03,100)
    #sistemamovil(t04,100)
    sistemamovil(t05,100)
    sistemamovil(t06,100)
    sistemamovil(t07,100)
    sistemamovil(t08,100)
    ax.plot3D([t0[0,3],t01[0,3]],[t0[1,3],t01[1,3]],[t0[2,3],t01[2,3]],color='red')
    ax.plot3D([t01[0,3],t02[0,3]],[t01[1,3],t02[1,3]],[t01[2,3],t02[2,3]],color='red')
    ax.plot3D([t02[0,3],t03[0,3]],[t02[1,3],t03[1,3]],[t02[2,3],t03[2,3]],color='red')
    ax.plot3D([t03[0,3],t04[0,3]],[t03[1,3],t04[1,3]],[t03[2,3],t04[2,3]],color='red')
    ax.plot3D([t04[0,3],t05[0,3]],[t04[1,3],t05[1,3]],[t04[2,3],t05[2,3]],color='red')
    ax.plot3D([t05[0,3],t06[0,3]],[t05[1,3],t06[1,3]],[t05[2,3],t06[2,3]],color='red')
    ax.plot3D([t06[0,3],t07[0,3]],[t06[1,3],t07[1,3]],[t06[2,3],t07[2,3]],color='red')
    ax.plot3D([t07[0,3],t08[0,3]],[t07[1,3],t08[1,3]],[t07[2,3],t08[2,3]],color='red')


def motoman(tb,t1a,t2a,t3a,t4a,t5a,t6a,t7a,t1b,t2b,t3b,t4b,t5b,t6b,t7b):
    T0=np.eye(4)
    Ti=rotaz(tb)
    Ti1=Ti@trasz(8)
    Ti2=Ti1@trasx(1.57)
    Ti3=Ti2@trasy(2.5)@rotax(-90)
    Tib3=Ti2@trasy(-2.5)@rotax(90)
    
    T01=Ti3@rotaz(270)@rotaz(t1a)@trasz(.3);
    T12=trasx(-1.09)
    T23=trasz(2.5)
    T34=trasx(1.09)@rotay(90)@rotaz(t2a)
    T45=rotay(-90)@trasx(1.2)
    T56=trasz(1)
    T67=trasx(-1.2)
    T78=trasz(0.45)@rotaz(t3a)
    T89=trasz(2.225)
    T910=trasx(1.04)
    T1011=trasz(1.225)
    T1112=trasx(-1.04)@rotay(90)@rotaz(t4a)
    T1213=rotay(-90)@trasx(-0.98)
    T1314=trasz(1.4)
    T1415=trasx(0.98)
    T1516=trasz(0.7)@rotaz(t5a)
    T1617=trasz(0.7)
    T1718=trasx(-0.86)
    T1819=trasz(1.4)
    T1920=trasx(0.86)@rotay(90)@rotaz(t6a)
    T2021=rotay(-90)@trasx(0.8)
    T2122=trasz(0.9)
    T2223=trasx(-0.8)
    T2324=trasz(0.9)@rotaz(90)@rotaz(t7a)
    T02=T01@T12
    T03=T02@T23
    T04=T03@T34
    T05=T04@T45
    T06=T05@T56
    T07=T06@T67
    T08=T07@T78
    T09=T08@T89
    T10=T09@T910
    T11=T10@T1011
    T12=T11@T1112
    T13=T12@T1213
    T14=T13@T1314
    T15=T14@T1415
    T16=T15@T1516
    T17=T16@T1617
    T18=T17@T1718
    T19=T18@T1819
    T20=T19@T1920
    T21=T20@T2021
    T22=T21@T2122
    T23=T22@T2223
    T24=T23@T2324
    print(T24)



    Tb01=Tib3@rotaz(270)@rotaz(t1b)@trasz(0.3)
    Tb12=trasx(-1.09)
    Tb23=trasz(2.5)
    TB34=trasx(1.09)@rotay(90)@rotaz(t2b)
    Tb45=rotay(-90)@trasx(1.2)
    Tb56=trasz(1)
    Tb67=trasx(-1.2) 
    Tb78=trasz(0.45)@rotaz(t3b)
    Tb89=trasz(2.225)
    TB910=trasx(1.04)
    Tb1011=trasz(1.225)
    Tb1112=trasx(-1.04)@rotay(90)@rotaz(t4b)
    Tb1213=rotay(-90)@trasx(-0.98)
    Tb1314=trasz(1.4)
    Tb1415=trasx(0.98)
    Tb1516=trasz(0.7)@rotaz(t5b)
    Tb1617=trasz(0.7)
    Tb1718=trasx(-0.86)
    Tb1819=trasz(1.4)
    Tb1920=trasx(0.86)@rotay(90)@rotaz(t6b)
    Tb2021=rotay(-90)@trasx(0.8)
    Tb2122=trasz(0.9)
    Tb2223=trasx(-0.8)
    Tb2324=trasz(0.9)@rotaz(90)@rotaz(t7b)

    Tb02=Tb01@Tb12
    Tb03=Tb02@Tb23
    Tb04=Tb03@TB34
    Tb05=Tb04@Tb45
    Tb06=Tb05@Tb56
    Tb07=Tb06@Tb67
    Tb08=Tb07@Tb78
    Tb09=Tb08@Tb89
    Tb10=Tb09@TB910
    Tb11=Tb10@Tb1011
    Tb12=Tb11@Tb1112
    Tb13=Tb12@Tb1213
    Tb14=Tb13@Tb1314
    Tb15=Tb14@Tb1415
    Tb16=Tb15@Tb1516
    Tb17=Tb16@Tb1617
    Tb18=Tb17@Tb1718
    Tb19=Tb18@Tb1819
    Tb20=Tb19@Tb1920
    Tb21=Tb20@Tb2021
    Tb22=Tb21@Tb2122
    Tb23=Tb22@Tb2223
    Tb24=Tb23@Tb2324

    sistemafijo()
    sistemamovil(T0)
    sistemamovil(T01)
    sistemamovil(T04)
    sistemamovil(T08)
    sistemamovil(T12)
    sistemamovil(T16)
    sistemamovil(T20)
    sistemamovil(T24)
    sistemamovil(T0)
    sistemamovil(Tb01)
    sistemamovil(Tb04)
    sistemamovil(Tb08)
    sistemamovil(Tb12)
    sistemamovil(Tb16)
    sistemamovil(Tb20)
    sistemamovil(Tb24)

    ax.plot3D([Ti[0,3], Ti1[0,3]], [Ti[1,3], Ti1[1,3]], [Ti[2,3], Ti1[2,3]],color='red')

    ax.plot3D([Ti1[0,3], Ti2[0,3]], [Ti1[1,3], Ti2[1,3]], [Ti1[2,3], Ti2[2,3]],color='red')
    ax.plot3D([Ti2[0,3], Ti3[0,3]], [Ti2[1,3], Ti3[1,3]], [Ti2[2,3], Ti3[2,3]],color='red')

    ax.plot3D([Ti3[0,3], T01[0,3]], [Ti3[1,3], T01[1,3]], [Ti3[2,3], T01[2,3]],color='red')
    ax.plot3D([T01[0,3], T02[0,3]], [T01[1,3], T02[1,3]], [T01[2,3], T02[2,3]],color='red')
    ax.plot3D([T02[0,3], T03[0,3]], [T02[1,3], T03[1,3]], [T02[2,3], T03[2,3]],color='red')
    ax.plot3D([T03[0,3], T04[0,3]], [T03[1,3], T04[1,3]], [T03[2,3], T04[2,3]],color='red')
    ax.plot3D([T04[0,3], T05[0,3]], [T04[1,3], T05[1,3]], [T04[2,3], T05[2,3]],color='red')
    ax.plot3D([T05[0,3], T06[0,3]], [T05[1,3], T06[1,3]], [T05[2,3], T06[2,3]],color='red')
    ax.plot3D([T06[0,3], T07[0,3]], [T06[1,3], T07[1,3]], [T06[2,3], T07[2,3]],color='red')
    ax.plot3D([T07[0,3], T08[0,3]], [T07[1,3], T08[1,3]], [T07[2,3], T08[2,3]],color='red')
    ax.plot3D([T08[0,3], T09[0,3]], [T08[1,3], T09[1,3]], [T08[2,3], T09[2,3]],color='red')
    ax.plot3D([T09[0,3], T10[0,3]], [T09[1,3], T10[1,3]], [T09[2,3], T10[2,3]],color='red')
    ax.plot3D([T10[0,3], T11[0,3]], [T10[1,3], T11[1,3]], [T10[2,3], T11[2,3]],color='red')
    ax.plot3D([T11[0,3], T12[0,3]], [T11[1,3], T12[1,3]], [T11[2,3], T12[2,3]],color='red')
    ax.plot3D([T12[0,3], T13[0,3]], [T12[1,3], T13[1,3]], [T12[2,3], T13[2,3]],color='red')
    ax.plot3D([T13[0,3], T14[0,3]], [T13[1,3], T14[1,3]], [T13[2,3], T14[2,3]],color='red')
    ax.plot3D([T14[0,3], T15[0,3]], [T14[1,3], T15[1,3]], [T14[2,3], T15[2,3]],color='red')
    ax.plot3D([T15[0,3], T16[0,3]], [T15[1,3], T16[1,3]], [T15[2,3], T16[2,3]],color='red')
    ax.plot3D([T16[0,3], T17[0,3]], [T16[1,3], T17[1,3]], [T16[2,3], T17[2,3]],color='red')
    ax.plot3D([T17[0,3], T18[0,3]], [T17[1,3], T18[1,3]], [T17[2,3], T18[2,3]],color='red')
    ax.plot3D([T18[0,3], T19[0,3]], [T18[1,3], T19[1,3]], [T18[2,3], T19[2,3]],color='red')
    ax.plot3D([T19[0,3], T20[0,3]], [T19[1,3], T20[1,3]], [T19[2,3], T20[2,3]],color='red')
    ax.plot3D([T20[0,3], T21[0,3]], [T20[1,3], T21[1,3]], [T20[2,3], T21[2,3]],color='red')
    ax.plot3D([T21[0,3], T22[0,3]], [T21[1,3], T22[1,3]], [T21[2,3], T22[2,3]],color='red')
    ax.plot3D([T22[0,3], T23[0,3]], [T22[1,3], T23[1,3]], [T22[2,3], T23[2,3]],color='red')
    ax.plot3D([T23[0,3], T24[0,3]], [T23[1,3], T24[1,3]], [T23[2,3], T24[2,3]],color='red')

    ax.plot3D([Ti[0,3], Ti1[0,3]], [Ti[1,3], Ti1[1,3]], [Ti[2,3], Ti1[2,3]],color='red')

    ax.plot3D([Ti1[0,3], Ti2[0,3]], [Ti1[1,3], Ti2[1,3]], [Ti1[2,3], Ti2[2,3]],color='red')
    ax.plot3D([Ti2[0,3], Tib3[0,3]], [Ti2[1,3], Tib3[1,3]], [Ti2[2,3], Tib3[2,3]],color='red')

    ax.plot3D([Tib3[0,3], Tb01[0,3]], [Tib3[1,3], Tb01[1,3]], [Tib3[2,3], Tb01[2,3]],color='red')
    ax.plot3D([Tb01[0,3], Tb02[0,3]], [Tb01[1,3], Tb02[1,3]], [Tb01[2,3], Tb02[2,3]],color='red')
    ax.plot3D([Tb02[0,3], Tb03[0,3]], [Tb02[1,3], Tb03[1,3]], [Tb02[2,3], Tb03[2,3]],color='red')
    ax.plot3D([Tb03[0,3], Tb04[0,3]], [Tb03[1,3], Tb04[1,3]], [Tb03[2,3], Tb04[2,3]],color='red')
    ax.plot3D([Tb04[0,3], Tb05[0,3]], [Tb04[1,3], Tb05[1,3]], [Tb04[2,3], Tb05[2,3]],color='red')
    ax.plot3D([Tb05[0,3], Tb06[0,3]], [Tb05[1,3], Tb06[1,3]], [Tb05[2,3], Tb06[2,3]],color='red')
    ax.plot3D([Tb06[0,3], Tb07[0,3]], [Tb06[1,3], Tb07[1,3]], [Tb06[2,3], Tb07[2,3]],color='red')
    ax.plot3D([Tb07[0,3], Tb08[0,3]], [Tb07[1,3], Tb08[1,3]], [Tb07[2,3], Tb08[2,3]],color='red')
    ax.plot3D([Tb08[0,3], Tb09[0,3]], [Tb08[1,3], Tb09[1,3]], [Tb08[2,3], Tb09[2,3]],color='red')
    ax.plot3D([Tb09[0,3], Tb10[0,3]], [Tb09[1,3], Tb10[1,3]], [Tb09[2,3], Tb10[2,3]],color='red')
    ax.plot3D([Tb10[0,3], Tb11[0,3]], [Tb10[1,3], Tb11[1,3]], [Tb10[2,3], Tb11[2,3]],color='red')
    ax.plot3D([Tb11[0,3], Tb12[0,3]], [Tb11[1,3], Tb12[1,3]], [Tb11[2,3], Tb12[2,3]],color='red')
    ax.plot3D([Tb12[0,3], Tb13[0,3]], [Tb12[1,3], Tb13[1,3]], [Tb12[2,3], Tb13[2,3]],color='red')
    ax.plot3D([Tb13[0,3], Tb14[0,3]], [Tb13[1,3], Tb14[1,3]], [Tb13[2,3], Tb14[2,3]],color='red')
    ax.plot3D([Tb14[0,3], Tb15[0,3]], [Tb14[1,3], Tb15[1,3]], [Tb14[2,3], Tb15[2,3]],color='red')
    ax.plot3D([Tb15[0,3], Tb16[0,3]], [Tb15[1,3], Tb16[1,3]], [Tb15[2,3], Tb16[2,3]],color='red')
    ax.plot3D([Tb16[0,3], Tb17[0,3]], [Tb16[1,3], Tb17[1,3]], [Tb16[2,3], Tb17[2,3]],color='red')
    ax.plot3D([Tb17[0,3], Tb18[0,3]], [Tb17[1,3], Tb18[1,3]], [Tb17[2,3], Tb18[2,3]],color='red')
    ax.plot3D([Tb18[0,3], Tb19[0,3]], [Tb18[1,3], Tb19[1,3]], [Tb18[2,3], Tb19[2,3]],color='red')
    ax.plot3D([Tb19[0,3], Tb20[0,3]], [Tb19[1,3], Tb20[1,3]], [Tb19[2,3], Tb20[2,3]],color='red')
    ax.plot3D([Tb20[0,3], Tb21[0,3]], [Tb20[1,3], Tb21[1,3]], [Tb20[2,3], Tb21[2,3]],color='red')
    ax.plot3D([Tb21[0,3], Tb22[0,3]], [Tb21[1,3], Tb22[1,3]], [Tb21[2,3], Tb22[2,3]],color='red')
    ax.plot3D([Tb22[0,3], Tb23[0,3]], [Tb22[1,3], Tb23[1,3]], [Tb22[2,3], Tb23[2,3]],color='red')
    ax.plot3D([Tb23[0,3], Tb24[0,3]], [Tb23[1,3], Tb24[1,3]], [Tb23[2,3], Tb24[2,3]],color='red')

def accmotoman(tb,t1a,t2a,t3a,t4a,t5a,t6a,t7a,t1b,t2b,t3b,t4b,t5b,t6b,t7b):
    T0=np.eye(4)
    Ti=trasz(893.5)
    Ti1=Ti@trasx(92.5)@rotaz(tb)
    Ti2=Ti1@trasx(100)@trasz(306.5)
    Ti3=Ti2@rotax(-90)@rotaz(-180)@rotaz(t1a)@trasz(265)
    Tib3=Ti2@rotax(90)@rotaz(-180)@rotaz(t1b)@trasz(265)
    
    T01=Ti3@rotax(-90)@rotaz(t2a)
    T12=trasz(-80)
    T23=trasy(-90)
    T34=trasz(80)
    T45=trasy(-90)@rotax(90)@rotaz(t3a)
    T56=trasz(90)
    T67=trasy(-80)
    T78=trasz(90)
    T89=trasy(80)@rotax(-90)@rotaz(t4a)
    T910=trasz(80)
    T1011=trasy(-90)
    T1112=trasz(-80)
    T1213=trasy(-90)@rotax(90)@rotaz(t5a)
    T1314=trasz(90)
    T1415=trasy(80)
    T1516=trasz(90)
    T1617=trasy(-80)@rotax(-90)@rotaz(t6a)
    T1718=trasz(-80)
    T1819=trasy(-87.5)
    T1920=trasz(80)
    T2021=trasy(-87.5)@rotax(90)@rotaz(t7a)
    T02=T01@T12
    T03=T02@T23
    T04=T03@T34
    T05=T04@T45
    T06=T05@T56
    T07=T06@T67
    T08=T07@T78
    T09=T08@T89
    T10=T09@T910
    T11=T10@T1011
    T12=T11@T1112
    T13=T12@T1213
    T14=T13@T1314
    T15=T14@T1415
    T16=T15@T1516
    T17=T16@T1617
    T18=T17@T1718
    T19=T18@T1819
    T20=T19@T1920
    T21=T20@T2021
    print("derecho ++++++")
    print(T21)


    Tb01=Tib3@rotax(-90)@rotaz(t2b)
    Tb12=trasz(-80)
    Tb23=trasy(-90)
    Tb34=trasz(80)
    Tb45=trasy(-90)@rotax(90)@rotaz(t3b)
    Tb56=trasz(90)
    Tb67=trasy(-80)
    Tb78=trasz(90)
    Tb89=trasy(80)@rotax(-90)@rotaz(t4b)
    Tb910=trasz(80)
    Tb1011=trasy(-90)
    Tb1112=trasz(-80)
    Tb1213=trasy(-90)@rotax(90)@rotaz(t5b)
    Tb1314=trasz(90)
    Tb1415=trasy(80)
    Tb1516=trasz(90)
    Tb1617=trasy(-80)@rotax(-90)@rotaz(t6b)
    Tb1718=trasz(-80)
    Tb1819=trasy(-87.5)
    Tb1920=trasz(80)
    Tb2021=trasy(-87.5)@rotax(90)@rotaz(t7b)
    
    

    Tb02=Tb01@Tb12
    Tb03=Tb02@Tb23
    Tb04=Tb03@Tb34
    Tb05=Tb04@Tb45
    Tb06=Tb05@Tb56
    Tb07=Tb06@Tb67
    Tb08=Tb07@Tb78
    Tb09=Tb08@Tb89
    Tb10=Tb09@Tb910
    Tb11=Tb10@Tb1011
    Tb12=Tb11@Tb1112
    Tb13=Tb12@Tb1213
    Tb14=Tb13@Tb1314
    Tb15=Tb14@Tb1415
    Tb16=Tb15@Tb1516
    Tb17=Tb16@Tb1617
    Tb18=Tb17@Tb1718
    Tb19=Tb18@Tb1819
    Tb20=Tb19@Tb1920
    Tb21=Tb20@Tb2021
    print("Izquierdo ++++++")
    print(Tb21)

    sistemafijo(100)
    sistemamovil(T0,100)
    sistemamovil(T01,100)
    sistemamovil(T05,100)
    sistemamovil(T09,100)
    sistemamovil(T13,100)
    sistemamovil(T17,100)
    sistemamovil(T21,100)
    sistemamovil(Tb01,100)
    sistemamovil(Tb05,100)
    sistemamovil(Tb09,100)
    sistemamovil(Tb13,100)
    sistemamovil(Tb17,100)
    sistemamovil(Tb21,100)

    ax.plot3D([T0[0,3], Ti[0,3]], [T0[1,3], Ti[1,3]], [T0[2,3], Ti[2,3]],color='red')


    ax.plot3D([Ti[0,3], Ti1[0,3]], [Ti[1,3], Ti1[1,3]], [Ti[2,3], Ti1[2,3]],color='red')

    ax.plot3D([Ti1[0,3], Ti2[0,3]], [Ti1[1,3], Ti2[1,3]], [Ti1[2,3], Ti2[2,3]],color='red')
    ax.plot3D([Ti2[0,3], Ti3[0,3]], [Ti2[1,3], Ti3[1,3]], [Ti2[2,3], Ti3[2,3]],color='red')

    ax.plot3D([Ti3[0,3], T01[0,3]], [Ti3[1,3], T01[1,3]], [Ti3[2,3], T01[2,3]],color='red')
    ax.plot3D([T01[0,3], T02[0,3]], [T01[1,3], T02[1,3]], [T01[2,3], T02[2,3]],color='red')
    ax.plot3D([T02[0,3], T03[0,3]], [T02[1,3], T03[1,3]], [T02[2,3], T03[2,3]],color='red')
    ax.plot3D([T03[0,3], T04[0,3]], [T03[1,3], T04[1,3]], [T03[2,3], T04[2,3]],color='red')
    ax.plot3D([T04[0,3], T05[0,3]], [T04[1,3], T05[1,3]], [T04[2,3], T05[2,3]],color='red')
    ax.plot3D([T05[0,3], T06[0,3]], [T05[1,3], T06[1,3]], [T05[2,3], T06[2,3]],color='red')
    ax.plot3D([T06[0,3], T07[0,3]], [T06[1,3], T07[1,3]], [T06[2,3], T07[2,3]],color='red')
    ax.plot3D([T07[0,3], T08[0,3]], [T07[1,3], T08[1,3]], [T07[2,3], T08[2,3]],color='red')
    ax.plot3D([T08[0,3], T09[0,3]], [T08[1,3], T09[1,3]], [T08[2,3], T09[2,3]],color='red')
    ax.plot3D([T09[0,3], T10[0,3]], [T09[1,3], T10[1,3]], [T09[2,3], T10[2,3]],color='red')
    ax.plot3D([T10[0,3], T11[0,3]], [T10[1,3], T11[1,3]], [T10[2,3], T11[2,3]],color='red')
    ax.plot3D([T11[0,3], T12[0,3]], [T11[1,3], T12[1,3]], [T11[2,3], T12[2,3]],color='red')
    ax.plot3D([T12[0,3], T13[0,3]], [T12[1,3], T13[1,3]], [T12[2,3], T13[2,3]],color='red')
    ax.plot3D([T13[0,3], T14[0,3]], [T13[1,3], T14[1,3]], [T13[2,3], T14[2,3]],color='red')
    ax.plot3D([T14[0,3], T15[0,3]], [T14[1,3], T15[1,3]], [T14[2,3], T15[2,3]],color='red')
    ax.plot3D([T15[0,3], T16[0,3]], [T15[1,3], T16[1,3]], [T15[2,3], T16[2,3]],color='red')
    ax.plot3D([T16[0,3], T17[0,3]], [T16[1,3], T17[1,3]], [T16[2,3], T17[2,3]],color='red')
    ax.plot3D([T17[0,3], T18[0,3]], [T17[1,3], T18[1,3]], [T17[2,3], T18[2,3]],color='red')
    ax.plot3D([T18[0,3], T19[0,3]], [T18[1,3], T19[1,3]], [T18[2,3], T19[2,3]],color='red')
    ax.plot3D([T19[0,3], T20[0,3]], [T19[1,3], T20[1,3]], [T19[2,3], T20[2,3]],color='red')
    ax.plot3D([T20[0,3], T21[0,3]], [T20[1,3], T21[1,3]], [T20[2,3], T21[2,3]],color='red')

    ax.plot3D([Ti[0,3], Ti1[0,3]], [Ti[1,3], Ti1[1,3]], [Ti[2,3], Ti1[2,3]],color='red')

    ax.plot3D([Ti1[0,3], Ti2[0,3]], [Ti1[1,3], Ti2[1,3]], [Ti1[2,3], Ti2[2,3]],color='red')
    ax.plot3D([Ti2[0,3], Tib3[0,3]], [Ti2[1,3], Tib3[1,3]], [Ti2[2,3], Tib3[2,3]],color='red')

    ax.plot3D([Tib3[0,3], Tb01[0,3]], [Tib3[1,3], Tb01[1,3]], [Tib3[2,3], Tb01[2,3]],color='red')
    ax.plot3D([Tb01[0,3], Tb02[0,3]], [Tb01[1,3], Tb02[1,3]], [Tb01[2,3], Tb02[2,3]],color='red')
    ax.plot3D([Tb02[0,3], Tb03[0,3]], [Tb02[1,3], Tb03[1,3]], [Tb02[2,3], Tb03[2,3]],color='red')
    ax.plot3D([Tb03[0,3], Tb04[0,3]], [Tb03[1,3], Tb04[1,3]], [Tb03[2,3], Tb04[2,3]],color='red')
    ax.plot3D([Tb04[0,3], Tb05[0,3]], [Tb04[1,3], Tb05[1,3]], [Tb04[2,3], Tb05[2,3]],color='red')
    ax.plot3D([Tb05[0,3], Tb06[0,3]], [Tb05[1,3], Tb06[1,3]], [Tb05[2,3], Tb06[2,3]],color='red')
    ax.plot3D([Tb06[0,3], Tb07[0,3]], [Tb06[1,3], Tb07[1,3]], [Tb06[2,3], Tb07[2,3]],color='red')
    ax.plot3D([Tb07[0,3], Tb08[0,3]], [Tb07[1,3], Tb08[1,3]], [Tb07[2,3], Tb08[2,3]],color='red')
    ax.plot3D([Tb08[0,3], Tb09[0,3]], [Tb08[1,3], Tb09[1,3]], [Tb08[2,3], Tb09[2,3]],color='red')
    ax.plot3D([Tb09[0,3], Tb10[0,3]], [Tb09[1,3], Tb10[1,3]], [Tb09[2,3], Tb10[2,3]],color='red')
    ax.plot3D([Tb10[0,3], Tb11[0,3]], [Tb10[1,3], Tb11[1,3]], [Tb10[2,3], Tb11[2,3]],color='red')
    ax.plot3D([Tb11[0,3], Tb12[0,3]], [Tb11[1,3], Tb12[1,3]], [Tb11[2,3], Tb12[2,3]],color='red')
    ax.plot3D([Tb12[0,3], Tb13[0,3]], [Tb12[1,3], Tb13[1,3]], [Tb12[2,3], Tb13[2,3]],color='red')
    ax.plot3D([Tb13[0,3], Tb14[0,3]], [Tb13[1,3], Tb14[1,3]], [Tb13[2,3], Tb14[2,3]],color='red')
    ax.plot3D([Tb14[0,3], Tb15[0,3]], [Tb14[1,3], Tb15[1,3]], [Tb14[2,3], Tb15[2,3]],color='red')
    ax.plot3D([Tb15[0,3], Tb16[0,3]], [Tb15[1,3], Tb16[1,3]], [Tb15[2,3], Tb16[2,3]],color='red')
    ax.plot3D([Tb16[0,3], Tb17[0,3]], [Tb16[1,3], Tb17[1,3]], [Tb16[2,3], Tb17[2,3]],color='red')
    ax.plot3D([Tb17[0,3], Tb18[0,3]], [Tb17[1,3], Tb18[1,3]], [Tb17[2,3], Tb18[2,3]],color='red')
    ax.plot3D([Tb18[0,3], Tb19[0,3]], [Tb18[1,3], Tb19[1,3]], [Tb18[2,3], Tb19[2,3]],color='red')
    ax.plot3D([Tb19[0,3], Tb20[0,3]], [Tb19[1,3], Tb20[1,3]], [Tb19[2,3], Tb20[2,3]],color='red')
    ax.plot3D([Tb20[0,3], Tb21[0,3]], [Tb20[1,3], Tb21[1,3]], [Tb20[2,3], Tb21[2,3]],color='red')

def accmotomanv(tb,t1a,t2a,t3a,t4a,t5a,t6a,t7a,t1b,t2b,t3b,t4b,t5b,t6b,t7b,mh=np.eye(4)):
    T0=np.eye(4)
    Ti=trasz(893.5)
    Ti1=Ti@trasx(92.5)@rotaz(tb)
    Ti2=Ti1@trasx(100)@trasz(306.5)
    Ti3=Ti2@rotax(-90)@rotaz(-180)@rotaz(t1a)@trasz(265)
    Tib3=Ti2@rotax(90)@rotaz(-180)@rotaz(t1b)@trasz(265)
    
    T01=Ti3@rotax(-90)@rotaz(t2a)
    T12=trasz(-80)
    T23=trasy(-90)
    T34=trasz(80)
    T45=trasy(-90)@rotax(90)@rotaz(t3a)
    T56=trasz(90)
    T67=trasy(-80)
    T78=trasz(90)
    T89=trasy(80)@rotax(-90)@rotaz(t4a)
    T910=trasz(80)
    T1011=trasy(-90)
    T1112=trasz(-80)
    T1213=trasy(-90)@rotax(90)@rotaz(t5a)
    T1314=trasz(90)
    T1415=trasy(80)
    T1516=trasz(90)
    T1617=trasy(-80)@rotax(-90)@rotaz(t6a)
    T1718=trasz(-80)
    T1819=trasy(-87.5)
    T1920=trasz(80)
    T2021=trasy(-87.5)@rotax(90)@rotaz(t7a)
    T02=T01@T12
    T03=T02@T23
    T04=T03@T34
    T05=T04@T45
    T06=T05@T56
    T07=T06@T67
    T08=T07@T78
    T09=T08@T89
    T10=T09@T910
    T11=T10@T1011
    T12=T11@T1112
    T13=T12@T1213
    T14=T13@T1314
    T15=T14@T1415
    T16=T15@T1516
    T17=T16@T1617
    T18=T17@T1718
    T19=T18@T1819
    T20=T19@T1920
    T21=T20@T2021


    Tb01=Tib3@rotax(-90)@rotaz(t2b)
    Tb12=trasz(-80)
    Tb23=trasy(-90)
    Tb34=trasz(80)
    Tb45=trasy(-90)@rotax(90)@rotaz(t3b)
    Tb56=trasz(90)
    Tb67=trasy(-80)
    Tb78=trasz(90)
    Tb89=trasy(80)@rotax(-90)@rotaz(t4b)
    Tb910=trasz(80)
    Tb1011=trasy(-90)
    Tb1112=trasz(-80)
    Tb1213=trasy(-90)@rotax(90)@rotaz(t5b)
    Tb1314=trasz(90)
    Tb1415=trasy(80)
    Tb1516=trasz(90)
    Tb1617=trasy(-80)@rotax(-90)@rotaz(t6b)
    Tb1718=trasz(-80)
    Tb1819=trasy(-87.5)
    Tb1920=trasz(80)
    Tb2021=trasy(-87.5)@rotax(90)@rotaz(t7b)
    
    

    Tb02=Tb01@Tb12
    Tb03=Tb02@Tb23
    Tb04=Tb03@Tb34
    Tb05=Tb04@Tb45
    Tb06=Tb05@Tb56
    Tb07=Tb06@Tb67
    Tb08=Tb07@Tb78
    Tb09=Tb08@Tb89
    Tb10=Tb09@Tb910
    Tb11=Tb10@Tb1011
    Tb12=Tb11@Tb1112
    Tb13=Tb12@Tb1213
    Tb14=Tb13@Tb1314
    Tb15=Tb14@Tb1415
    Tb16=Tb15@Tb1516
    Tb17=Tb16@Tb1617
    Tb18=Tb17@Tb1718
    Tb19=Tb18@Tb1819
    Tb20=Tb19@Tb1920
    Tb21=Tb20@Tb2021
    return T21, Tb21


def motomannewton(vd,vd2,sem,mh=np.eye(4)):
    d=0.01
    calc=True
    j=np.zeros([24,15])
    b=np.ones([24,1])
    tb=sem[0]
    t1=sem[1]
    t2=sem[2]
    t3=sem[3]
    t4=sem[4]
    t5=sem[5]
    t6=sem[6]
    t7=sem[7]
    tb1=sem[8]
    tb2=sem[9]
    tb3=sem[10]
    tb4=sem[11]
    tb5=sem[12]
    tb6=sem[13]
    tb7=sem[14]
    tbd=tb+d
    t1d=t1+d
    t2d=t2+d
    t3d=t3+d
    t4d=t4+d
    t5d=t5+d
    t6d=t6+d
    t7d=t7+d
    tb1d=tb1+d
    tb2d=tb2+d
    tb3d=tb3+d
    tb4d=tb4+d
    tb5d=tb5+d
    tb6d=tb6+d
    tb7d=tb7+d
    con=0
    while calc:
        con=con+1
        t,t2al=accmotomanv(tb,t1,t2,t3,t4,t5,t6,t7,tb1,tb2,tb3,tb4,tb5,tb6,tb7)
        tn=t-vd
        tn2=t2al-vd2
        b[0,0]=tn[0,0]
        b[1,0]=tn[0,1]
        b[2,0]=tn[0,2]
        b[3,0]=tn[0,3]
        b[4,0]=tn[1,0]
        b[5,0]=tn[1,1]
        b[6,0]=tn[1,2]
        b[7,0]=tn[1,3]
        b[8,0]=tn[2,0]
        b[9,0]=tn[2,1]
        b[10,0]=tn[2,2]
        b[11,0]=tn[2,3]
        b[12,0]=tn2[0,0]
        b[13,0]=tn2[0,1]
        b[14,0]=tn2[0,2]
        b[15,0]=tn2[0,3]
        b[16,0]=tn2[1,0]
        b[17,0]=tn2[1,1]
        b[18,0]=tn2[1,2]
        b[19,0]=tn2[1,3]
        b[20,0]=tn2[2,0]
        b[21,0]=tn2[2,1]
        b[22,0]=tn2[2,2]
        b[23,0]=tn2[2,3]
        tv1=[t1d,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1]
        tv2=[t2,t2d,t2,t2,t2,t2,t2,t2,t2,t2,t2,t2,t2,t2,t2]
        tv3=[t3,t3,t3d,t3,t3,t3,t3,t3,t3,t3,t3,t3,t3,t3,t3]
        tv4=[t4,t4,t4,t4d,t4,t4,t4,t4,t4,t4,t4,t4,t4,t4,t4]
        tv5=[t5,t5,t5,t5,t5d,t5,t5,t5,t5,t5,t5,t5,t5,t5,t5]
        tv6=[t6,t6,t6,t6,t6,t6d,t6,t6,t6,t6,t6,t6,t6,t6,t6]
        tv7=[t7,t7,t7,t7,t7,t7,t7d,t7,t7,t7,t7,t7,t7,t7,t7]
        tvb1=[tb1,tb1,tb1,tb1,tb1,tb1,tb1,tb1d,tb1,tb1,tb1,tb1,tb1,tb1,tb1]
        tvb2=[tb2,tb2,tb2,tb2,tb2,tb2,tb2,tb2,tb2d,tb2,tb2,tb2,tb2,tb2,tb2]
        tvb3=[tb3,tb3,tb3,tb3,tb3,tb3,tb3,tb3,tb3,tb3d,tb3,tb3,tb3,tb3,tb3]
        tvb4=[tb4,tb4,tb4,tb4,tb4,tb4,tb4,tb4,tb4,tb4,tb4d,tb4,tb4,tb4,tb4]
        tvb5=[tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5d,tb5,tb5,tb5]
        tvb6=[tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6d,tb6,tb6]
        tvb7=[tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7d,tb7]
        tvb=[tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tbd]
        
        n=0
        while n<15:
            td,td2=accmotomanv(tvb[n],tv1[n],tv2[n],tv3[n],tv4[n],tv5[n],tv6[n],tv7[n],tvb1[n],tvb2[n],tvb3[n],tvb4[n],tvb5[n],tvb6[n],tvb7[n],mh)
            tj=(td-t)/d
            tj2=(td2-t2al)/d
            j[0,n]=tj[0,0]
            j[1,n]=tj[0,1]
            j[2,n]=tj[0,2]
            j[3,n]=tj[0,3]
            j[4,n]=tj[1,0]
            j[5,n]=tj[1,1]
            j[6,n]=tj[1,2]
            j[7,n]=tj[1,3]
            j[8,n]=tj[2,0]
            j[9,n]=tj[2,1]
            j[10,n]=tj[2,2]
            j[11,n]=tj[2,3]
            j[12,n]=tj2[0,0]
            j[13,n]=tj2[0,1]
            j[14,n]=tj2[0,2]
            j[15,n]=tj2[0,3]
            j[16,n]=tj2[1,0]
            j[17,n]=tj2[1,1]
            j[18,n]=tj2[1,2]
            j[19,n]=tj2[1,3]
            j[20,n]=tj2[2,0]
            j[21,n]=tj2[2,1]
            j[22,n]=tj2[2,2]
            j[23,n]=tj2[2,3]
            n=n+1
        R=np.linalg.pinv(j)@(-b)
        t1=t1+R[0,0]
        t2=t2+R[1,0]
        t3=t3+R[2,0]
        t4=t4+R[3,0]
        t5=t5+R[4,0]
        t6=t6+R[5,0]
        t7=t6+R[6,0]
        tb1=tb1+R[7,0]
        tb2=tb2+R[8,0]
        tb3=tb3+R[9,0]
        tb4=tb4+R[10,0]
        tb5=tb5+R[11,0]
        tb6=tb6+R[12,0]
        tb7=tb7+R[13,0]
        tb=tb+R[14,0]
        t1=t1%360
        t2=t2%360
        t3=t3%360
        t4=t4%360
        t5=t5%360
        t6=t6%360
        t7=t7%360
        tb1=tb1%360
        tb2=tb2%360
        tb3=tb3%360
        tb4=tb4%360
        tb5=tb5%360
        tb6=tb6%360
        tb7=tb7%360
        tb=tb%360
        
        
        t1d=t1+d
        t2d=t2+d
        t3d=t3+d
        t4d=t4+d
        t5d=t5+d
        t6d=t6+d
        t7d=t7+d
        tb1d=tb1+d
        tb2d=tb2+d
        tb3d=tb3+d
        tb4d=tb4+d
        tb5d=tb5+d
        tb6d=tb6+d
        tb7d=tb7+d
        tbd=tb+d
        if (abs(b[0,0])<0.0001 and abs(b[1,0])<0.0001 and abs(b[2,0])<0.0001 and abs(b[3,0])<0.0001 and abs(b[4,0])<0.0001 and abs(b[5,0])<0.0001 and abs(b[6,0])<0.0001 and abs(b[7,0])<0.0001 and abs(b[8,0])<0.0001 and abs(b[9,0])<0.0001 and abs(b[10,0])<0.0001 and abs(b[11,0])<0.0001 and abs(b[12,0])<0.0001 and abs(b[13,0])<0.0001 and abs(b[14,0])<0.0001 and abs(b[15,0])<0.0001 and abs(b[16,0])<0.0001 and abs(b[17,0])<0.0001 and abs(b[18,0])<0.0001 and abs(b[19,0])<0.0001 and abs(b[20,0])<0.0001 and abs(b[21,0])<0.0001 and abs(b[22,0])<0.0001 and abs(b[23,0])<0.0001):
            calc=False
        if con>100:
            calc=False
            t1=sem[0]
            t2=sem[1]
            t3=sem[2]
            t4=sem[3]
            t5=sem[4]
            t6=sem[5]
            t7=sem[6]
            tb1=sem[7]
            tb2=sem[8]
            tb3=sem[9]
            tb4=sem[10]
            tb5=sem[11]
            tb6=sem[12]
            tb7=sem[13]
            tb=sem[14]
    tetas=[tb,t1,t2,t3,t4,t5,t6,t7,tb1,tb2,tb3,tb4,tb5,tb6,tb7]
    print(con)
    print(tetas)
    return tetas

def accmotomanv2(tb,t1a,t2a,t3a,t4a,t5a,t6a,t7a,t1b,t2b,t3b,t4b,t5b,t6b,t7b,mh=np.eye(4)):
    T0=np.eye(4)
    Ti=trasz(893.5)
    Ti1=Ti@trasx(92.5)@rotaz(tb)
    Ti2=Ti1@trasx(100)@trasz(306.5)
    Ti3=Ti2@rotax(-90)@rotaz(-180)@rotaz(t1a)@trasz(265)
    Tib3=Ti2@rotax(90)@rotaz(-180)@rotaz(t1b)@trasz(265)
    
    T01=Ti3@rotax(-90)@rotaz(t2a)
    T12=trasz(-80)
    T23=trasy(-90)
    T34=trasz(80)
    T45=trasy(-90)@rotax(90)@rotaz(t3a)
    T56=trasz(90)
    T67=trasy(-80)
    T78=trasz(90)
    T89=trasy(80)@rotax(-90)@rotaz(t4a)
    T910=trasz(80)
    T1011=trasy(-90)
    T1112=trasz(-80)
    T1213=trasy(-90)@rotax(90)@rotaz(t5a)
    T1314=trasz(90)
    T1415=trasy(80)
    T1516=trasz(90)
    T1617=trasy(-80)@rotax(-90)@rotaz(t6a)
    T1718=trasz(-80)
    T1819=trasy(-87.5)
    T1920=trasz(80)
    T2021=trasy(-87.5)@rotax(90)@rotaz(t7a)
    T02=T01@T12
    T03=T02@T23
    T04=T03@T34
    T05=T04@T45
    T06=T05@T56
    T07=T06@T67
    T08=T07@T78
    T09=T08@T89
    T10=T09@T910
    T11=T10@T1011
    T12=T11@T1112
    T13=T12@T1213
    T14=T13@T1314
    T15=T14@T1415
    T16=T15@T1516
    T17=T16@T1617
    T18=T17@T1718
    T19=T18@T1819
    T20=T19@T1920
    T21=T20@T2021


    Tb01=Tib3@rotax(-90)@rotaz(t2b)
    Tb12=trasz(-80)
    Tb23=trasy(-90)
    Tb34=trasz(80)
    Tb45=trasy(-90)@rotax(90)@rotaz(t3b)
    Tb56=trasz(90)
    Tb67=trasy(-80)
    Tb78=trasz(90)
    Tb89=trasy(80)@rotax(-90)@rotaz(t4b)
    Tb910=trasz(80)
    Tb1011=trasy(-90)
    Tb1112=trasz(-80)
    Tb1213=trasy(-90)@rotax(90)@rotaz(t5b)
    Tb1314=trasz(90)
    Tb1415=trasy(80)
    Tb1516=trasz(90)
    Tb1617=trasy(-80)@rotax(-90)@rotaz(t6b)
    Tb1718=trasz(-80)
    Tb1819=trasy(-87.5)
    Tb1920=trasz(80)
    Tb2021=trasy(-87.5)@rotax(90)@rotaz(t7b)
    
    

    Tb02=Tb01@Tb12
    Tb03=Tb02@Tb23
    Tb04=Tb03@Tb34
    Tb05=Tb04@Tb45
    Tb06=Tb05@Tb56
    Tb07=Tb06@Tb67
    Tb08=Tb07@Tb78
    Tb09=Tb08@Tb89
    Tb10=Tb09@Tb910
    Tb11=Tb10@Tb1011
    Tb12=Tb11@Tb1112
    Tb13=Tb12@Tb1213
    Tb14=Tb13@Tb1314
    Tb15=Tb14@Tb1415
    Tb16=Tb15@Tb1516
    Tb17=Tb16@Tb1617
    Tb18=Tb17@Tb1718
    Tb19=Tb18@Tb1819
    Tb20=Tb19@Tb1920
    Tb21=Tb20@Tb2021
    return T21, Tb21

def motomannewton2(vd,vd2,sem,mh=np.eye(4)):
    d=10
    calc=True
    j=np.zeros([24,15])
    b=np.ones([24,1])
    tb=sem[0]
    t1=sem[1]
    t2=sem[2]
    t3=sem[3]
    t4=sem[4]
    t5=sem[5]
    t6=sem[6]
    t7=sem[7]
    tb1=sem[8]
    tb2=sem[9]
    tb3=sem[10]
    tb4=sem[11]
    tb5=sem[12]
    tb6=sem[13]
    tb7=sem[14]
    tbd=tb+d
    t1d=t1+d
    t2d=t2+d
    t3d=t3+d
    t4d=t4+d
    t5d=t5+d
    t6d=t6+d
    t7d=t7+d
    tb1d=tb1+d
    tb2d=tb2+d
    tb3d=tb3+d
    tb4d=tb4+d
    tb5d=tb5+d
    tb6d=tb6+d
    tb7d=tb7+d
    con=0
    while calc:
        con=con+1
        t,t2al=accmotomanv(tb,t1,t2,t3,t4,t5,t6,t7,tb1,tb2,tb3,tb4,tb5,tb6,tb7)
        #print(t)
        #print(t2al)
        #print(vd)
        #print(vd2)
        tn=t-vd
        tn2=t2al-vd2
        #print(tn)
        #print(tn2)
        b[0,0]=tn[0,0]
        b[1,0]=tn[0,1]
        b[2,0]=tn[0,2]
        b[3,0]=tn[0,3]
        b[4,0]=tn[1,0]
        b[5,0]=tn[1,1]
        b[6,0]=tn[1,2]
        b[7,0]=tn[1,3]
        b[8,0]=tn[2,0]
        b[9,0]=tn[2,1]
        b[10,0]=tn[2,2]
        b[11,0]=tn[2,3]
        b[12,0]=tn2[0,0]
        b[13,0]=tn2[0,1]
        b[14,0]=tn2[0,2]
        b[15,0]=tn2[0,3]
        b[16,0]=tn2[1,0]
        b[17,0]=tn2[1,1]
        b[18,0]=tn2[1,2]
        b[19,0]=tn2[1,3]
        b[20,0]=tn2[2,0]
        b[21,0]=tn2[2,1]
        b[22,0]=tn2[2,2]
        b[23,0]=tn2[2,3]
        #print(np.around(b,4))
        tv1=[t1d,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1,t1]
        tv2=[t2,t2d,t2,t2,t2,t2,t2,t2,t2,t2,t2,t2,t2,t2,t2]
        tv3=[t3,t3,t3d,t3,t3,t3,t3,t3,t3,t3,t3,t3,t3,t3,t3]
        tv4=[t4,t4,t4,t4d,t4,t4,t4,t4,t4,t4,t4,t4,t4,t4,t4]
        tv5=[t5,t5,t5,t5,t5d,t5,t5,t5,t5,t5,t5,t5,t5,t5,t5]
        tv6=[t6,t6,t6,t6,t6,t6d,t6,t6,t6,t6,t6,t6,t6,t6,t6]
        tv7=[t7,t7,t7,t7,t7,t7,t7d,t7,t7,t7,t7,t7,t7,t7,t7]
        tvb1=[tb1,tb1,tb1,tb1,tb1,tb1,tb1,tb1d,tb1,tb1,tb1,tb1,tb1,tb1,tb1]
        tvb2=[tb2,tb2,tb2,tb2,tb2,tb2,tb2,tb2,tb2d,tb2,tb2,tb2,tb2,tb2,tb2]
        tvb3=[tb3,tb3,tb3,tb3,tb3,tb3,tb3,tb3,tb3,tb3d,tb3,tb3,tb3,tb3,tb3]
        tvb4=[tb4,tb4,tb4,tb4,tb4,tb4,tb4,tb4,tb4,tb4,tb4d,tb4,tb4,tb4,tb4]
        tvb5=[tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5,tb5d,tb5,tb5,tb5]
        tvb6=[tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6,tb6d,tb6,tb6]
        tvb7=[tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7,tb7d,tb7]
        tvb=[tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tb,tbd]
        
        n=0
        while n<15:
            td,td2=accmotomanv(tvb[n],tv1[n],tv2[n],tv3[n],tv4[n],tv5[n],tv6[n],tv7[n],tvb1[n],tvb2[n],tvb3[n],tvb4[n],tvb5[n],tvb6[n],tvb7[n],mh)
            tj=(td-t)/d
            tj2=(td2-t2al)/d
            j[0,n]=tj[0,0]
            j[1,n]=tj[0,1]
            j[2,n]=tj[0,2]
            j[3,n]=tj[0,3]
            j[4,n]=tj[1,0]
            j[5,n]=tj[1,1]
            j[6,n]=tj[1,2]
            j[7,n]=tj[1,3]
            j[8,n]=tj[2,0]
            j[9,n]=tj[2,1]
            j[10,n]=tj[2,2]
            j[11,n]=tj[2,3]
            j[12,n]=tj2[0,0]
            j[13,n]=tj2[0,1]
            j[14,n]=tj2[0,2]
            j[15,n]=tj2[0,3]
            j[16,n]=tj2[1,0]
            j[17,n]=tj2[1,1]
            j[18,n]=tj2[1,2]
            j[19,n]=tj2[1,3]
            j[20,n]=tj2[2,0]
            j[21,n]=tj2[2,1]
            j[22,n]=tj2[2,2]
            j[23,n]=tj2[2,3]
            n=n+1
        R=np.linalg.pinv(j)@(-b)
        #print(np.around(j,4))
        #print(np.around(R,4))
        t1=t1+R[0,0]
        t2=t2+R[1,0]
        t3=t3+R[2,0]
        t4=t4+R[3,0]
        t5=t5+R[4,0]
        t6=t6+R[5,0]
        t7=t7+R[6,0]
        tb1=tb1+R[7,0]
        tb2=tb2+R[8,0]
        tb3=tb3+R[9,0]
        tb4=tb4+R[10,0]
        tb5=tb5+R[11,0]
        tb6=tb6+R[12,0]
        tb7=tb7+R[13,0]
        tb=tb+R[14,0]
        t1=t1%360
        t2=t2%360
        t3=t3%360
        t4=t4%360
        t5=t5%360
        t6=t6%360
        t7=t7%360
        tb1=tb1%360
        tb2=tb2%360
        tb3=tb3%360
        tb4=tb4%360
        tb5=tb5%360
        tb6=tb6%360
        tb7=tb7%360
        tb=tb%360
        if tb>180:
            tb=-(360-tb)
        if tb<-180:
            tb=360+tb
            
        if t1>180:
            t1=-(360-t1)
        if t1<-180:
            t1=360+t1
            
        if t2>180:
            t2=-(360-t2)
        if t2<-180:
            t2=360+t2
            
        if t3>180:
            t3=-(360-t3)
        if t3<-180:
            t3=360+t3
            
        if t4>180:
            t4=-(360-t4)
        if t4<-180:
            t4=360+t4
            
        if t5>180:
            t5=-(360-t5)
        if t5<-180:
            t5=360+t5
            
        if t6>180:
            t6=-(360-t6)
        if t6<-180:
            t6=360+t6
            
        if t7>180:
            t7=-(360-t7)
        if t7<-180:
            t7=360+t7

        if tb1>180:
            tb1=-(360-tb1)
        if tb1<-180:
            tb1=360+tb1
            
        if tb2>180:
            tb2=-(360-tb2)
        if tb2<-180:
            tb2=360+tb2
            
        if tb3>180:
            tb3=-(360-tb3)
        if tb3<-180:
            tb3=360+tb3
            
        if tb4>180:
            tb4=-(360-tb4)
        if tb4<-180:
            tb4=360+tb4
            
        if tb5>180:
            tb5=-(360-tb5)
        if tb5<-180:
            tb5=360+tb5
            
        if tb6>180:
            tb6=-(360-tb6)
        if tb6<-180:
            tb6=360+tb6
            
        if tb7>180:
            tb7=-(360-tb7)
        if tb7<-180:
            tb7=360+tb7
        
        
        t1d=t1+d
        t2d=t2+d
        t3d=t3+d
        t4d=t4+d
        t5d=t5+d
        t6d=t6+d
        t7d=t7+d
        tb1d=tb1+d
        tb2d=tb2+d
        tb3d=tb3+d
        tb4d=tb4+d
        tb5d=tb5+d
        tb6d=tb6+d
        tb7d=tb7+d
        tbd=tb+d
        if (abs(b[0,0])<0.0001 and abs(b[1,0])<0.0001 and abs(b[2,0])<0.0001 and abs(b[3,0])<0.0001 and abs(b[4,0])<0.0001 and abs(b[5,0])<0.0001 and abs(b[6,0])<0.0001 and abs(b[7,0])<0.0001 and abs(b[8,0])<0.0001 and abs(b[9,0])<0.0001 and abs(b[10,0])<0.0001 and abs(b[11,0])<0.0001 and abs(b[12,0])<0.0001 and abs(b[13,0])<0.0001 and abs(b[14,0])<0.0001 and abs(b[15,0])<0.0001 and abs(b[16,0])<0.0001 and abs(b[17,0])<0.0001 and abs(b[18,0])<0.0001 and abs(b[19,0])<0.0001 and abs(b[20,0])<0.0001 and abs(b[21,0])<0.0001 and abs(b[22,0])<0.0001 and abs(b[23,0])<0.0001):
            calc=False
        if con>100:
            calc=False
            t1=sem[1]
            t2=sem[2]
            t3=sem[3]
            t4=sem[4]
            t5=sem[5]
            t6=sem[6]
            t7=sem[7]
            tb1=sem[8]
            tb2=sem[9]
            tb3=sem[10]
            tb4=sem[11]
            tb5=sem[12]
            tb6=sem[13]
            tb7=sem[14]
            tb=sem[0]
    tetas=[tb,t1,t2,t3,t4,t5,t6,t7,tb1,tb2,tb3,tb4,tb5,tb6,tb7]
    print(con)
    print(tetas)
    return tetas

def motomanmovej(p1,p2):
    n=1
    y=0
    while y<15:
        if p1[y]>180:
            p1[y]=-(360-p1[y])
        if p1[y]<-180:
            p1[y]=360+p1[y]
        y=y+1
    y=0
    while y<15:
        if p2[y]>180:
            p2[y]=-(360-p2[y])
        if p2[y]<-180:
            p2[y]=360+p2[y]
        y=y+1
            
    tetas=[p1[0],p1[1],p1[2],p1[3],p1[4],p1[5],p1[6],p1[7],p1[8],p1[9],p1[10],p1[11],p1[12],p1[13],p1[14]]
    paso=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    paso[0]=(p2[0]-p1[0])/30
    paso[1]=(p2[1]-p1[1])/30
    paso[2]=(p2[2]-p1[2])/30
    paso[3]=(p2[3]-p1[3])/30
    paso[4]=(p2[4]-p1[4])/30
    paso[5]=(p2[5]-p1[5])/30
    paso[6]=(p2[6]-p1[6])/30
    paso[7]=(p2[7]-p1[7])/30
    paso[8]=(p2[8]-p1[8])/30
    paso[9]=(p2[9]-p1[9])/30
    paso[10]=(p2[10]-p1[10])/30
    paso[11]=(p2[11]-p1[11])/30
    paso[12]=(p2[12]-p1[12])/30
    paso[13]=(p2[13]-p1[13])/30
    paso[14]=(p2[14]-p1[14])/30
    while n<31:
        tetas[0]=tetas[0]+paso[0]
        tetas[1]=tetas[1]+paso[1]
        tetas[2]=tetas[2]+paso[2]
        tetas[3]=tetas[3]+paso[3]
        tetas[4]=tetas[4]+paso[4]
        tetas[5]=tetas[5]+paso[5]
        tetas[6]=tetas[6]+paso[6]
        tetas[7]=tetas[7]+paso[7]
        tetas[8]=tetas[8]+paso[8]
        tetas[9]=tetas[9]+paso[9]
        tetas[10]=tetas[10]+paso[10]
        tetas[11]=tetas[11]+paso[11]
        tetas[12]=tetas[12]+paso[12]
        tetas[13]=tetas[13]+paso[13]
        tetas[14]=tetas[14]+paso[14]
        n=n+1
        ax.cla()
        setaxis(1000)
        #print(tetar)
        accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
        dibujar()
    return tetas

def motomanmovel(tetas,tetas2):
    con=0
    n=0
    sem=tetas.copy()
    mtr1,mtr2=accmotomanv(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
    mtr12,mtr22=accmotomanv(tetas2[0],tetas2[1],tetas2[2],tetas2[3],tetas2[4],tetas2[5],tetas2[6],tetas2[7],tetas2[8],tetas2[9],tetas2[10],tetas2[11],tetas2[12],tetas2[13],tetas2[14])
    pn=mtr1
    pn2=mtr2
    p1xd=mtr1[0,3]
    p1yd=mtr1[1,3]
    p1zd=mtr1[2,3]
    p1xi=mtr2[0,3]
    p1yi=mtr2[1,3]
    p1zi=mtr2[2,3]
    p2xd=mtr12[0,3]
    p2yd=mtr12[1,3]
    p2zd=mtr12[2,3]
    p2xi=mtr22[0,3]
    p2yi=mtr22[1,3]
    p2zi=mtr22[2,3]
    dxd=p2xd-p1xd
    dyd=p2yd-p1yd 
    dzd=p2zd-p1zd
    dxi=p2xi-p1xi
    dyi=p2yi-p1yi 
    dzi=p2zi-p1zi
    angrot1=obtangrot(mtr12,mtr1)
    angrot1[0]=angrot1[0]/30
    angrot1[1]=angrot1[1]/30
    angrot1[2]=angrot1[2]/30
    angroti1=[0,0,0]

    angrot2=obtangrot(mtr22,mtr2)
    angrot2[0]=angrot2[0]/30
    angrot2[1]=angrot2[1]/30
    angrot2[2]=angrot2[2]/30
    angroti2=[0,0,0]
    while n<1:
        pn=mtr1@rotaz(angroti1[2])@rotay(angroti1[1])@rotax(angroti1[0])
        angroti1[0]=angroti1[0]+angrot1[0]
        angroti1[1]=angroti1[1]+angrot1[1]
        angroti1[2]=angroti1[2]+angrot1[2]
        xd=p1xd+dxd*n
        yd=p1yd+dyd*n
        zd=p1zd+dzd*n

        pn2=mtr1@rotaz(angroti2[2])@rotay(angroti2[1])@rotax(angroti2[0])
        angroti2[0]=angroti2[0]+angrot2[0]
        angroti2[1]=angroti2[1]+angrot2[1]
        angroti2[2]=angroti2[2]+angrot2[2]
        xi=p1xi+dxi*n
        yi=p1yi+dyi*n
        zi=p1zi+dzi*n
        n+=1/30
        con+=1
        pn[0,3]=xd
        pn[1,3]=yd
        pn[2,3]=zd
        pn2[0,3]=xi
        pn2[1,3]=yi
        pn2[2,3]=zi
        tetas=motomannewton2(pn,pn2,sem)
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
        ax.cla()
        setaxis(1000)
        accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
        dibujar()
    return tetas

def IRB1600(t1,t2,t3,t4,t5,t6):
    t0=np.eye(4)
    t01=rotaz(t1)#
    t12=trasz(486.5)@rotax(-90)#
    t23=trasx(150)@rotaz(t2)
    t34=trasy(-700)@rotaz(90)@rotaz(t3)#
    t45=trasx(-110)@rotax(90)@rotaz(t4)#
    t56=trasz(640)@rotax(-90)@rotaz(t5)#
    t67=trasy(-150)@rotax(90)@rotaz(t6)
    t78=trasz(50)#
    t02=t01@t12
    t03=t02@t23
    t04=t03@t34
    t05=t04@t45
    t06=t05@t56
    t07=t06@t67
    t08=t07@t78
    sistemafijo(100)
    sistemamovil(t01,100)
    #sistemamovil(t02,100)
    sistemamovil(t03,100)
    sistemamovil(t04,100)
    sistemamovil(t05,100)
    sistemamovil(t06,100)
    sistemamovil(t07,100)
    #sistemamovil(t08,100)
    ax.plot3D([t0[0,3],t01[0,3]],[t0[1,3],t01[1,3]],[t0[2,3],t01[2,3]],color='red')
    ax.plot3D([t01[0,3],t02[0,3]],[t01[1,3],t02[1,3]],[t01[2,3],t02[2,3]],color='red')
    ax.plot3D([t02[0,3],t03[0,3]],[t02[1,3],t03[1,3]],[t02[2,3],t03[2,3]],color='red')
    ax.plot3D([t03[0,3],t04[0,3]],[t03[1,3],t04[1,3]],[t03[2,3],t04[2,3]],color='red')
    ax.plot3D([t04[0,3],t05[0,3]],[t04[1,3],t05[1,3]],[t04[2,3],t05[2,3]],color='red')
    ax.plot3D([t05[0,3],t06[0,3]],[t05[1,3],t06[1,3]],[t05[2,3],t06[2,3]],color='red')
    ax.plot3D([t06[0,3],t07[0,3]],[t06[1,3],t07[1,3]],[t06[2,3],t07[2,3]],color='red')
    ax.plot3D([t07[0,3],t08[0,3]],[t07[1,3],t08[1,3]],[t07[2,3],t08[2,3]],color='red')
    return(t08)



def IRB1600v(t1,t2,t3,t4,t5,t6,mh=np.eye(4)):
    t0=np.eye(4)
    t01=rotaz(t1)#
    t12=trasz(486.5)@rotax(-90)#
    t23=trasx(150)@rotaz(t2)
    t34=trasy(-700)@rotaz(90)@rotaz(t3)#
    t45=trasx(-110)@rotax(90)@rotaz(t4)#
    t56=trasz(640)@rotax(-90)@rotaz(t5)#
    t67=trasy(-150)@rotax(90)@rotaz(t6)
    t78=trasz(50)#
    t02=t01@t12
    t03=t02@t23
    t04=t03@t34
    t05=t04@t45
    t06=t05@t56
    t07=t06@t67
    t08=t07@t78
    t08=t08@mh
    #print(t08)
    return t08

def IRB1600newton(vd,sem,mh=np.eye(4)):
    d=0.01
    calc=True
    j=np.zeros([12,6])
    b=np.ones([12,1])
    t1=sem[0]
    t2=sem[1]
    t3=sem[2]
    t4=sem[3]
    t5=sem[4]
    t6=sem[5]
    t1d=t1+d
    t2d=t2+d
    t3d=t3+d
    t4d=t4+d
    t5d=t5+d
    t6d=t6+d
    con=0
    while calc:
        con=con+1
        t=IRB1600v(t1,t2,t3,t4,t5,t6,mh)
        tn=t-vd
        b[0,0]=tn[0,0]
        b[1,0]=tn[0,1]
        b[2,0]=tn[0,2]
        b[3,0]=tn[0,3]
        b[4,0]=tn[1,0]
        b[5,0]=tn[1,1]
        b[6,0]=tn[1,2]
        b[7,0]=tn[1,3]
        b[8,0]=tn[2,0]
        b[9,0]=tn[2,1]
        b[10,0]=tn[2,2]
        b[11,0]=tn[2,3]
        tv1=[t1d,t1,t1,t1,t1,t1]
        tv2=[t2,t2d,t2,t2,t2,t2]
        tv3=[t3,t3,t3d,t3,t3,t3]
        tv4=[t4,t4,t4,t4d,t4,t4]
        tv5=[t5,t5,t5,t5,t5d,t5]
        tv6=[t6,t6,t6,t6,t6,t6d]
        n=0
        while n<6:
            td=IRB1600v(tv1[n],tv2[n],tv3[n],tv4[n],tv5[n],tv6[n],mh)
            tj=(td-t)/d
            j[0,n]=tj[0,0]
            j[1,n]=tj[0,1]
            j[2,n]=tj[0,2]
            j[3,n]=tj[0,3]
            j[4,n]=tj[1,0]
            j[5,n]=tj[1,1]
            j[6,n]=tj[1,2]
            j[7,n]=tj[1,3]
            j[8,n]=tj[2,0]
            j[9,n]=tj[2,1]
            j[10,n]=tj[2,2]
            j[11,n]=tj[2,3]
            n=n+1
        R=np.linalg.pinv(j)@(-b)
        t1=t1+R[0,0]
        t2=t2+R[1,0]
        t3=t3+R[2,0]
        t4=t4+R[3,0]
        t5=t5+R[4,0]
        t6=t6+R[5,0]
        t1=t1%360
        t2=t2%360
        t3=t3%360
        t4=t4%360
        t5=t5%360
        t6=t6%360
        if t1>180:
            t1=-(360-t1)
        if t1<-180:
            t1=360+t1
        if t2>150:
            t2=-(360-t2)
        if t2<-90:
            t2=360+t2
        if t3>79:
            t3=-(360-t3)
        if t3<-238:
            t3=360+t3
        if t4>155:
            t4=-(360-t4)
        if t4<-155:
            t4=360+t4
        if t5>135:
            t5=-(360-t5)
        if t5<-90:
            t5=360+t5
        if t6>200:
            t6=-(360-t6)
        if t6<-200:
            t6=360+t6
        if t2>150 or t2<-90:
            if (abs(b[0,0])<0.1 and abs(b[1,0])<0.1 and abs(b[2,0])<0.1 and abs(b[3,0])<0.1 and abs(b[4,0])<0.1 and abs(b[5,0])<0.1 and abs(b[6,0])<0.1 and abs(b[7,0])<0.1 and abs(b[8,0])<0.1 and abs(b[9,0])<0.1 and abs(b[10,0])<0.1 and abs(b[11,0])<0.1):
                                 
                print("**********")
                print("**********")
                print("T2 fuera de limite")
                print("**********")
                print("**********")
                t2=t2+100
                b[0,0]=1
        if t3>79 or t3<-238:
            if (abs(b[0,0])<0.1 and abs(b[1,0])<0.1 and abs(b[2,0])<0.1 and abs(b[3,0])<0.1 and abs(b[4,0])<0.1 and abs(b[5,0])<0.1 and abs(b[6,0])<0.1 and abs(b[7,0])<0.1 and abs(b[8,0])<0.1 and abs(b[9,0])<0.1 and abs(b[10,0])<0.1 and abs(b[11,0])<0.1):

                print("**********")
                print("**********")
                print("T3 fuera de limite")
                print("**********")
                print("**********")
                t3=t3+100
                b[0,0]=1
        if t4>155 or t4<-155:
            if (abs(b[0,0])<0.1 and abs(b[1,0])<0.1 and abs(b[2,0])<0.1 and abs(b[3,0])<0.1 and abs(b[4,0])<0.1 and abs(b[5,0])<0.1 and abs(b[6,0])<0.1 and abs(b[7,0])<0.1 and abs(b[8,0])<0.1 and abs(b[9,0])<0.1 and abs(b[10,0])<0.1 and abs(b[11,0])<0.1):

                print("**********")
                print("**********")
                print("T4 fuera de limite")
                print("**********")
                print("**********")
                t4=t4+100
                b[0,0]=1
                print()
        if t5>135 or t5<-90:
            if (abs(b[0,0])<0.1 and abs(b[1,0])<0.1 and abs(b[2,0])<0.1 and abs(b[3,0])<0.1 and abs(b[4,0])<0.1 and abs(b[5,0])<0.1 and abs(b[6,0])<0.1 and abs(b[7,0])<0.1 and abs(b[8,0])<0.1 and abs(b[9,0])<0.1 and abs(b[10,0])<0.1 and abs(b[11,0])<0.1):

                print("**********")
                print("**********")
                print("T5 fuera de limite")
                print("**********")
                print("**********")
                t5=t5+100
                b[0,0]=1
        
        t1d=t1+d
        t2d=t2+d
        t3d=t3+d
        t4d=t4+d
        t5d=t5+d
        t6d=t6+d
        if (abs(b[0,0])<0.0001 and abs(b[1,0])<0.0001 and abs(b[2,0])<0.0001 and abs(b[3,0])<0.0001 and abs(b[4,0])<0.0001 and abs(b[5,0])<0.0001 and abs(b[6,0])<0.0001 and abs(b[7,0])<0.0001 and abs(b[8,0])<0.0001 and abs(b[9,0])<0.0001 and abs(b[10,0])<0.0001 and abs(b[11,0])<0.0001):
            calc=False
        if con>100:
            calc=False
            t1=sem[0]
            t2=sem[1]
            t3=sem[2]
            t4=sem[3]
            t5=sem[4]
            t6=sem[5]
    tetas=[t1,t2,t3,t4,t5,t6]
    print(con)
    print(tetas)
    return tetas
            

def ur5v(t1,t2,t3,t4,t5,t6,mh=np.eye(4),mb=np.eye(4)):
    t0=np.eye(4)
    t01=mb@rotaz(t1)@trasz(89.2)#
    t12=trasy(-134.2)@rotax(90)@rotaz(t2)#
    t23=trasy(425)
    t34=trasz(-118.45)@rotaz(t3)#
    t45=trasx(392.25)@rotaz(t4)#
    t56=trasz(94.75)@rotax(-90)@rotaz(t5)#
    t67=trasz(94.75)
    t78=trasx(82.5)@rotay(90)@rotaz(-90)@rotaz(t6)#
    t02=t01@t12
    t03=t02@t23
    t04=t03@t34
    t05=t04@t45
    t06=t05@t56
    t07=t06@t67
    t08=t07@t78
    t08=t08@mh
    return t08

def ur5newton(vd,sem,mh=np.eye(4),mb=np.eye(4)):
    d=0.01
    calc=True
    j=np.zeros([12,6])
    b=np.ones([12,1])
    t1=sem[0]
    t2=sem[1]
    t3=sem[2]
    t4=sem[3]
    t5=sem[4]
    t6=sem[5]
    t1d=t1+d
    t2d=t2+d
    t3d=t3+d
    t4d=t4+d
    t5d=t5+d
    t6d=t6+d
    con=0
    while calc:
        con=con+1
        t=ur5v(t1,t2,t3,t4,t5,t6,mh,mb)
        tn=t-vd
        b[0,0]=tn[0,0]
        b[1,0]=tn[0,1]
        b[2,0]=tn[0,2]
        b[3,0]=tn[0,3]
        b[4,0]=tn[1,0]
        b[5,0]=tn[1,1]
        b[6,0]=tn[1,2]
        b[7,0]=tn[1,3]
        b[8,0]=tn[2,0]
        b[9,0]=tn[2,1]
        b[10,0]=tn[2,2]
        b[11,0]=tn[2,3]
        tv1=[t1d,t1,t1,t1,t1,t1]
        tv2=[t2,t2d,t2,t2,t2,t2]
        tv3=[t3,t3,t3d,t3,t3,t3]
        tv4=[t4,t4,t4,t4d,t4,t4]
        tv5=[t5,t5,t5,t5,t5d,t5]
        tv6=[t6,t6,t6,t6,t6,t6d]
        n=0
        while n<6:
            td=ur5v(tv1[n],tv2[n],tv3[n],tv4[n],tv5[n],tv6[n],mh,mb)
            tj=(td-t)/d
            j[0,n]=tj[0,0]
            j[1,n]=tj[0,1]
            j[2,n]=tj[0,2]
            j[3,n]=tj[0,3]
            j[4,n]=tj[1,0]
            j[5,n]=tj[1,1]
            j[6,n]=tj[1,2]
            j[7,n]=tj[1,3]
            j[8,n]=tj[2,0]
            j[9,n]=tj[2,1]
            j[10,n]=tj[2,2]
            j[11,n]=tj[2,3]
            n=n+1
        R=np.linalg.pinv(j)@(-b)
        t1=t1+R[0,0]
        t2=t2+R[1,0]
        t3=t3+R[2,0]
        t4=t4+R[3,0]
        t5=t5+R[4,0]
        t6=t6+R[5,0]
        t1=t1%360
        t2=t2%360
        t3=t3%360
        t4=t4%360
        t5=t5%360
        t6=t6%360
        if t1>180:
            t1=-(360-t1)
        if t1<-180:
            t1=360+t1

        if t2>180:
            t2=-(360-t2)
        if t2<-180:
            t2=360+t2

        if t3>180:
            t3=-(360-t3)
        if t3<-180:
            t3=360+t3

        if t4>180:
            t4=-(360-t4)
        if t4<-180:
            t4=360+t4

        if t5>180:
            t5=-(360-t5)
        if t5<-180:
            t5=360+t5

        if t6>180:
            t6=-(360-t6)
        if t6<-180:
            t6=360+t6
        t1d=t1+d
        t2d=t2+d
        t3d=t3+d
        t4d=t4+d
        t5d=t5+d
        t6d=t6+d
        if (abs(b[0,0])<0.0001 and abs(b[1,0])<0.0001 and abs(b[2,0])<0.0001 and abs(b[3,0])<0.0001 and abs(b[4,0])<0.0001 and abs(b[5,0])<0.0001 and abs(b[6,0])<0.0001 and abs(b[7,0])<0.0001 and abs(b[8,0])<0.0001 and abs(b[9,0])<0.0001 and abs(b[10,0])<0.0001 and abs(b[11,0])<0.0001):
            calc=False
        if con>100:
            calc=False
            t1=sem[0]
            t2=sem[1]
            t3=sem[2]
            t4=sem[3]
            t5=sem[4]
            t6=sem[5]
    tetas=[t1,t2,t3,t4,t5,t6]
    print(con)
    print(tetas)
    return tetas

def obtangrotv(t1,t2,t3,p1ea):
    #t0=rotax(p1ea[0])@rotay(p1ea[1])@rotaz(p1ea[2])
    t=p1ea@rotaz(t3)@rotay(t2)@rotax(t1)
    return t

def obtangrot(p2,p1ea):
    d=0.01
    calc=True
    j=np.zeros([9,3])
    b=np.ones([9,1])
    t1=0
    t2=0
    t3=0
    t1d=t1+d
    t2d=t2+d
    t3d=t3+d
    con=0
    while calc:
        con=con+1
        t=obtangrotv(t1,t2,t3,p1ea)
        tn=t-p2
        b[0,0]=tn[0,0]
        b[1,0]=tn[0,1]
        b[2,0]=tn[0,2]
        b[3,0]=tn[1,0]
        b[4,0]=tn[1,1]
        b[5,0]=tn[1,2]
        b[6,0]=tn[2,0]
        b[7,0]=tn[2,1]
        b[8,0]=tn[2,2]
        tv1=[t1d,t1,t1]
        tv2=[t2,t2d,t2]
        tv3=[t3,t3,t3d]
        n=0
        while n<3:
            td=obtangrotv(tv1[n],tv2[n],tv3[n],p1ea)
            tj=(td-t)/d
            j[0,n]=tj[0,0]
            j[1,n]=tj[0,1]
            j[2,n]=tj[0,2]
            j[3,n]=tj[1,0]
            j[4,n]=tj[1,1]
            j[5,n]=tj[1,2]
            j[6,n]=tj[2,0]
            j[7,n]=tj[2,1]
            j[8,n]=tj[2,2]
            n=n+1
        R=np.linalg.pinv(j)@(-b)
        t1=t1+R[0,0]
        t2=t2+R[1,0]
        t3=t3+R[2,0]
        t1=t1%360
        t2=t2%360
        t3=t3%360
        if t1>180:
            t1=-(360-t1)
        if t1<-180:
            t1=360+t1

        if t2>180:
            t2=-(360-t2)
        if t2<-180:
            t2=360+t2

        if t3>180:
            t3=-(360-t3)
        if t3<-180:
            t3=360+t3

        t1d=t1+d
        t2d=t2+d
        t3d=t3+d
        if (abs(b[0,0])<0.0001 and abs(b[1,0])<0.0001 and abs(b[2,0])<0.0001 and abs(b[3,0])<0.0001 and abs(b[4,0])<0.0001 and abs(b[5,0])<0.0001 and abs(b[6,0])<0.0001 and abs(b[7,0])<0.0001 and abs(b[8,0])<0.0001):
            calc=False
        if con>100:
            calc=False
            t1=sem[0]
            t2=sem[1]
            t3=sem[2]
    tetas=[t1,t2,t3]
    print(con)
    print(tetas)
    return tetas

def herramienta1v(z=0,so=6.4):
    t0=np.eye(4)
    t01=rotaz(z)@trasz(100)#
    t12=trasy(-39)#
    t23=trasz(30)
    t34=rotax(-6.21)@trasy(-79)#
    t45=trasz(225)#
    t56=rotax(-45)@trasz(111)#
    t67=trasz(so)
    t02=t01@t12
    t03=t02@t23
    t04=t03@t34
    t05=t04@t45
    t06=t05@t56
    t07=t06@t67
    return t07

def herramienta1(mr,z=0,so=6.4):
    t0=np.eye(4)
    t01=mr@rotaz(z)@trasz(100)#
    t12=trasy(-39)#
    t23=trasz(30)
    t34=rotax(-6.21)@trasy(-79)#
    t45=trasz(225)#
    t56=rotax(-45)@trasz(111)#
    t67=trasz(so)
    t02=t01@t12
    t03=t02@t23
    t04=t03@t34
    t05=t04@t45
    t06=t05@t56
    t07=t06@t67
    sistemamovil(t01,100)
    sistemamovil(t02,100)
    sistemamovil(t03,100)
    sistemamovil(t04,100)
    sistemamovil(t05,100)
    sistemamovil(t06,100)
    sistemamovil(t07,100)
    ax.plot3D([mr[0,3],t01[0,3]],[mr[1,3],t01[1,3]],[mr[2,3],t01[2,3]],color='red')
    ax.plot3D([t01[0,3],t02[0,3]],[t01[1,3],t02[1,3]],[t01[2,3],t02[2,3]],color='red')
    ax.plot3D([t02[0,3],t03[0,3]],[t02[1,3],t03[1,3]],[t02[2,3],t03[2,3]],color='red')
    ax.plot3D([t03[0,3],t04[0,3]],[t03[1,3],t04[1,3]],[t03[2,3],t04[2,3]],color='red')
    ax.plot3D([t04[0,3],t05[0,3]],[t04[1,3],t05[1,3]],[t04[2,3],t05[2,3]],color='red')
    ax.plot3D([t05[0,3],t06[0,3]],[t05[1,3],t06[1,3]],[t05[2,3],t06[2,3]],color='red')
    ax.plot3D([t06[0,3],t07[0,3]],[t06[1,3],t07[1,3]],[t06[2,3],t07[2,3]],color='red')


def animsistemamovilx(t):
    n=0
    
    while n<t:
       ax.cla() 
       setaxis(-1,1,-1,1,-1,1)
       r=rotax(n)
       sistemafijo()
       sistemamovil(r)
       n=n+1
       dibujar()

def animsistemamovily(t):
    n=0

    while n<t:
        ax.cla()
        setaxis(-1,1,-1,1,-1,1)
        r=rotay(n)
        sistemafijo()
        sistemamovil(r)
        n=n+1
        dibujar()
    
def animsistemamovilz(t):
    n=0

    while n<t:
        ax.cla()
        setaxis()
        r=rotaz(n)
        sistemafijo()
        sistemamovil(r)
        n=n+1
        dibujar()


def muevemoscax(t): 
    n=0
    while n<t:
        ax.cla()
        setaxis()
        r=rotax(n)
        ax.scatter(0,0.4,0.6,'o')
        Auvw=np.array([[0],[0.4],[0.6]])
        Axyz=np.dot(r,Auvw)
        x=Axyz[0,0]
        y=Axyz[1,0]
        z=Axyz[2,0]
        sistemafijo()
        sistemamovil(r)
        ax.scatter(x,y,z,'o')
        n=n+1
        dibujar()

def muevemoscay(t):
    n=0
    while n<t:
        ax.cla()
        setaxis()
        r=rotay(n)
        ax.scatter(0,0.4,0.6,'o')
        Auvw=np.array([[0],[0.4],[0.6]])
        Axyz=np.dot(r,Auvw)
        x=Axyz[0,0]
        y=Axyz[1,0]
        z=Axyz[2,0]
        sistemafijo()
        sistemamovil(r)
        ax.scatter(x,y,z,'o')
        n=n+1
        dibujar()

def muevemoscaz(t):
    n=0
    while n<t:
        ax.cla()
        setaxis()
        r=rotaz(n)
        ax.scatter(0,0.4,0.6,'o')
        Auvw=np.array([[0],[0.4],[0.6]])
        Axyz=np.dot(r,Auvw)
        x=Axyz[0,0]
        y=Axyz[1,0]
        z=Axyz[2,0]
        sistemafijo()
        sistemamovil(r)
        ax.scatter(x,y,z,'o')
        n=n+1
        dibujar()

def dibujarcaja(d=1,w=1,l=1,r=0):
    #setaxis()
    a1=np.array([[0],[0],[0],[1]], dtype=object)
    b1=np.array([[0],[0],[l],[1]], dtype=object)
    c1=np.array([[0],[w],[l],[1]], dtype=object)
    d1=np.array([[0],[w],[0],[1]], dtype=object)
    e1=np.array([[d],[0],[0],[1]], dtype=object)
    f1=np.array([[d],[0],[l],[1]], dtype=object)
    g1=np.array([[d],[w],[l],[1]], dtype=object)
    h1=np.array([[d],[w],[0],[1]], dtype=object)
    a=np.dot(r,a1)
    b=np.dot(r,b1)
    c=np.dot(r,c1)
    d=np.dot(r,d1)
    e=np.dot(r,e1)
    f=np.dot(r,f1)
    g=np.dot(r,g1)
    h=np.dot(r,h1)
    ax.plot3D([a[0,0],b[0,0]],[a[1,0],b[1,0]],[a[2,0],b[2,0]],color='red') 
    ax.plot3D([a[0,0],d[0,0]],[a[1,0],d[1,0]],[a[2,0],d[2,0]],color='red') 
    ax.plot3D([a[0,0],e[0,0]],[a[1,0],e[1,0]],[a[2,0],e[2,0]],color='red')
    ax.plot3D([b[0,0],c[0,0]],[b[1,0],c[1,0]],[b[2,0],c[2,0]],color='red')
    ax.plot3D([b[0,0],f[0,0]],[b[1,0],f[1,0]],[b[2,0],f[2,0]],color='red')
    ax.plot3D([c[0,0],d[0,0]],[c[1,0],d[1,0]],[c[2,0],d[2,0]],color='red')
    ax.plot3D([c[0,0],g[0,0]],[c[1,0],g[1,0]],[c[2,0],g[2,0]],color='red')
    ax.plot3D([d[0,0],h[0,0]],[d[1,0],h[1,0]],[d[2,0],h[2,0]],color='red')
    ax.plot3D([e[0,0],h[0,0]],[e[1,0],h[1,0]],[e[2,0],h[2,0]],color='red')
    ax.plot3D([e[0,0],f[0,0]],[e[1,0],f[1,0]],[e[2,0],f[2,0]],color='red')
    ax.plot3D([g[0,0],f[0,0]],[g[1,0],f[1,0]],[g[2,0],f[2,0]],color='red')
    ax.plot3D([g[0,0],h[0,0]],[g[1,0],h[1,0]],[g[2,0],h[2,0]],color='red')

    
def animcajax(t):
    n=0
    while n<t:
        ax.cla()
        setaxis()
        r=rotax(n)
        dibujarcaja(r=r)
        n=n+1
        sistemafijo()
        dibujar()

def animcajay(t):
    n=0
    while n<t:
        ax.cla()
        setaxis()
        r=rotay(n)
        dibujarcaja(r=r)
        n=n+1
        sistemafijo()
        dibujar()

def animcajaz(t):
    n=0
    while n<t:
        ax.cla()
        setaxis()
        r=rotaz(n)
        dibujarcaja(r=r)
        n=n+1
        sistemafijo()
        dibujar()
        
    

def animcajaxyz(t1,t2,t3,t4):
    n=0
    while n<t1:
        ax.cla()
        setaxis()
        r=rotaz(n)
        dibujarcaja(r=r)
        n=n+1
        sistemafijo()
        dibujar()
        
    Rc=r
    n=0
    while n<t2:
        ax.cla()
        setaxis()
        r=rotax(n)
        r=np.dot(r,Rc)
        dibujarcaja(r=r)
        n=n+1
        sistemafijo()
        dibujar()

    Rc=r
    n=0
    while n<t3:
        ax.cla()
        setaxis()
        r=rotay(n)
        r=np.dot(Rc,r)
        dibujarcaja(r=r)
        n=n+1
        sistemafijo()
        dibujar()

    Rc=r
    n=0
    while n<t4:
        ax.cla()
        setaxis()
        r=rotax(n)
        r=np.dot(r,Rc)
        dibujarcaja(r=r)
        n=n+1
        sistemafijo()
        dibujar()

#         Ryft4 Rzft2 Rxft1 I Rxmt3 Rzmt5
def animcajaxyz2(t1,t2,t3,t4,t5):
    n1=0
    n2=0
    n3=0
    n4=0
    n5=0
    while n1<t1:
        ax.cla()
        setaxis()
        r=mmatrix(rotay(n4),rotaz(n2),rotax(n1),rotax(n3),rotaz(n5))
        dibujarcaja(r=r)
        n1=n1+1
        sistemafijo()
        sistemamovil(r)
        dibujar()
        
    Rc=r
    n=0
    while n2<t2:
        ax.cla()
        setaxis()
        r=mmatrix(rotay(n4),rotaz(n2),rotax(n1),rotax(n3),rotaz(n5))
        dibujarcaja(r=r)
        n2=n2+1
        sistemafijo()
        sistemamovil(r)
        dibujar()

    Rc=r
    n=0
    while n3<t3:
        ax.cla()
        setaxis()
        r=mmatrix(rotay(n4),rotaz(n2),rotax(n1),rotax(n3),rotaz(n5))
        dibujarcaja(r=r)
        n3=n3+1
        sistemafijo()
        sistemamovil(r)
        dibujar()

    Rc=r
    n=0
    while n4<t4:
        ax.cla()
        setaxis()
        r=mmatrix(rotay(n4),rotaz(n2),rotax(n1),rotax(n3),rotaz(n5))
        dibujarcaja(r=r)
        n4=n4+1
        sistemafijo()
        sistemamovil(r)
        dibujar()

    Rc=r
    n=0
    while n5<t5:
        ax.cla()
        setaxis()
        r=mmatrix(rotay(n4),rotaz(n2),rotax(n1),rotax(n3),rotaz(n5))
        dibujarcaja(r=r)
        n5=n5+1
        sistemafijo()
        sistemamovil(r)
        dibujar()

def animcajaxyzt(Dx,t1,t2):
    n=0
    while n<Dx+0.01:
        ax.cla()
        setaxis(4)
        r=trasx(n)
        print(r)
        dibujarcaja(r=r)
        n=n+0.2
        sistemafijo()
        sistemamovil(r)
        dibujar()
        
    Rc=r
    n=0
    while n<t1+0.01:
        ax.cla()
        setaxis(4)
        r=rotaz(n)
        r=np.dot(Rc,r)
        dibujarcaja(r=r)
        n=n+5
        sistemafijo()
        sistemamovil(r)
        dibujar()

    Rc=r
    n=0
    while n<t2+0.01:
        ax.cla()
        setaxis(4)
        r=rotaxf(n,Rc)
        dibujarcaja(r=r)
        n=n+5
        sistemafijo()
        sistemamovil(r)
        dibujar()


def animcajaxyzt2(Dx,Dy,t1,t2):
    n=0
    while n<Dx+0.01:
        ax.cla()
        setaxis(4)
        r=trasx(n)
        a=minv(r)
        a1=np.linalg.inv(r)
        print('incio')
        print('r')
        print(np.round(r,3))
        print('a')
        print(np.round(a,3))
        print('a1')
        print(np.round(a1,3))
        print('fin')
        dibujarcaja(r=r)
        n=n+0.2
        sistemafijo()
        sistemamovil(r)
        dibujar()
    Rc=r
    n=0
    while n<Dy+0.01:
        ax.cla()
        setaxis(4)
        r=trasy(n)
        r=np.dot(Rc,r)
        a=minv(r)
        a1=np.linalg.inv(r)
        print('incio')
        print('r')
        print(np.round(r,3))
        print('a')
        print(np.round(a,3))
        print('a1')
        print(np.round(a1,3))
        print('fin')
        dibujarcaja(r=r)
        n=n+0.2
        sistemafijo()
        sistemamovil(r)
        dibujar()
        
    Rc=r
    n=0
    while n<t1+0.01:
        ax.cla()
        setaxis(4)
        r=rotaz(n)
        r=np.dot(Rc,r)
        a=minv(r)
        a1=np.linalg.inv(r)
        print('incio')
        print('r')
        print(np.round(r,3))
        print('a')
        print(np.round(a,3))
        print('a1')
        print(np.round(a1,3))
        print('fin')
        dibujarcaja(r=r)
        n=n+5
        sistemafijo()
        sistemamovil(r)
        dibujar()

    Rc=r
    n=0
    while n<t2+0.01:
        ax.cla()
        setaxis(4)
        r=rotaxf(n,Rc)
        a=minv(r)
        a1=np.linalg.inv(r)
        print('incio')
        print('r')
        print(np.round(r,3))
        print('a')
        print(np.round(a,3))
        print('a1')
        print(np.round(a1,3))
        print('fin')
        dibujarcaja(r=r)
        n=n+5
        sistemafijo()
        sistemamovil(r)
        dibujar()

def animejeresaotro():
    n=0
    while n<3+0.01:
        ax.cla()
        setaxis(10)
        tab=trasx(n)
        n=n+0.2
        sistemafijo()
        sistemamovil(tab)
        dibujar()
        
    Rtab=tab
    n=0
    while n<5+0.01:
        ax.cla()
        setaxis(10)
        tab=trasy(n)
        tab=np.dot(Rtab,tab)
        n=n+0.2
        sistemafijo()
        sistemamovil(tab)
        dibujar()

    Rtab=tab
    n=0
    while n<45+0.01:
        ax.cla()
        setaxis(10)
        tab=rotax(n)
        tab=np.dot(Rtab,tab)
        n=n+5
        sistemafijo()
        sistemamovil(tab)
        dibujar()
        
    n=0
    while n>-5-0.01:
        ax.cla()
        setaxis(10)
        tac=trasx(n)
        n=n-0.2
        sistemafijo()
        sistemamovil(tac)
        sistemamovil(tab)
        dibujar()
        
    Rtac=tac
    n=0
    while n>-4-0.01:
        ax.cla()
        setaxis(10)
        tac=trasy(n)
        tac=np.dot(Rtac,tac)
        n=n-0.2
        sistemafijo()
        sistemamovil(tac)
        sistemamovil(tab)
        dibujar()

    tba=minv(tab)
    tbc=np.dot(tba,tac)

    n=0
    while n>-6-0.01:
        ax.cla()
        setaxis(10)
        #ntbc=rotazf(n,tbc)
        ntbc=np.dot(trasy(n),tbc)
        tac=np.dot(tab,ntbc)
        n=n-0.2
        sistemafijo()
        sistemamovil(tac)
        sistemamovil(tab)
        dibujar()

def animppp(d1,d2,d3):
    n1=0
    n2=0
    n3=0
    while n1<d1+0.01:
        ax.cla()
        setaxis(10)
        ppp(n1,n2,n3)
        n1=n1+0.2
        dibujar()

    while n2<d2+0.01:
        ax.cla()
        setaxis(10)
        ppp(n1,n2,n3)
        n2=n2+0.2
        dibujar()

    while n3<d3+0.01:
        ax.cla()
        setaxis(10)
        ppp(n1,n2,n3)
        n3=n3+0.2
        dibujar()

def animrpp(t1,d2,d3):
    n1=0
    n2=2
    n3=1
    while n1<t1+0.01:
        ax.cla()
        setaxis(5)
        rpp(n1,n2,n3)
        n1=n1+5
        dibujar()

    while n2<d2+0.01:
        ax.cla()
        setaxis(5)
        rpp(n1,n2,n3)
        n2=n2+0.2
        dibujar()

    while n3<d3+0.01:
        ax.cla()
        setaxis(5)
        rpp(n1,n2,n3)
        n3=n3+0.2
        dibujar()


def animrrp(t1,t2,d3):
    n1=0
    n2=0
    n3=1
    while n1<t1+0.01:
        ax.cla()
        setaxis(5)
        rrp(n1,n2,n3)
        n1=n1+5
        dibujar()

    while n2<t2+0.01:
        ax.cla()
        setaxis(5)
        rrp(n1,n2,n3)
        n2=n2+5
        dibujar()

    while n3<d3+0.01:
        ax.cla()
        setaxis(5)
        rrp(n1,n2,n3)
        n3=n3+0.2
        dibujar()

def animrrr(t1,t2,t3):
    n1=0
    n2=0
    n3=0
    while n1<t1+0.01:
        ax.cla()
        setaxis(5)
        rrr(n1,n2,n3)
        n1=n1+5
        dibujar()

    while n2<t2+0.01:
        ax.cla()
        setaxis(5)
        rrr(n1,n2,n3)
        n2=n2+5
        dibujar()

    while n3<t3+0.01:
        ax.cla()
        setaxis(5)
        rrr(n1,n2,n3)
        n3=n3+5
        dibujar()

def animscara(t1,t2,d3,t4):
    n1=0
    n2=0
    n3=1
    n4=0
    while n1<t1+0.01:
        ax.cla()
        setaxis(5)
        scara(n1,n2,n3,n4)
        n1=n1+5
        dibujar()

    while n2<t2+0.01:
        ax.cla()
        setaxis(5)
        scara(n1,n2,n3,n4)
        n2=n2+5
        dibujar()

    while n3<d3+0.01:
        ax.cla()
        setaxis(5)
        scara(n1,n2,n3,n4)
        n3=n3+0.2
        dibujar()

    while n4<t4+0.01:
        ax.cla()
        setaxis(5)
        scara(n1,n2,n3,n4)
        n4=n4+5
        dibujar()


def animcobras800(t1,t2,d3,t4):
    n1=0
    n2=0
    n3=1
    n4=0
    while n1<t1+0.01:
        ax.cla()
        setaxis(1000)
        cobras800(n1,n2,n3,n4)
        n1=n1+5
        dibujar()

    while n2<t2+0.01:
        ax.cla()
        setaxis(1000)
        cobras800(n1,n2,n3,n4)
        n2=n2+5
        dibujar()

    while n3<d3+0.01:
        ax.cla()
        setaxis(1000)
        cobras800(n1,n2,n3,n4)
        n3=n3+5
        dibujar()

    while n4<t4+0.01:
        ax.cla()
        setaxis(1000)
        cobras800(n1,n2,n3,n4)
        n4=n4+5
        dibujar()


def animur5(t1,t2,t3,t4,t5,t6):
    n1=0
    n2=0
    n3=0
    n4=0
    n5=0
    n6=0
    while n1<t1+0.01:
        ax.cla()
        setaxis(1000)
        ur5(n1,n2,n3,n4,n5,n6)
        n1=n1+5
        dibujar()

    while n2<t2+0.01:
        ax.cla()
        setaxis(1000)
        ur5(n1,n2,n3,n4,n5,n6)
        n2=n2+5
        dibujar()

    while n3<t3+0.01:
        ax.cla()
        setaxis(1000)
        ur5(n1,n2,n3,n4,n5,n6)
        n3=n3+5
        dibujar()

    while n4<t4+0.01:
        ax.cla()
        setaxis(1000)
        ur5(n1,n2,n3,n4,n5,n6)
        n4=n4+5
        dibujar()

    while n5<t5+0.01:
        ax.cla()
        setaxis(1000)
        ur5(n1,n2,n3,n4,n5,n6)
        n5=n5+5
        dibujar()

    while n6<t6+0.01:
        ax.cla()
        setaxis(1000)
        ur5(n1,n2,n3,n4,n5,n6)
        n6=n6+5
        dibujar()



def animur5(tb,t1a,t2a,t3a,t4a,t5a,t6a,t7a,t1b,t2b,t3b,t4b,t5b,t6b,t7b):
    nb=0
    n1a=0
    n2a=0
    n3a=0
    n4a=0
    n5a=0
    n6a=0
    n7a=0
    n1b=0
    n2b=0
    n3b=0
    n4b=0
    n5b=0
    n6b=0
    n7b=0
    while n1a<t1a+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n1a=n1a+5
        dibujar()

    while n2a<t2a+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n2a=n2a+5
        dibujar()

    while n3a<t3a+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n3a=n3a+5
        dibujar()

    while n4a<t4a+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n4a=n4a+5
        dibujar()

    while n5a<t5a+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n5a=n5a+5
        dibujar()

    while n6a<t6a+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n6a=n6a+5
        dibujar()

    while n7a<t7a+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n7a=n7a+5
        dibujar()


    while n1b<t1b+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n1b=n1b+5
        dibujar()

    while n2b<t2b+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n2b=n2b+5
        dibujar()

    while n3b<t3b+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n3b=n3b+5
        dibujar()

    while n4b<t4b+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n4b=n4b+5
        dibujar()

    while n5b<t5b+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n5b=n5b+5
        dibujar()

    while n6b<t6b+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n6b=n6b+5
        dibujar()

    while n7b<t7b+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        n7b=n7b+5
        dibujar()

    while nb<tb+0.01:
        ax.cla()
        setaxis(15)
        motoman(nb,n1a,n2a,n3a,n4a,n5a,n6a,n7a,n1b,n2b,n3b,n4b,n5b,n6b,n7b)
        nb=nb+5
        dibujar()


def animIRB1600(t1,t2,t3,t4,t5,t6):
    n1=0
    n2=0
    n3=0
    n4=0
    n5=0
    n6=0
    while n1<t1+0.01:
        ax.cla()
        setaxis(1500)
        IRB1600(n1,n2,n3,n4,n5,n6)
        n1=n1+5
        dibujar()

    while n2<t2+0.01:
        ax.cla()
        setaxis(1500)
        IRB1600(n1,n2,n3,n4,n5,n6)
        n2=n2+5
        dibujar()

    while n3<t3+0.01:
        ax.cla()
        setaxis(1500)
        IRB1600(n1,n2,n3,n4,n5,n6)
        n3=n3+5
        dibujar()

    while n4<t4+0.01:
        ax.cla()
        setaxis(1500)
        IRB1600(n1,n2,n3,n4,n5,n6)
        n4=n4+5
        dibujar()

    while n5<t5+0.01:
        ax.cla()
        setaxis(1500)
        IRB1600(n1,n2,n3,n4,n5,n6)
        n5=n5+5
        dibujar()

    while n6<t6+0.01:
        ax.cla()
        setaxis(1500)
        IRB1600(n1,n2,n3,n4,n5,n6)
        n6=n6+5
        dibujar()

def animIRB1600newton():
    n1=0
    n2=0
    n3=0
    n4=0
    vd=IRB1600v(0,0,0,0,0,0)
    sem=[0,0,0,0,0,0]
    while n1<200+0.01:
        vdn=vd@trasx(n1)
        ax.cla()
        setaxis(1500)
        tetas=IRB1600newton(vdn,sem)
        IRB1600(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        n1=n1+20
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn

    while n2>-45-0.01:
        vdn=vd@rotax(n2)
        ax.cla()
        setaxis(1500)
        tetas=IRB1600newton(vdn,sem)
        IRB1600(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        n2=n2-5
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn

    while n3<300+0.01:
        vdn=vd@trasy(n3)
        ax.cla()
        setaxis(1500)
        tetas=IRB1600newton(vdn,sem)
        IRB1600(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        n3=n3+20
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn

    while n4<200+0.01:
        vdn=vd@trasz(n4)
        ax.cla()
        setaxis(1500)
        tetas=IRB1600newton(vdn,sem)
        IRB1600(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        n4=n4+20
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn


def animIRB1600newtoncirculo(r):
    n1=0
    vd=IRB1600v(0,0,0,0,0,0)
    sem=[0,0,0,0,0,0]
    while n1<2*np.pi+0.01:
        x=np.cos(n1)
        y=np.sin(n1)
        x=x-1
        vdn=vd@trasx(-x*r)
        vdn=vdn@trasy(-y*r)
        ax.cla()
        setaxis(1500)
        tetas=IRB1600newton(vdn,sem)
        IRB1600(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        robot.MoveJ(tetas)
        n1=n1+(np.pi/30)
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn

def animIRB1600newtoncirculoh1(r):
    n1=0
    mh=herramienta1v(-90)
    vd=IRB1600v(0,0,0,0,0,0,mh)
    sem=[0,0,0,0,0,0]
    while n1<2*np.pi+0.01:
        x=np.cos(n1)
        y=np.sin(n1)
        x=x-1
        vdn=vd@rotax(45)
        vdn=vdn@trasx(-x*r)
        vdn=vdn@trasy(-y*r)
        ax.cla()
        setaxis(1500)
        tetas=IRB1600newton(vdn,sem,mh)
        IRB1600(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        mr=IRB1600v(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        herramienta1(mr,-90)
        robot.MoveJ(tetas)
        n1=n1+(np.pi/30)
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn

def animur5newtoncirculo(r):
    n1=0
    vd=ur5v(0,0,0,0,0,0)
    sem=[0,0,0,0,0,0]
    while n1<2*np.pi+0.01:
        x=np.cos(n1)
        y=np.sin(n1)
        x=x-1
        #vdn=vd@rotax(-90)
        vdn=vd@trasy(-x*r)
        vdn=vdn@trasx(y*r)
        ax.cla()
        setaxis(1500)
        tetas=ur5newton(vdn,sem)
        ur5(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        valores1=tetas.copy()
        valores1[1]=valores1[1]-90
        valores1[2]=valores1[2]-90
        valores1[4]=valores1[4]+90
        valores1[4]=valores1[4]%360
        robot.MoveJ(valores1)
        n1=n1+(np.pi/30)
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn

def animur5newtoncirculoh1(r):
    n1=0
    mh=herramienta1v()
    vd=ur5v(0,0,0,0,0,0,mh)
    sem=[0,0,0,0,0,0]
    while n1<2*np.pi+0.01:
        x=np.cos(n1)
        y=np.sin(n1)
        x=x-1
        vdn=vd@rotax(51.21)
        vdn=vdn@trasy(-x*r)
        vdn=vdn@trasx(y*r)
        ax.cla()
        setaxis(1500)
        tetas=ur5newton(vdn,sem,mh)
        ur5(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        mr=ur5v(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        herramienta1(mr)
        valores1=tetas.copy()
        valores1[1]=valores1[1]-90
        valores1[2]=valores1[2]-90
        valores1[4]=valores1[4]+90
        valores1[4]=valores1[4]%360
        robot.MoveJ(valores1)
        n1=n1+(np.pi/30)
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn

def animur5newtoncirculome(r):
    n1=0
    mb=trasx(1200)@rotaz(180)@trasy(220)
    vd=ur5v(0,0,0,0,0,0)
    sem=[0,0,0,0,0,0]
    while n1<2*np.pi+0.01:
        x=np.cos(n1)
        y=np.sin(n1)
        x=x-1
        #vdn=vd@rotax(-90)
        vdn=vd@trasy(-x*r)
        vdn=vdn@trasx(y*r)
        vdn2=vdn@trasz(200)@rotax(180)@rotaz(180)
        ax.cla()
        setaxis(1500)
        tetas=ur5newton(vdn,sem)
        tetas2=ur5newton(vdn2,sem,mb=mb)
        ur5(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        ur5(tetas2[0],tetas2[1],tetas2[2],tetas2[3],tetas2[4],tetas2[5],mb)
        valores1=tetas.copy()
        valores1e=tetas2.copy()
        valores1[1]=valores1[1]-90
        valores1[2]=valores1[2]-90
        valores1[4]=valores1[4]+90
        valores1[4]=valores1[4]%360
        valores1e[1]=valores1e[1]-90
        valores1e[2]=valores1e[2]-90
        valores1e[4]=valores1e[4]+90
        valores1e[4]=valores1e[4]%360
        #robot.MoveJ(valores1)
        #robote.MoveJ(valores1e)
        n1=n1+(np.pi/30)
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn

def animur5newtoncirculoh1me(r):
    n1=0
    mb=trasx(1250)@rotaz(180)@trasy(220)
    mh=herramienta1v()
    vd=ur5v(0,0,0,0,0,0,mh)
    sem=[0,0,0,0,0,0]
    while n1<2*np.pi+0.01:
        x=np.cos(n1)
        y=np.sin(n1)
        x=x-1
        vdn=vd@rotax(0)
        vdn=vdn@trasy(-x*r)
        vdn=vdn@trasx(y*r)
        vdn2=vdn@trasz(200)@rotax(180)@rotaz(180)
        ax.cla()
        setaxis(1500)
        tetas=ur5newton(vdn,sem,mh)
        tetas2=ur5newton(vdn2,sem,mb=mb)
        ur5(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        ur5(tetas2[0],tetas2[1],tetas2[2],tetas2[3],tetas2[4],tetas2[5],mb)
        mr=ur5v(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5])
        herramienta1(mr)
        valores1=tetas.copy()
        valores1[1]=valores1[1]-90
        valores1[2]=valores1[2]-90
        valores1[4]=valores1[4]+90
        valores1[4]=valores1[4]%360
        #robot.MoveJ(valores1)
        n1=n1+(np.pi/30)
        dibujar()
        sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5]]
    vd=vdn


arduino=serial.Serial("COM5",9600,timeout=1)
time.sleep(1)
leyendo=True
mtr1,mtr2=accmotomanv(0,90,-90,-90,-130,0,-60,90,90,90,-90,-130,0,-60,90)
tetas=[0,90,-90,-90,-130,0,-60,90,90,90,-90,-130,0,-60,90]
movesel=1
puntosmove=[0,0,0,0,0,0,0,0,0]
selbrazo=1;
selbrazopuntos=[0,0,0,0,0,0,0,0,0]
mtr12=mtr1.copy()


modo=1
puntos=np.zeros((10,4,4))
tetaspuntos=np.zeros((10,15))
puntonum=-1


x1omin=0
x1omax=1023
x1nmin=-10
x1nmax=10
x1or=(x1omax-x1omin)
x1nr=(x1nmax-x1nmin)

x2omin=0
x2omax=1023
x2nmin=-10
x2nmax=10
x2or=(x2omax-x2omin)
x2nr=(x2nmax-x2nmin)

x3omin=0
x3omax=1023
x3nmin=-10
x3nmax=10
x3or=(x3omax-x3omin)
x3nr=(x3nmax-x3nmin)

y1omin=0
y1omax=1023
y1nmin=-10
y1nmax=10
y1or=(y1omax-y1omin)
y1nr=(y1nmax-y1nmin)

y2omin=0
y2omax=1023
y2nmin=-10
y2nmax=10
y2or=(y2omax-y2omin)
y2nr=(y2nmax-y2nmin)

y3omin=0
y3omax=1023
y3nmin=-10
y3nmax=10
y3or=(y3omax-y3omin)
y3nr=(y3nmax-y3nmin)

def leerarduino():
    global selbrazo
    global leyendo
    global modo
    global mtr1
    global mtr2
    global mtr12
    sem=[0,90,-90,-90,-130,0,-60,90,90,90,-90,-130,0,-60,90]
    global tetas
    time.sleep(1)
    while leyendo:
        mensaje="0"
        arduino.write(mensaje.encode('ascii'))
        vals=arduino.readline().decode('ascii')
        print(vals)
        pos1=vals.index(",")
        x1=vals[0:pos1]
        pos2=vals.index(",",(pos1+1))
        y1=vals[pos1+1:pos2]
        pos3=vals.index(",",(pos2+1))
        x2=vals[pos2+1:pos3]
        pos4=vals.index(",",(pos3+1))
        y2=vals[pos3+1:pos4]
        pos5=vals.index(",",(pos4+1))
        x3=vals[pos4+1:pos5]
        y3=vals[pos5+1:]
        x1=int(x1)
        x2=int(x2)
        x3=int(x3)
        y1=int(y1)
        y2=int(y2)
        y3=int(y3)
        x1n=(((x1-x1omin)*x1nr)/x1or)+x1nmin
        x2n=(((x2-x2omin)*x2nr)/x2or)+x2nmin
        x3n=(((x3-x3omin)*x3nr)/x3or)+x3nmin
        y1n=(((y1-y1omin)*y1nr)/y1or)+y1nmin
        y2n=(((y2-y2omin)*y2nr)/y2or)+y2nmin
        y3n=(((y3-y3omin)*y3nr)/y3or)+y3nmin
        if (abs(x1n)<1.8):
             x1n=0
        if (abs(x2n)<1.8):
             x2n=0
        if (abs(x3n)<1.8):
             x3n=0
        if (abs(y1n)<1.8):
             y1n=0
        if (abs(y2n)<1.8):
             y2n=0
        if (abs(y3n)<1.8):
             y3n=0
        print(x1n)
        print(x2n)
        print(x3n)
        print(y1n)
        print(y2n)
        print(y3n)
        if modo==1:
            if selbrazo==1:
                tetas[1]=tetas[1]+x1n
                tetas[2]=tetas[2]+y1n
                tetas[3]=tetas[3]+x2n
                tetas[4]=tetas[4]+y2n
                tetas[5]=tetas[5]+x3n
                tetas[6]=tetas[6]+y3n
                tetas[7]=tetas[7]
                ax.cla()
                setaxis(1000)
                accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                mtr1,mtr2=accmotomanv(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                dibujar()
            elif selbrazo==2:
                tetas[8]=tetas[8]+x1n
                tetas[9]=tetas[9]+y1n
                tetas[10]=tetas[10]+x2n
                tetas[11]=tetas[11]+y2n
                tetas[12]=tetas[12]+x3n
                tetas[13]=tetas[13]+y3n
                tetas[14]=tetas[14]
                ax.cla()
                setaxis(1000)
                accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                mtr1,mtr2=accmotomanv(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                dibujar()
            elif selbrazo==3:
                tetas[1]=tetas[1]+x1n
                tetas[2]=tetas[2]+y1n
                tetas[3]=tetas[3]+x2n
                tetas[4]=tetas[4]+y2n
                tetas[5]=tetas[5]+x3n
                tetas[6]=tetas[6]+y3n
                mtr1,mtr2=accmotomanv(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                mtr2=mtr1@mtr12
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                tetas=motomannewton2(mtr1,mtr2,sem)
                ax.cla()
                setaxis(1000)
                accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                mtr1,mtr2=accmotomanv(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                dibujar()
                pass
        elif modo==2:
            if selbrazo==1:
                ax.cla()
                setaxis(1000)
                mtr1=mtr1@trasx(x1n*5)
                mtr1=mtr1@rotax(y1n)
                mtr1=mtr1@trasy(x2n*5)
                mtr1=mtr1@rotay(y2n)
                mtr1=mtr1@trasz(x3n*5)
                mtr1=mtr1@rotaz(y3n)
                tetas=motomannewton2(mtr1,mtr2,sem)
                accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                dibujar()
            elif selbrazo==2:
                ax.cla()
                setaxis(1000)
                mtr2=mtr2@trasx(x1n*5)
                mtr2=mtr2@rotax(y1n)
                mtr2=mtr2@trasy(x2n*5)
                mtr2=mtr2@rotay(y2n)
                mtr2=mtr2@trasz(x3n*5)
                mtr2=mtr2@rotaz(y3n)
                tetas=motomannewton2(mtr1,mtr2,sem)
                accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                dibujar()
            elif selbrazo==3:
                ax.cla()
                setaxis(1000)
                mtr1=mtr1@trasx(x1n*5)
                mtr1=mtr1@rotax(y1n)
                mtr1=mtr1@trasy(x2n*5)
                mtr1=mtr1@rotay(y2n)
                mtr1=mtr1@trasz(x3n*5)
                mtr1=mtr1@rotaz(y3n)
                mtr2=mtr1@mtr12
                tetas=motomannewton2(mtr1,mtr2,sem)
                accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                dibujar()
        elif modo==3:
            if selbrazo==1:
                ax.cla()
                setaxis(1000)
                mtr1=trasx(x1n*5)@mtr1
                mtr1=rotaxf(y1n,mtr1)
                mtr1=trasy(x2n*5)@mtr1
                mtr1=rotayf(y2n,mtr1)
                mtr1=trasz(x3n*5)@mtr1
                mtr1=rotazf(y3n,mtr1)
                tetas=motomannewton2(mtr1,mtr2,sem)
                accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                dibujar()
            elif selbrazo==2:
                ax.cla()
                setaxis(1000)
                mtr2=trasx(x1n*5)@mtr2
                mtr2=rotaxf(y1n,mtr2)
                mtr2=trasy(x2n*5)@mtr2
                mtr2=rotayf(y2n,mtr2)
                mtr2=trasz(x3n*5)@mtr2
                mtr2=rotazf(y3n,mtr2)
                tetas=motomannewton2(mtr1,mtr2,sem)
                accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                dibujar()
            elif selbrazo==3:
                ax.cla()
                setaxis(1000)
                mtr1=trasx(x1n*5)@mtr1
                mtr1=rotaxf(y1n,mtr1)
                mtr1=trasy(x2n*5)@mtr1
                mtr1=rotayf(y2n,mtr1)
                mtr1=trasz(x3n*5)@mtr1
                mtr1=rotazf(y3n,mtr1)
                mtr2=mtr1@mtr12
                tetas=motomannewton2(mtr1,mtr2,sem)
                accmotoman(tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14])
                sem=[tetas[0],tetas[1],tetas[2],tetas[3],tetas[4],tetas[5],tetas[6],tetas[7],tetas[8],tetas[9],tetas[10],tetas[11],tetas[12],tetas[13],tetas[14]]
                dibujar()
            
    print("finalizando hilo")



axbtnvalm=plt.axes([0.42,0.06,0.15,0.06])
axbtnmove=plt.axes([0.6,0.06,0.15,0.06])
axbtnmode=plt.axes([0.42,0.13,0.15,0.06])
axbtn1=plt.axes([0.85,0.01,0.12,0.08])
axtxtpuntos = plt.axes([0.02, 0.2, 0.34, 0.75])
axbtnsecuencia=plt.axes([0.04,0.13,0.22,0.06])
axbtnreset=plt.axes([0.04,0.06,0.11,0.06])
axrbtnselbrazo=plt.axes([0.27,0.02,0.14,0.175])
        
btnmodo=Button(axbtnmode,'modo: Joint')
btnvalm=Button(axbtnvalm,'Grabar')
btnmove=Button(axbtnmove,'Move J')
button1=Button(axbtn1,'cerrar')
btnsecuencia=Button(axbtnsecuencia,'Realizar secuencia')
btnreset=Button(axbtnreset,'Reset')
txtpuntos = TextBox(axtxtpuntos, '',initial='Empieza a grabar puntos')
radioselbrazo = RadioButtons(axrbtnselbrazo, ('Brazo 1', 'Brazo 2', '2 Brazos'))
ax.cla()
setaxis(1000)
accmotoman(0,90,-90,-90,-130,0,-60,90,90,90,-90,-130,0,-60,90)
dibujar()

def grabar(event):
    global puntos
    global puntonum
    global tetas
    global tetaspuntos
    global movesel
    global puntosmove
    if puntonum<9:
        puntonum+=1
        tetaspuntos[puntonum,:]=tetas
        print(puntos)
        print(tetaspuntos)
        n=0
        texto=""
        if puntonum>0:
            puntosmove[puntonum-1]=movesel
            while n<puntonum:
                if puntosmove[n]==1:
                    texto=texto+"p"+str(n)+" p"+str(n+1)+" MoveJ\n"
                elif puntosmove[n]==2:
                    texto=texto+"p"+str(n)+" p"+str(n+1)+" MoveL\n"
                else:
                    texto=texto+"p"+str(n)+" p"+str(n+1)+" MoveC\n"
                n=n+1
            txtpuntos.set_val(texto)
    else:
        print("No hay mas espacios para grabar")

def cerrar(event):
    global leyendo
    leyendo=False
    plt.close()
    hilo1.join()
    arduino.close()
    sys.exit()

def mode(event):
    global modo
    if modo==1:
        modo=2
        btnmodo.label.set_text("modo: Tool")
    elif modo==2:
        modo=3
        btnmodo.label.set_text("modo: World")
    elif modo==3:
        modo=1
        btnmodo.label.set_text("modo: Joint")

def move(event):
    global movesel
    if movesel==1:
        movesel=2
        btnmove.label.set_text("Move L")
    elif movesel==2:
        movesel=3
        btnmove.label.set_text("Move C")
    elif movesel==3:
        movesel=1
        btnmove.label.set_text("Move J")
        
def realizarsecuencia(event):
    global puntos
    global puntonum
    global tetas
    global tetaspuntos
    global puntosmove
    p1=np.zeros((4,4))
    p2=np.zeros((4,4))
    tetas1=[0,0,0,0,0,0]
    tetas2=[0,0,0,0,0,0]
    n=0
    if puntonum>0:
        tetaanterior=tetas1=tetaspuntos[n,:]
        while n<puntonum:
            if puntosmove[n]==1:
                tetas1=tetaspuntos[n,:]
                tetas2=tetaspuntos[n+1,:]
                tetaanterior=motomanmovej(tetaanterior,tetas2)
            elif puntosmove[n]==2:
                tetas1=tetaspuntos[n,:]
                tetas2=tetaspuntos[n+1,:]
                tetaanterior=motomanmovel(tetaanterior,tetas2)
            else:
                tetas1=tetaspuntos[n,:]
                p1=puntos[n,:,:]
                p2=puntos[n+1,:,:]
                ur5movec(p1,p2,tetas1)
            n=n+1
        tetas=tetaanterior
    else:
        print("No hay suficientes puntos grabados")
            
def resetboton(event):
    global puntonum
    puntonum=-1
    txtpuntos.set_val("")

def brazoseleccionado(label):
    global selbrazo
    global mtr1
    global mtr2
    global mtr12
    if label=='Brazo 1':
        selbrazo=1
    elif label=='Brazo 2':
        selbrazo=2
    elif label=='2 Brazos':
        selbrazo=3
        mtr12=minv(mtr1)@mtr2
    
    
btnmodo.on_clicked(mode)
button1.on_clicked(cerrar)
btnvalm.on_clicked(grabar)
btnmove.on_clicked(move)
btnreset.on_clicked(resetboton)
btnsecuencia.on_clicked(realizarsecuencia)
radioselbrazo.on_clicked(brazoseleccionado)
hilo1=threading.Thread(target=leerarduino(),daemon=True)
hilo1.start()


#Motoman CSDA10F








