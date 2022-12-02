def forw(a,b,c,d,e,f):
    import math as m
    import numpy as np
    a1=a*m.pi/180
    a2=b*m.pi/180
    a3=(b+c)*m.pi/180
    a4=(b+c+d)*m.pi/180
    a5=e*m.pi/180
    a6=f*m.pi/180
    l1=61+43.5
    l2=82.85
    l3=82.85
    l4=73.85
    l5=54.57
    l6=0
    z=l1+l2*m.cos(a2)+l3*m.cos(a3)+(l4+l5)*m.cos(a4)+l6
    xy=l2*m.sin(a2)+l3*m.sin(a3)+(l4+l5)*m.sin(a4)
    y=xy*m.sin(a1)
    x=xy*m.cos(a1)
    o=a5+90/180
    r=a6
    o+r
    pos=np.array([[x],[y],[z]])
    return pos
def inv(pos):
    import math as m
    import numpy as np
    pos=np.array(pos)
    x=float(pos[0])
    y=float(pos[1])
    z=float(pos[2])
    l1=61+43.5
    l2=82.85
    l3=82.85
    l4=73.85
    l5=54.57
    xy=(x**2+y**2)**0.5
    xyz=(xy**2+z**2)**0.5
    a1=round(m.atan(y/x)*180/m.pi,0)*m.pi/180
    low=-91
    high=91
    A2=A3=A4=-100
    xyzT=xyT=zT=-100000
    for i in range(low,high,1):
        a2=(i)*m.pi/180
        for j in range(low,high,1):
            a3=(j+i)*m.pi/180
            for k in range(low,high,1):
                a4=(k+j+i)*m.pi/180
                zT=l1+l2*m.cos(a2)+l3*m.cos(a3)+(l4+l5)*m.cos(a4)
                xyT=l2*m.sin(a2)+l3*m.sin(a3)+(l4+l5)*m.sin(a4)
                xyzT=(zT**2+xyT**2)**0.5
                yT=xyT*m.sin(a1)
                xT=xyT*m.cos(a1)
                if xT == x and y == yT and z == zT:
                    A4=k
                    break
            if xT == x and y == yT and z == zT:
                A3=j
                break
        if xT == x and y == yT and z == zT:
            A2=i
            break
    xyz-xyzT
    B1=a1*180/m.pi
    B2=A2
    B3=A3
    B4=A4
    B5=0
    B6=0
    a=np.array([[B1],[B2],[B3],[B4],[B5],[B6]])
    return a
def finv(pos):
    import math as m
    import numpy as np
    pos=np.array(pos)
    x=float(pos[0])
    y=float(pos[1])
    z=float(pos[2])
    l1=61+43.5
    l2=82.85
    l3=82.85
    l4=73.85
    l5=54.57
    xy=(x**2+y**2)**0.5
    xyz=(xy**2+z**2)**0.5
    a1=round(m.atan(y/x)*180/m.pi,0)*m.pi/180
    low=-91
    high=91
    A2=A3=A4=-100
    xyzT=xyT=zT=-100000
    for i in range(low,high,1):
        a2=(i)*m.pi/180
        for j in range(low,high,1):
            a3=(j+i)*m.pi/180
            z03=l1+l2*m.cos(a2)+l3*m.cos(a3)
            xy03=l2*m.sin(a2)+l3*m.sin(a3)
            dz=z-z03
            dxy=xy-xy03
            k=round(m.atan(dz/dxy)*180/m.pi,0)
            a4=(90-k)*m.pi/180
            zT=l1+l2*m.cos(a2)+l3*m.cos(a3)+(l4+l5)*m.cos(a4)
            xyT=l2*m.sin(a2)+l3*m.sin(a3)+(l4+l5)*m.sin(a4)
            xyzT=(zT**2+xyT**2)**0.5
            yT=xyT*m.sin(a1)
            xT=xyT*m.cos(a1)
            xyz-xyzT
            if xT == x and y == yT and z == zT:
                A3=j
                A4=90-k-i-j
                break
        if xT == x and y == yT and z == zT:
            A2=i
            break
    B1=a1*180/m.pi
    B2=A2
    B3=A3
    B4=A4
    B5=0
    B6=0
    a=np.array([[B1],[B2],[B3],[B4],[B5],[B6]])
    return a