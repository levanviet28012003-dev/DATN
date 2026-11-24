#!/usr/bin/env python3
"""Sinh mesh STL (chỉ hình học, đơn vị m) cho URDF; màu khai báo trong URDF."""
import math
import struct
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / 'meshes'


class Mesh:
    def __init__(self):
        self.parts = {}
    def tri(self, color, a,b,c):
        self.parts.setdefault(color, []).extend((a,b,c))
    def quad(self, color,a,b,c,d):
        self.tri(color,a,b,c); self.tri(color,a,c,d)
    def box(self, color, center, size, angle=0):
        cx,cy,cz=center; sx,sy,sz=size
        def p(x,y,z):
            return (cx+x*math.cos(angle)-z*math.sin(angle), cy+y,
                    cz+x*math.sin(angle)+z*math.cos(angle))
        v=[p(x*sx/2,y*sy/2,z*sz/2) for x,y,z in
           [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
        for a,b,c,d in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
            self.quad(color,v[a],v[b],v[c],v[d])
    def ring(self,color,inner,outer,width,y=0,n=96):
        def p(r,a,h): return (r*math.cos(a),h,r*math.sin(a))
        for i in range(n):
            a=2*math.pi*i/n; b=2*math.pi*(i+1)/n
            for r,rev in [(outer,False),(inner,True)]:
                if r == 0: continue
                q=[p(r,a,y-width/2),p(r,b,y-width/2),p(r,b,y+width/2),p(r,a,y+width/2)]
                self.quad(color,*(q[::-1] if rev else q))
            for h,rev in [(y-width/2,False),(y+width/2,True)]:
                q=[p(inner,a,h),p(inner,b,h),p(outer,b,h),p(outer,a,h)]
                self.quad(color,*(q[::-1] if rev else q))
    def save_stl(self, name):
        """Geometry only, with explicit face normals; colour comes from URDF."""
        vertices = [v for part in self.parts.values() for v in part]
        data = bytearray(b'Hiwonder geometry; colour supplied by URDF'.ljust(80, b' '))
        data.extend(struct.pack('<I', len(vertices) // 3))
        for i in range(0, len(vertices), 3):
            a, b, c = vertices[i:i+3]
            u = [b[j]-a[j] for j in range(3)]
            v = [c[j]-a[j] for j in range(3)]
            n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
            length = math.sqrt(sum(x*x for x in n))
            n = [x/length for x in n] if length else [0., 0., 1.]
            data.extend(struct.pack('<12fH', *n, *a, *b, *c, 0))
        (OUT / (name + '.stl')).write_bytes(data)


def plate():
    m=Mesh()
    # Rasterised machining profile: actual openings rather than painted holes.
    def solid(x,y):
        half=.06025 if x<.025 else (.048 if x<.067 else .057)
        half-=max(0,abs(x)-.124)*.65
        if abs(y)>half: return False
        for sx in [-.109,-.079,-.049,-.019]:
            if abs(x-sx)<.003 and abs(y)<.042: return False
        if -.01<x<.035 and abs(y)<.014: return False
        if .078<x<.099 and abs(y)<.009: return False
        for sx in [-.12,-.09,-.06,-.03,0,.04,.07,.105,.12]:
            for sy in [-.049,-.033,.033,.049]:
                if (x-sx)**2+(y-sy)**2<.0018**2: return False
        return True
    step=.001
    active={}
    for i in range(271):
        x=-.135+i*step
        runs=[]; start=None
        for j in range(122):
            hit=i<270 and j<121 and solid(x+step/2,-.0605+(j+.5)*step)
            if hit and start is None: start=j
            if not hit and start is not None: runs.append((start,j)); start=None
        for run in list(active):
            if run not in runs:
                first=active.pop(run); a,b=run
                m.box('chassis',((first+i)*step/2-.135,(a+b)*step/2-.0605,0),((i-first)*step,(b-a)*step,.002))
        for run in runs: active.setdefault(run,i)
    m.save_stl('chassis_plate')


def wheel():
    m=Mesh()
    m.ring('rubber',.027,.0365,.029)
    # Narrow segmented chevron tread blocks across the tyre crown.
    for i in range(64):
        a=2*math.pi*i/64
        for j in range(5):
            t=a+(.018 if j%2 else -.018)
            m.box('rubber',(.037*math.cos(t),(j-2)*.0055,.037*math.sin(t)),(.003,.005,.001),t-math.pi/2)
    for side in [-1,1]:
        m.ring('red',.026,.0305,.0018,side*.015)
        for i in range(12):
            a=i*math.pi/6
            m.box('black',(.017*math.cos(a),side*.014,.017*math.sin(a)),(.023,.003,.004),a)
        m.ring('silver',0,.004,.002,side*.0165,32)
    m.ring('black',0,.008,.028,n=48)
    # Mỗi màu một file STL để URDF gán màu riêng cho từng visual.
    for color, vertices in m.parts.items():
        part = Mesh()
        part.parts[color] = vertices
        part.save_stl('wheel_' + color)

if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    plate(); wheel()
