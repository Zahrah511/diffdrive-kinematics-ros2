import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'diffdrive_sim', 'diffdrive_sim'))
from diffdrive_kin import *  # noqa: E402,F403
"""Checks the core equations against the hand-computed numbers of the variations."""
L=0.5
ok=lambda a,b,tol=1e-3: abs(a-b)<tol
# V1
assert icc_radius(0.5,0.5,L)==math.inf
assert icc_radius(0.3,-0.3,L)==0
p=simulate((0,0,0),0.3,-0.3,L,0.01,1.0); assert ok(p[-1][0],0) and ok(p[-1][1],0)
# V2
vr,vl=inverse(0.7,2.0,L); print("V2",vr,vl); assert ok(vr,0.7875) and ok(vl,0.6125)
pts=simulate((0,0,0),vr,vl,L,0.01,2*math.pi*2/0.7)
c=icc_point((0,0,0),vr,vl,L); print("V2 radius range",min(math.dist(q[:2],c) for q in pts),max(math.dist(q[:2],c) for q in pts))
# V3
sq=run_segments((0,0,0),plan_square(2,0.5,1.2,L),L,0.01); print("V3 square end",sq[-1])
f8=run_segments((0,0,0),plan_figure8(1.0,0.5,L),L,0.01); print("V3 fig8 end",f8[-1])
# V4
R,V=1.5,0.6; vr,vl=inverse(V,R,L); T=2*math.pi*R/V
for dt in [0.02,0.05,0.2,0.5,1.0]:
    e=simulate((0,0,0),vr,vl,L,dt,T,"euler"); x=simulate((0,0,0),vr,vl,L,dt,T,"exact")
    same_t=max(math.dist(a[:2],b[:2]) for a,b in zip(e,x))
    c=(0,R); circ=max(abs(math.dist(a[:2],c)-R) for a in e)
    print(f"V4 dt={dt}: same-time {same_t*100:.1f} cm, from-circle {circ*100:.1f} cm")
# V5
a=plan_arc_to_goal(2,1,0.5,L); print("V5A",a); assert ok(a['R'],2.5) and ok(a['VR'],0.55) and ok(a['VL'],0.45)
end=run_segments((0,0,0),[(a['VR'],a['VL'],a['t'])],L,0.001)[-1]; print("V5A end",end, math.degrees(end[2]))
b=plan_spin_then_straight(2,1,0.5,1.2,L); print("V5B",b, sum(s[2] for s in b))
end=run_segments((0,0,0),b,L,0.001)[-1]; print("V5B end",end, math.degrees(end[2]))
print("w2r",world_to_robot((1.73,2.37),(0.5,0.5,math.radians(30))))
print("ALL OK")
