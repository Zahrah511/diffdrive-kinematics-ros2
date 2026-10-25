"""Variation 4 figure: Euler vs exact pose update (offline, uses diffdrive_kin)."""
import math, sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'diffdrive_sim'))
from diffdrive_sim import diffdrive_kin as k
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({"font.family":"DejaVu Sans","font.size":15,"axes.edgecolor":"#8796A5","axes.labelcolor":"#2E3A47","xtick.color":"#4A5868","ytick.color":"#4A5868"})
L=0.5; PUR,ORA,INK,MUT="#5B4BC4","#D9622B","#18222E","#4A5868"
# left: circle R=1.5 V=0.6, max distance of Euler path from the true circle
R,V=1.5,0.6; vr,vl=k.inverse(V,R,L); T=2*math.pi*R/V
dts=[0.001,0.02,0.05,0.2,0.5,1.0]; err=[]
for dt in dts:
    e=k.simulate((0,0,0),vr,vl,L,dt,T,"euler"); err.append(max(abs(math.dist(p[:2],(0,R))-R) for p in e)*100)
print(list(zip(dts,[round(x,2) for x in err])))
# right: dt=0.2, V constant vs V varying with turning
dt=0.2; t=[];a=[];b=[]
pe1=px1=pe2=px2=(0,0,0)
for i in range(1500):
    tt=i*dt; w=0.8*math.sin(0.5*tt)
    V1=0.6; V2=0.6+0.4*math.sin(0.5*tt)
    for (V_,store) in ((V1,1),(V2,2)):
        vr_,vl_=V_+w*L/2,V_-w*L/2
        if store==1: pe1=k.step_euler(pe1,vr_,vl_,L,dt); px1=k.step_exact(px1,vr_,vl_,L,dt)
        else: pe2=k.step_euler(pe2,vr_,vl_,L,dt); px2=k.step_exact(px2,vr_,vl_,L,dt)
    t.append(tt+dt); a.append(math.dist(pe1[:2],px1[:2])*100); b.append(math.dist(pe2[:2],px2[:2])*100)
fig,(a1,a2)=plt.subplots(1,2,figsize=(16,6.4),facecolor="white")
a1.plot(dts,err,color=ORA,lw=2.5,marker="o",ms=9)
for d,e in zip(dts,err):
    if d>=0.2: a1.annotate(f"{e:.1f} cm",(d,e),textcoords="offset points",xytext=(10,-6),fontsize=13,color=INK)
a1.annotate("dt = 0.02 s: 0.6 cm\nour Gazebo runs, dt ≈ 0.001 s: 0.03 cm",(0.02,err[1]),textcoords="data",
            xytext=(0.36,2.0),fontsize=13,color=INK,arrowprops=dict(arrowstyle="-",color=MUT))
a1.set_xlabel("time step dt (s)"); a1.set_ylabel("max distance from true circle (cm)")
a1.set_title("Circle R = 1.5 m: Euler error grows in proportion to dt",fontsize=17,color=INK,loc="left")
a1.grid(alpha=.25)
a2.plot(t,a,color=PUR,lw=2.5); a2.plot(t,b,color=ORA,lw=2.5)
a2.text(t[-1],a[-1]+12,f"V constant: {a[-1]:.0f} cm after 300 s",ha="right",fontsize=13,color=INK)
a2.text(5,320,f"V changes while turning: {b[-1]:.0f} cm after 300 s",ha="left",fontsize=13,color=INK)
a2.set_xlabel("time (s), dt = 0.2 s"); a2.set_ylabel("Euler vs exact gap (cm)")
a2.set_title("Same turns, dt = 0.2 s: drift depends on V",fontsize=17,color=INK,loc="left"); a2.grid(alpha=.25)
for ax in (a1,a2):
    ax.spines["top"].set_visible(False); ax.spines["right"].set_visible(False)
fig.tight_layout(); fig.savefig(os.path.join(os.path.dirname(__file__), "..", "docs", "figures", "v4_euler.png"), dpi=120)
print(round(a[-1],1),round(b[-1],1))
