import glob, os, numpy as np, cadquery as cq
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

D='/home/umran/tiltrotor_project_updates/cad/step/parts/'
tris=[]
for f in sorted(glob.glob(D+'*.step')):
    s=cq.importers.importStep(f).val()
    v,t=s.tessellate(1.0)
    P=np.array([[p.x,p.y,p.z] for p in v])
    tris.append(P[np.array(t)])
T=np.vstack(tris); print('üçgen',len(T))

views=[('Üstten (x-y)',0,1,2,'x [mm]','y [mm]'),
       ('Yandan (x-z)',0,2,1,'x [mm]','z [mm]'),
       ('Önden (y-z)',1,2,0,'y [mm]','z [mm]')]
fig,axes=plt.subplots(3,1,figsize=(11,13))
for ax,(title,a,b,depth,xl,yl) in zip(axes,views):
    order=np.argsort(T[:,:,depth].mean(1))
    polys=T[order][:,:,[a,b]]
    n=np.cross(T[order][:,1]-T[order][:,0],T[order][:,2]-T[order][:,0])
    nn=np.abs(n[:,depth])/(np.linalg.norm(n,axis=1)+1e-12)
    col=plt.cm.Blues(0.35+0.55*nn)
    ax.add_collection(PolyCollection(polys,facecolors=col,edgecolors='none'))
    ax.set_title(title); ax.set_xlabel(xl); ax.set_ylabel(yl)
    ax.set_aspect('equal'); ax.autoscale_view()
    ax.set_xlim(polys[:,:,0].min()-30,polys[:,:,0].max()+30)
    ax.set_ylim(polys[:,:,1].min()-30,polys[:,:,1].max()+30)
    ax.grid(alpha=.3)
plt.tight_layout(); plt.savefig('/home/umran/tiltrotor_project_updates/cad/onizleme.png',dpi=110)
print('yazıldı')
