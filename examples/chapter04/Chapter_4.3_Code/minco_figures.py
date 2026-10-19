"""English, editable grayscale MINCO mechanism figures for Section 4.3.4."""
from pathlib import Path
import json
import numpy as np
from algorithm_figures import plt,save
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from matplotlib import font_manager
from minco import generate,system
from trajectory_generation import minimum_snap
ROOT=Path(__file__).resolve().parent
font_manager.findfont('Times New Roman',fallback_to_default=False)

def box(ax,x,y,w,h,label,size=17):
    ax.add_patch(Rectangle((x,y),w,h,fc='white',ec='black',lw=1.2))
    ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=size)
def arrow(ax,a,b):ax.annotate('',xy=b,xytext=a,arrowprops={'arrowstyle':'->','lw':1.6,'color':'black','mutation_scale':16})

def fig19():
    data=json.loads((ROOT/'results/minco_results.json').read_text());w=np.array(data['waypoints']);ts=np.array(data['durations'])
    tr=generate(w,ts);qp=minimum_snap(w,ts);_,p=tr.sample();_,pq=qp.sample(150)
    _,moved=generate(data['changed_waypoints'],ts).sample();_,retimed=generate(w,data['changed_durations']).sample()
    fig,axs=plt.subplots(1,2,figsize=(10.5,5.9));fig.subplots_adjust(left=.075,right=.985,bottom=.41,top=.83,wspace=.24)
    fig.suptitle('Minimum snap is an objective; MINCO is a parameterization',fontsize=22,weight='bold',y=.98)
    for ax in axs:ax.set(xlabel='x (m)',ylabel='y (m)',xlim=(-.12,2.12),ylim=(-.12,1.65));ax.grid(c='.86',lw=.5);ax.tick_params(labelsize=15)
    axs[0].set_title('(a) Same points and durations',fontsize=19)
    axs[0].plot(p[:,0],p[:,1],c='black',lw=2.5)
    axs[0].plot(pq[::15,0],pq[::15,1],ls='',marker='o',mfc='white',mec='black',ms=6)
    axs[0].scatter(w[:,0],w[:,1],marker='s',s=40,c='black',zorder=5)
    axs[0].text(.04,.92,'KKT and MINCO give the same curve',transform=axs[0].transAxes,fontsize=15)
    axs[0].legend(handles=[Line2D([],[],c='black',lw=2,label='MINCO'),Line2D([],[],c='black',mfc='white',marker='o',ls='',label='KKT samples'),Line2D([],[],c='black',marker='s',ls='',label='Specified point')],loc='upper left',bbox_to_anchor=(-.02,-.25),frameon=False,fontsize=14,ncol=2)
    axs[1].set_title('(b) Deform through sparse inputs',fontsize=19)
    axs[1].plot(p[:,0],p[:,1],c='black',lw=2.5)
    axs[1].plot(moved[:,0],moved[:,1],c='.35',ls='--',lw=2.3)
    axs[1].plot(retimed[:,0],retimed[:,1],c='.2',ls=':',lw=2.5)
    axs[1].scatter(w[:,0],w[:,1],marker='s',s=40,c='black',zorder=5)
    axs[1].plot(1,1.2,marker='s',mfc='white',mec='black',ms=7)
    axs[1].legend(handles=[Line2D([],[],c='black',lw=2,label='Base'),Line2D([],[],c='.35',ls='--',label='Move q'),Line2D([],[],c='.2',ls=':',label='Change times')],loc='upper left',bbox_to_anchor=(-.03,-.25),frameon=False,fontsize=14,ncol=2)
    ax=fig.add_axes([0,0,1,1]);ax.axis('off')
    for x,label in [(.04,'Outer inputs\nq and positive durations'),(.385,'Banded linear solve\nM(τ) C = B(q)'),(.73,'Reconstructed\ntimed trajectory')]:box(ax,x,.095,.23,.10,label,16)
    arrow(ax,(.27,.145),(.385,.145));arrow(ax,(.615,.145),(.73,.145))
    fig.text(.5,.042,'Two 3D segments: 48 coefficients reconstructed from 3 point coordinates + 2 durations.',ha='center',fontsize=15)
    fig.text(.5,.009,'Both panels show x–y projections at z = 1 m. Changing durations can also change the curve shape.',ha='center',fontsize=14)
    save(fig,'fig_4_19_minco_parameterization')

def fig20():
    a,_,_=system([[0,0,0],[1,.8,0],[2,.5,0],[3,0,0]],[1.2,1.5,1.8]);mask=np.abs(a.dense())>0
    fig=plt.figure(figsize=(10.5,5.9));fig.suptitle('One forward solve and one adjoint solve per objective evaluation',fontsize=21,weight='bold',y=.985)
    ax=fig.add_axes([.07,.23,.36,.63]);ax.imshow(mask,cmap='gray_r',interpolation='nearest',origin='upper',extent=(0,24,24,0));ax.set_xticks([0,8,16,24]);ax.set_yticks([0,4,12,20,24]);ax.tick_params(labelsize=15)
    ax.set(xlabel='Coefficient columns',ylabel='Equation rows');ax.set_title('Three segments: 24 × 24 system',fontsize=18,pad=14)
    for x in (8,16):ax.axvline(x,c='.6',lw=.8)
    for y in (4,12,20):ax.axhline(y,c='.6',lw=.8)
    flow=fig.add_axes([.49,.16,.49,.71]);flow.set(xlim=(0,1),ylim=(0,1));flow.axis('off')
    labels=['1  Assemble and factor M; solve for C','2  Evaluate objective and partial gradients','3  Solve Mᵀ G = ∂k / ∂C using the factors','4  Gather gradients of points and durations']
    for i,label in enumerate(labels):
        y=.79-i*.23;box(flow,.015,y,.97,.16,label,16)
        if i<3:arrow(flow,(.5,y),(.5,y-.07))
    fig.text(.735,.085,'An outer optimizer updates q and τ, then repeats.',ha='center',fontsize=16)
    fig.text(.5,.026,'Black cells = nonzero entries; gray rules = segment / row blocks. Fixed band width gives O(m) solves.',ha='center',fontsize=15)
    save(fig,'fig_4_20_minco_solve')

if __name__=='__main__':fig19();fig20();print('Figures 4.19–4.20 generated as editable English grayscale SVG/PNG.')
