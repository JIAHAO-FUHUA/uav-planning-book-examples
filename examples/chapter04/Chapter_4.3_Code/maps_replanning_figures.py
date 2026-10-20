"""Editable English monochrome mechanism figures, with explicit legends."""
import numpy as np
from algorithm_figures import plt,save
from matplotlib.patches import Rectangle,Circle as CirclePatch,Patch
from matplotlib.lines import Line2D
from maps_replanning import Circle,handoff_case,at

def arrow(ax,a,b):
    ax.annotate('',xy=b,xytext=a,arrowprops={'arrowstyle':'->','color':'black','lw':1.8,'mutation_scale':16})

def fig21():
    fig,axs=plt.subplots(1,2,figsize=(10.5,4.7));fig.subplots_adjust(left=.055,right=.98,bottom=.29,top=.78,wspace=.25)
    fig.suptitle('Occupancy says “blocked?”; distance adds “how far and which way?”',fontsize=21,weight='bold',y=.97)
    a,b=axs
    for ax in axs:ax.set_aspect('equal');ax.set_xlim(0,5);ax.set_ylim(0,3);ax.set_xlabel('x (m)',fontsize=18);ax.set_ylabel('y (m)',fontsize=18);ax.tick_params(labelsize=15)
    a.set_title('(a) One valid return in a voxel slice',fontsize=18)
    a.add_patch(Rectangle((0,0),5,3,fc='.95',ec='black',hatch='////',lw=.7))
    for ix in range(4):a.add_patch(Rectangle((ix,1),1,1,fc='white',ec='.6',lw=.6))
    a.add_patch(Rectangle((3,1),1,1,fc='.45',ec='black'))
    a.plot([.45,3.45],[1.5,1.5],c='black',lw=1.7);arrow(a,(1.3,1.5),(2.4,1.5))
    a.plot(.45,1.5,'^',c='black',ms=9);a.plot(3.45,1.5,'x',c='black',ms=9,mew=2)
    a.text(.35,2.2,'Sensor',fontsize=16);a.text(3.05,2.2,'Hit',fontsize=16)
    a.text(4.42,1.5,'?',ha='center',fontsize=26,bbox={'fc':'white','ec':'none','pad':0})
    b.set_title('(b) Ideal ESDF in known space',fontsize=18)
    theta=np.linspace(0,2*np.pi,360)
    for level in (.5,1.,1.5):
        radius=.55+level;b.plot(2.6+radius*np.cos(theta),1.5+radius*np.sin(theta),c='black',ls='--',lw=1.)
        b.text(2.6+radius*.72,1.5-radius*.69,f'{level:g} m',fontsize=13,bbox={'fc':'white','ec':'none','pad':0})
    b.add_patch(CirclePatch((2.6,1.5),.55,fc='.55',ec='black'))
    b.plot(1.3,1.5,'o',c='black',ms=6);arrow(b,(1.3,1.5),(.45,1.5));b.text(.2,2.72,'Gradient: more clearance',fontsize=15,bbox={'fc':'white','ec':'none','pad':0})
    b.text(2.6,1.5,'d < 0',ha='center',va='center',fontsize=15)
    fig.legend(handles=[Patch(fc='white',ec='black',label='Free evidence'),Patch(fc='.45',ec='black',label='Occupied'),Patch(fc='.95',ec='black',hatch='////',label='Unknown'),Line2D([],[],c='black',ls='--',label='Distance contour'),Line2D([],[],c='black',marker='o',label='Query / gradient')],loc='lower center',bbox_to_anchor=(.5,.005),ncol=3,frameon=False,fontsize=15)
    save(fig,'fig_4_21_map_distance')

def fig22():
    fig,axs=plt.subplots(1,2,figsize=(10.5,4.5));fig.subplots_adjust(left=.065,right=.98,bottom=.30,top=.78,wspace=.25)
    fig.suptitle('A descent direction improves a curve; clear samples do not certify it',fontsize=21,weight='bold',y=.98)
    for ax in axs:
        ax.set_aspect('equal');ax.set_xlim(-.2,4.2);ax.set_ylim(-.85,.9);ax.set_yticks([0]);ax.set_xlabel('x (m)',fontsize=17);ax.tick_params(labelsize=15)
        ax.add_patch(CirclePatch((2,0),.4,fc='.55',ec='black'))
        ax.add_patch(CirclePatch((2,0),.7,fc='none',ec='black',ls='--',lw=1.5))
    a,b=axs;a.set_title('(a) Penalty descent at one point',fontsize=18);b.set_title('(b) Endpoint samples miss a collision',fontsize=18)
    a.plot(1.4,0,'o',c='black',ms=7);arrow(a,(1.38,0),(.55,0))
    a.text(.15,.71,'Descent points away',fontsize=16,bbox={'fc':'white','ec':'none','pad':0});a.text(.1,-.78,'d = 0.2 m < required 0.3 m',fontsize=16,bbox={'fc':'white','ec':'none','pad':0})
    b.plot([0,4],[0,0],c='black',lw=2);b.plot([0,4],[0,0],ls='',marker='o',mfc='white',mec='black',ms=8)
    arrow(b,(2.8,0),(3.4,0));b.text(.05,.71,'Both endpoint distances = 1.6 m',fontsize=16,bbox={'fc':'white','ec':'none','pad':0})
    b.annotate('Collision between samples',xy=(2,0),xytext=(.35,-.78),fontsize=16,arrowprops={'arrowstyle':'->','lw':1.2},bbox={'fc':'white','ec':'none','pad':0})
    fig.legend(handles=[Patch(fc='.55',ec='black',label='Obstacle'),Line2D([],[],c='black',ls='--',label='Required clearance boundary'),Line2D([],[],c='black',marker='o',mfc='white',ls='',label='Checked sample')],loc='lower center',bbox_to_anchor=(.5,.035),ncol=3,fontsize=15,frameon=False)
    save(fig,'fig_4_22_collision_check')

def fig23():
    fig=plt.figure(figsize=(10.5,5.5));fig.suptitle('Compute ahead; switch once at a shared state and clock time',fontsize=21,weight='bold',y=.98)
    a=fig.add_axes([.065,.31,.41,.46]);a.set(xlim=(.85,2.1),ylim=(-.25,1.8));a.set_yticks([]);a.set_xlabel('Absolute time (s)',fontsize=17);a.set_xticks([1.,1.3,1.6,1.9]);a.tick_params(labelsize=16);a.set_title('(a) Timeline',fontsize=19)
    for edge in ('left','right','top'):a.spines[edge].set_visible(False)
    a.plot([.9,1.6],[.1,.1],c='black',lw=4);a.plot([1.6,2.05],[.1,.1],c='black',lw=3,ls='--')
    a.plot([1.,1.3],[1.,1.],c='.45',lw=9,solid_capstyle='butt');a.plot([1.3,1.55],[1.,1.],c='black',lw=1.5);a.axvline(1.6,c='black',lw=1)
    a.text(.99,1.43,'Request',ha='center',fontsize=15);a.text(1.3,1.43,'Ready',ha='center',fontsize=15);a.text(1.77,1.4,'Switch',ha='center',fontsize=16)
    a.text(1.13,.7,'Solve',fontsize=15,ha='center');a.text(1.45,.4,'Validate',fontsize=15,ha='center')
    a.text(1.9,.54,'Reject\nlate result',ha='center',fontsize=15)
    a.plot(1.85,1.,'x',c='black',ms=9,mew=2)
    b=fig.add_axes([.56,.25,.41,.55]);tr,_=handoff_case();_,p=tr.sample(240);b.set_title('(b) Fixed-height trajectory',fontsize=19)
    b.plot([.6,1.6],[0,0],c='black',lw=3);b.plot([1.6,5],[0,0],c='.5',lw=1.5,ls=':')
    b.plot(p[:,0],p[:,1],c='black',ls='--',lw=2.5)
    b.add_patch(CirclePatch((2.7,0),.35,fc='.55',ec='black'));b.add_patch(CirclePatch((2.7,0),.55,fc='none',ec='black',ls='-.',lw=1.5))
    b.plot(1.6,0,'o',mfc='white',mec='black',ms=9,zorder=5);arrow(b,(1.6,0),(2.02,0))
    b.annotate('Same p, v, a, jerk',xy=(1.6,0),xytext=(.6,2.25),fontsize=16,arrowprops={'arrowstyle':'->','lw':1.2})
    b.plot(2.1,1.3,'s',c='black',ms=5);b.text(2.35,1.05,'W',fontsize=16)
    b.text(4.25,.35,'Stop',fontsize=15);b.plot(5,0,'s',c='black',ms=5)
    b.set(xlim=(.55,5.25),ylim=(-.65,2.6),xlabel='x (m)',ylabel='y (m)');b.set_aspect('equal');b.tick_params(labelsize=15);b.grid(c='.9',lw=.5)
    fig.legend(handles=[Line2D([],[],c='black',lw=3,label='Executed old prefix'),Line2D([],[],c='black',ls='--',label='New suffix'),Line2D([],[],c='.5',ls=':',label='Discarded old suffix'),Patch(fc='.55',ec='black',label='Obstacle'),Line2D([],[],c='black',ls='-.',label='Required clearance')],loc='lower center',bbox_to_anchor=(.5,.015),ncol=3,fontsize=15,frameon=False)
    save(fig,'fig_4_23_replanning_handoff')

if __name__=='__main__':fig21();fig22();fig23();print('Figures 4.21--4.23 generated.')
