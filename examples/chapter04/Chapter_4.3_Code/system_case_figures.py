"""English editable monochrome system mechanisms and actual ROS traces."""
from pathlib import Path
import json
import numpy as np
from algorithm_figures import plt,save
from matplotlib.patches import Rectangle,Patch
from matplotlib.lines import Line2D
ROOT=Path(__file__).resolve().parent

def arrow(ax,a,b):ax.annotate('',xy=b,xytext=a,arrowprops={'arrowstyle':'->','lw':1.6,'color':'black','mutation_scale':15})
def box(ax,x,y,w,h,label,gray=False):
    ax.add_patch(Rectangle((x,y),w,h,fc='.88' if gray else 'white',ec='black',lw=1.3));ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=17)

def fig24():
    fig,ax=plt.subplots(figsize=(10.5,3.8));fig.subplots_adjust(left=.025,right=.985,bottom=.18,top=.8);ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
    fig.suptitle('Fast-Planner: a search supplies the route; ESDF gradients refine it',fontsize=21,weight='bold',y=.97)
    labels=['Occupancy\nthen ESDF','Kinodynamic\nA* seed','Optimize B-spline\ncontrol points','Adjust knot times\nand check']
    for i,label in enumerate(labels):
        x=.015+i*.252;box(ax,x,.52,.22,.32,label,i==2)
        if i<3:arrow(ax,(x+.22,.68),(x+.252,.68))
    box(ax,.3,.04,.4,.23,'Timestamped trajectory → execution')
    arrow(ax,(.87,.52),(.87,.16));arrow(ax,(.87,.16),(.7,.16));arrow(ax,(.3,.16),(.125,.16));arrow(ax,(.125,.16),(.125,.52))
    fig.text(.5,.07,'White = representation / processing stage; gray = nonlinear optimization. The lower loop repeats online.',ha='center',fontsize=15)
    save(fig,'fig_4_24_fast_pipeline')

def trajectory_figure(key,number,title):
    data=json.loads((ROOT/f'results/ros_{key}/result.json').read_text());a=np.array(data['trace']);scene=data['scene']
    fig,axs=plt.subplots(1,2,figsize=(10.5,4.6));fig.subplots_adjust(left=.075,right=.98,bottom=.29,top=.79,wspace=.27)
    fig.suptitle(title,fontsize=21,weight='bold',y=.97)
    ax,b=axs
    for box_ in scene['boxes']:
        lo=box_['low'];hi=box_['high'];ax.add_patch(Rectangle(lo[:2],hi[0]-lo[0],hi[1]-lo[1],fc='.65',ec='black'))
    xy=a[:,1:3].copy();jumps=np.flatnonzero(np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1)>.2)
    for j in jumps:xy[j+1]=np.nan
    ax.plot(xy[:,0],xy[:,1],c='black',lw=2.2);ax.plot(*scene['start'][:2],'o',mfc='white',mec='black',ms=7);ax.plot(*scene['goal'][:2],'*',c='black',ms=11)
    for j in jumps:
        ax.plot(a[j:j+2,1],a[j:j+2,2],c='.4',ls='--',lw=1.4)
        ax.annotate('Command jump ≈ 2.10 m',xy=a[j+1,1:3],xytext=(.7,-3.4),fontsize=13,arrowprops={'arrowstyle':'->','color':'black'})
    ax.set(xlim=(-8,8),ylim=(-4,4),xlabel='x (m)',ylabel='y (m)');ax.set_aspect('equal');ax.grid(c='.9',lw=.6);ax.tick_params(labelsize=14);ax.set_title('(a) Recorded x–y trace',fontsize=19)
    ax.annotate('',xy=a[len(a)//2,1:3],xytext=a[len(a)//2-25,1:3],arrowprops={'arrowstyle':'->','lw':1.7})
    b.plot(a[:,0],a[:,3],c='black',lw=2,label='Height');b.set(xlabel='Recorder time (s)',ylabel='Height z (m)',ylim=(-.2,3.2));b.set_title('(b) Recorded height',fontsize=19);b.grid(c='.9',lw=.6);b.tick_params(labelsize=14)
    if key=='fast':
        b.axhline(.3,c='.5',ls='--',lw=1.3);b.text(.4,.38,'Assessment floor = 0.30 m',fontsize=13)
    b.axhline(3,c='.4',ls=':',lw=1.3);b.text(.5,2.7,'Obstacle top = 3 m',fontsize=15)
    fig.legend(handles=[Patch(fc='.65',ec='black',label='Obstacle footprint'),Line2D([],[],c='black',lw=2,label='Executed reference'),Line2D([],[],marker='o',mfc='white',mec='black',ls='',label='Start'),Line2D([],[],c='black',marker='*',ls='',label='Goal')],loc='lower center',bbox_to_anchor=(.5,.04),ncol=4,frameon=False,fontsize=15)
    save(fig,f'fig_4_{number}_{key}_simulation')

def fig26():
    fig=plt.figure(figsize=(10.5,4.9));fig.suptitle('EGO-Planner-v2: build a local escape direction where the curve collides',fontsize=21,weight='bold',y=.98)
    a=fig.add_axes([.04,.3,.43,.49]);a.set(xlim=(0,5),ylim=(-.2,2.5));a.set_aspect('equal');a.axis('off')
    a.add_patch(Rectangle((2.2,0),1,1.6,fc='.65',ec='black'))
    a.plot([.2,4.8],[.75,.75],c='black',ls=':',lw=2)
    a.plot([.2,1.6,2,3.6,4.8],[.75,.75,2,2,.75],c='black',ls='--',lw=1.8)
    a.plot(2.65,.75,'x',c='black',ms=8,mew=2);a.plot(2.65,1.6,'o',c='black',ms=5)
    arrow(a,(2.65,1.6),(2.65,2.25));a.text(2.85,2.1,'Escape direction',fontsize=15)
    a.text(.25,.25,'Colliding curve point',fontsize=15);a.text(.55,2.25,'A* detour',fontsize=15)
    b=fig.add_axes([.53,.24,.45,.59]);b.set(xlim=(0,1),ylim=(0,1));b.axis('off')
    for i,label in enumerate(['Positions + positive durations','MINCO quintic reconstruction','Jerk / time / obstacle / limit costs','L-BFGS update; check and restart']):
        y=.78-i*.24;box(b,.015,y,.97,.18,label,gray=i in (1,2))
        if i<3:arrow(b,(.5,y),(.5,y-.06))
    fig.legend(handles=[Patch(fc='.65',ec='black',label='Inflated obstacle'),Line2D([],[],c='black',ls=':',label='Colliding seed'),Line2D([],[],c='black',ls='--',label='Free guide path'),Line2D([],[],c='black',marker='o',ls='',label='Local anchor')],loc='lower center',bbox_to_anchor=(.5,.03),ncol=4,frameon=False,fontsize=14)
    save(fig,'fig_4_26_ego_constraint')

def fig28():
    metrics=json.loads((ROOT/'results/ros_case_comparison.json').read_text());fig,axs=plt.subplots(1,2,figsize=(10.5,4.8));fig.subplots_adjust(left=.08,right=.98,bottom=.28,top=.79,wspace=.30)
    fig.suptitle('Goal arrival alone does not establish clearance or a strict speed bound',fontsize=21,weight='bold',y=.97)
    for key,style in [('fast','-'),('ego','--')]:
        data=json.loads((ROOT/f'results/ros_{key}/result.json').read_text());a=np.array(data['trace'])
        axs[0].plot(a[:,0],metrics[key]['point_distances_m'],c='black',ls=style,lw=2)
        axs[1].plot(a[:,0],np.linalg.norm(a[:,4:7],axis=1),c='black',ls=style,lw=2)
    axs[0].axhline(.4,c='.4',ls=':',lw=1.5);axs[0].text(7,.18,'Revised: 0.40 m',fontsize=14)
    axs[0].set(ylim=(0,2),xlabel='Recorder time (s)',ylabel='Boundary clearance (m)');axs[0].set_title('(a) Reassess the same traces',fontsize=19)
    axs[1].axhline(1.5,c='.4',ls=':',lw=1.5);axs[1].set(ylim=(0,1.65),xlabel='Recorder time (s)',ylabel='Command velocity norm (m/s)');axs[1].set_title('(b) Nominal setting: 1.5 m/s',fontsize=19)
    for ax in axs:ax.grid(c='.9',lw=.6);ax.tick_params(labelsize=14)
    fig.legend(handles=[Line2D([],[],c='black',ls='-',lw=2,label='Fast-Planner'),Line2D([],[],c='black',ls='--',lw=2,label='EGO-Planner-v2'),Line2D([],[],c='.4',ls=':',label='Assessment threshold')],loc='lower center',bbox_to_anchor=(.5,.035),ncol=3,frameon=False,fontsize=15)
    save(fig,'fig_4_28_validation_comparison')

if __name__=='__main__':
    fig24();trajectory_figure('fast',25,'Fast-Planner: local ROS obstacle-avoidance example');fig26()
    trajectory_figure('ego',27,'EGO-Planner-v2: local ROS obstacle-avoidance example');fig28()
    print('Figures 4.24--4.28 generated.')
