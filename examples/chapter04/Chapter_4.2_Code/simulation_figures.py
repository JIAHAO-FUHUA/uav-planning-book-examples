"""Editable English Figures 4.12--4.14 from the simulation JSON records."""
from pathlib import Path
import json,math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from figure_support import configure_fonts,save

ROOT=Path(__file__).resolve().parent
BLUE='#111111';GREEN='#444444';GRAY='#999999';ORANGE='#333333';RED='#222222'

def moving_grid(report):
    fig,axes=plt.subplots(2,2,figsize=(10.1,8.6))
    fig.subplots_adjust(left=.045,right=.985,top=.90,bottom=.25,wspace=.20,hspace=.65)
    for i,(ax,e) in enumerate(zip(axes.flat,report['snapshots'])):
        ax.set(xlim=(-.5,47.5),ylim=(-.5,31.5),aspect='equal');ax.axis('off')
        for x,y in e['blocked']:ax.add_patch(Rectangle((x-.5,y-.5),1,1,fc='#303030',ec='none'))
        for (x,y),blocked in e['changes']:
            ax.add_patch(Rectangle((x-.5,y-.5),1,1,fc='#BBBBBB' if blocked else 'white',ec='black',hatch='///',lw=.8,zorder=3))
        for path,color,ls,lw in [(e['previous_plan'],GRAY,'--',1.4),(e['executed'],GREEN,'-.',2.5),(e['path'],BLUE,'-',2.5)]:
            if path:
                a=np.array(path);ax.plot(a[:,0],a[:,1],c=color,ls=ls,lw=lw,zorder=4)
        c=e['current'];g=e['goal'];ax.plot(*c,'o',c=BLUE,ms=8,zorder=6);ax.plot(*g,'*',c=GREEN,ms=13,zorder=6)
        ax.annotate('Current',c,xytext=(-5,-22) if i<2 else (-44,12),textcoords='offset points',ha='center',fontsize=16)
        ax.annotate('T',g,xytext=(0,8),textcoords='offset points',ha='center',fontsize=18)
        if i==0:
            ax.text(25.5,25.5,'Upper passage',fontsize=15);ax.text(25.5,4.0,'Lower passage',fontsize=15)
        if e['status']=='hold':
            ax.text(3,28,'No route: keep current position',fontsize=17,color=RED,bbox={'fc':'white','ec':'none','pad':1})
            ax.annotate('Obsolete route is rejected',xy=(24,9),xytext=(25,18),ha='left',fontsize=15,color=RED,
                arrowprops={'arrowstyle':'->','color':RED,'lw':1.2})
        ax.set_title(f'{i+1}  Step {e["step"]}: {e["label"]}',fontsize=18,fontweight='bold',pad=12)
        pos=ax.get_position()
        length='No route' if e['remaining_cost'] is None else f'Remaining length = {e["remaining_cost"]:.3f}'
        fig.text((pos.x0+pos.x1)/2,pos.y0-.045,
            length+f'; current = ({c[0]},{c[1]})\nProcessed: D* Lite {e["dstar_processed"]}; fresh A* {e["astar_processed"]}',
            fontsize=16,ha='center',va='top',linespacing=1.25)
    fig.text(.5,.96,'Observe map changes before executing the next edge',fontsize=20,fontweight='bold',ha='center')
    fig.legend(handles=[Line2D([],[],c=GREEN,lw=2.5,ls='-.',label='Already executed'),
        Line2D([],[],c=BLUE,lw=2.5,label='Current planned route'),
        Line2D([],[],c=GRAY,ls='--',lw=1.4,label='Previous plan'),
        Rectangle((0,0),1,1,fc='white',ec='black',hatch='///',label='Changed cells'),
        Rectangle((0,0),1,1,fc='#303030',label='Occupied cells')],
        loc='lower center',bbox_to_anchor=(.5,.045),ncol=3,frameon=False,fontsize=16)
    fig.text(.5,.012,'Circle = current; star = T; step = grid edges; length = cell units; counts = Python queue removals.',ha='center',fontsize=15.0)
    save(fig,'fig_4_12_execution_replanning')

def box_faces(b):
    x0,x1,y0,y1,z0,z1=b
    a=[(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
       (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
    return [[a[j] for j in ids] for ids in [(0,1,2,3),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]]

def scene3d(report):
    result=report['display'][1];path=np.array(result['path'])
    fig=plt.figure(figsize=(10.1,5.7));fig.subplots_adjust(left=.02,right=.985,top=.81,bottom=.25,wspace=.25)
    ax=fig.add_subplot(1,2,1,projection='3d')
    for box in report['scene']['boxes']:
        poly=Poly3DCollection(box_faces(box),facecolors='#b1b7bd',edgecolors='#555555',alpha=.25,linewidths=.65)
        ax.add_collection3d(poly)
    ax.plot(path[:,0],path[:,1],path[:,2],c=BLUE,lw=2.4,zorder=6)
    for p,label,color in [(report['start'],'S',BLUE),(report['goal'],'T',GREEN)]:
        ax.scatter(*p,s=35,c=color,depthshade=False);ax.text(p[0],p[1],p[2]+.6,label,fontsize=17)
    ax.set(xlim=(0,14),ylim=(0,10),zlim=(.5,5.5));ax.set_box_aspect((14,10,5))
    ax.view_init(elev=24,azim=-65);ax.tick_params(labelsize=12,pad=0)
    ax.set_xlabel(r'$x$ (m)',fontsize=16,labelpad=4);ax.set_ylabel(r'$y$ (m)',fontsize=16,labelpad=4);ax.set_zlabel(r'$z$ (m)',fontsize=16,labelpad=2)
    ax.set_title('(a) One continuous 3D path',fontsize=18,fontweight='bold',pad=16)
    ax=fig.add_subplot(1,2,2)
    for i,b in enumerate(report['scene']['boxes']):
        x0,x1,y0,y1,z0,z1=b;ax.add_patch(Rectangle((x0,z0),x1-x0,z1-z0,fc='#b1b7bd',ec='#555555'))
        ax.text((x0+x1)/2,1.25 if i==0 else 4.5,'A' if i==0 else 'B',ha='center',fontsize=18,fontweight='bold')
    ax.axhline(1,c=RED,ls='--',lw=1.2)
    ax.plot(path[:,0],path[:,2],c=BLUE,lw=2.4)
    ax.plot(1,1,'o',c=BLUE,ms=7);ax.plot(13,1,'*',c=GREEN,ms=12)
    ax.annotate('S',(1,1),xytext=(-10,-20),textcoords='offset points',fontsize=17)
    ax.annotate('T',(13,1),xytext=(0,-20),textcoords='offset points',fontsize=17)
    ax.annotate('Climb above 3.4 m',xy=(6.5,3.6),xytext=(1.1,4.9),fontsize=16,
                arrowprops={'arrowstyle':'->','color':BLUE,'lw':1},color=BLUE)
    ax.annotate('Pass below 2.2 m',xy=(9.5,1.9),xytext=(10.4,3.0),fontsize=15.5,
                arrowprops={'arrowstyle':'->','color':BLUE,'lw':1},color=BLUE,ha='center')
    ax.set(xlim=(0,14),ylim=(.5,5.5));ax.set_xlabel(r'$x$ (m)',fontsize=16);ax.set_ylabel(r'$z$ (m)',fontsize=16)
    ax.tick_params(labelsize=14);ax.grid(alpha=.15)
    ax.set_title('(b) Side view makes the heights explicit',fontsize=17.5,fontweight='bold',pad=16)
    fig.text(.5,.95,r'Both inflated obstacles span $y$ = 0 to 10 m; sideways escape is excluded.',ha='center',fontsize=18)
    fig.legend(handles=[Line2D([],[],c=BLUE,lw=2.5,label='RRT* path (seed 17)'),
        Rectangle((0,0),1,1,fc='#b1b7bd',ec='#555555',label='Inflated closed box'),
        Line2D([],[],c=RED,ls='--',label=r'Fixed height $z=1$ m (blocked)')],
        loc='lower center',bbox_to_anchor=(.5,.06),ncol=3,frameon=False,fontsize=15.5)
    fig.text(.5,.012,f'3D length = {result["cost"]:.3f} m; every returned segment is collision-checked.',ha='center',fontsize=16.5)
    save(fig,'fig_4_13_3d_scene')

def sampling_comparison(report):
    fig,axes=plt.subplots(1,2,figsize=(10.1,4.5));fig.subplots_adjust(left=.065,right=.985,top=.78,bottom=.28,wspace=.25)
    a=axes[0]
    for r,color,ls in zip(report['display'],[GRAY,BLUE],['--','-']):
        values=np.array(r['history']);xx=values[:,0].tolist();yy=values[:,1].tolist()
        if xx[-1]<r['draws']:xx.append(r['draws']);yy.append(yy[-1])
        a.step(xx,yy,where='post',c=color,ls=ls,lw=2,label=r['algorithm'])
    first=report['display'][1]['first_draw']
    a.axvline(first,c='#555555',ls=':',lw=1)
    a.text(.035,.075,f'First route: draw {first}',transform=a.transAxes,fontsize=15)
    a.set(xlabel='Free-sample draws',ylabel='Best path length (m)',xlim=(0,report['settings']['budget']))
    a.set_title('(a) Continue both trees for 2000 draws',fontsize=18,fontweight='bold',pad=12)
    a.legend(frameon=False,fontsize=15.5,loc='upper right');a.grid(alpha=.15)
    a=axes[1];seeds=report['settings']['seeds'];xx=np.arange(len(seeds));barwidth=.32
    for shift,name,color in [(-barwidth/2,'RRT',GRAY),(barwidth/2,'RRT*',BLUE)]:
        values=[row['cost'] for row in report['seed_results'] if row['algorithm']==name]
        bars=a.bar(xx+shift,values,barwidth,color='white' if name=='RRT' else BLUE,edgecolor='black',hatch='///' if name=='RRT' else None,label=name)
        a.bar_label(bars,fmt='%.1f',fontsize=13,padding=2)
    a.set_xticks(xx,seeds);a.set(xlabel='Specified seed',ylabel='Final path length (m)',ylim=(0,32))
    a.set_title('(b) Repeat with five fixed seeds',fontsize=18,fontweight='bold',pad=12)
    for ax in axes:ax.tick_params(labelsize=14);ax.xaxis.label.set_size(16);ax.yaxis.label.set_size(16)
    fig.text(.5,.945,'Within each seed: identical accepted coordinates, different tree parents and costs.',ha='center',fontsize=18)
    fig.text(.5,.125,'Dashed / hatched = RRT; solid = RRT*. Both retain the best goal route.',ha='center',fontsize=15.5)
    fig.text(.5,.045,'All 5 seeds succeed for each method in this scene; this is not a general success-rate claim.',ha='center',fontsize=15.5)
    save(fig,'fig_4_14_3d_comparison')

if __name__=='__main__':
    configure_fonts()
    grid=json.loads((ROOT/'results/simulation_grid.json').read_text(encoding='utf-8'))
    three=json.loads((ROOT/'results/simulation_3d.json').read_text(encoding='utf-8'))
    if len(three['display'])!=2 or len(three['seed_results'])!=10:raise RuntimeError('First run simulations.py --case all with the default five seeds')
    moving_grid(grid);scene3d(three);sampling_comparison(three)
    print('Generated three English SVG/PNG figures from validated simulation data.')
