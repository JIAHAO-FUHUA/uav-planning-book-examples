"""Generate the editable, English Figures 4.8--4.11 from recorded experiments."""
from pathlib import Path
import json, math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D
from figure_support import configure_fonts, save
from sampling_dynamics import propagate, Scene, CONTROLS, primitive_status

ROOT=Path(__file__).resolve().parent
BLUE='#111111'; GREEN='#444444'; RED='#222222'; AMBER='#333333'; GRAY='#AAAAAA'

def map_axes(ax,scene,obstacle_fill='#303030'):
    ax.set(xlim=(0,scene['width']),ylim=(0,scene['height']),aspect='equal')
    for x0,x1,y0,y1 in scene['obstacles']:
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fc=obstacle_fill,ec='#303030',zorder=3))
    ax.set_xlabel(r'$x$ (m)',fontsize=16);ax.set_ylabel(r'$y$ (m)',fontsize=16)
    ax.tick_params(labelsize=14);ax.grid(alpha=.12)

def endpoints(ax,start,goal):
    ax.plot(*start,'o',c=BLUE,ms=7,zorder=8);ax.plot(*goal,'*',c=GREEN,ms=12,zorder=8)
    ax.annotate('S',start,xytext=(-1,-16),textcoords='offset points',fontsize=17,ha='center')
    ax.annotate('T',goal,xytext=(0,-16),textcoords='offset points',fontsize=17,ha='center')

def sampling(report):
    fig,axes=plt.subplots(1,3,figsize=(10.1,5.8))
    fig.subplots_adjust(left=.055,right=.985,top=.64,bottom=.28,wspace=.27)
    specs=[('PRM: build, then query','1 Sample free points\n2 Connect checked neighbors\n3 Attach S/T; search roadmap',
            '250 samples; 12 neighbors\nReusable roadmap'),
           ('RRT: grow until a route','1 Sample; find nearest node\n2 Steer at most 1 m\n3 Check edge; add if free',
            'First route at draw 53\n44 tree nodes'),
           ('RRT*: grow and improve','1 Extend as in RRT\n2 Choose cheapest free parent\n3 Rewire nearby nodes',
            '600 draws; 539 tree nodes\n436 successful rewires')]
    for ax,r,(title,steps,ending) in zip(axes,report['sampling'],specs):
        map_axes(ax,report['scene'])
        for i,j in r['edges']:
            a,b=r['nodes'][i],r['nodes'][j];ax.plot([a[0],b[0]],[a[1],b[1]],c=GRAY,lw=.55,zorder=1)
        p=np.array(r['path']);ax.plot(p[:,0],p[:,1],c=BLUE,lw=2.5,zorder=5)
        endpoints(ax,report['start'],report['goal'])
        pos=ax.get_position()
        fig.text((pos.x0+pos.x1)/2,.955,title,ha='center',fontsize=18,fontweight='bold')
        fig.text(pos.x0,.88,steps,fontsize=16,va='top',linespacing=1.35)
        fig.text((pos.x0+pos.x1)/2,.195,f'Path length = {r["cost"]:.3f} m\n'+ending,
                ha='center',va='top',fontsize=16,linespacing=1.2)
    fig.legend(handles=[Line2D([],[],c=BLUE,lw=2.5,label='Returned path'),
        Line2D([],[],c=GRAY,lw=1,label='Checked graph / tree edge'),
        Rectangle((0,0),1,1,fc='#303030',label='Inflated obstacle')],
        loc='lower center',bbox_to_anchor=(.5,.002),ncol=3,frameon=False,fontsize=15)
    save(fig,'fig_4_8_sampling')

def rewiring():
    pts={'S':(0.,0.),'A':(0.,3.),'B':(3.,3.),'N':(2.3,1.2)}
    d=lambda a,b:math.dist(pts[a],pts[b])
    gn=d('S','N');old=6+d('B','N');new=gn+d('N','B')
    fig,axes=plt.subplots(1,3,figsize=(10.1,4.5))
    fig.subplots_adjust(left=.018,right=.995,top=.78,bottom=.34,wspace=.1)
    def edge(ax,a,b,color=GRAY,style='-',width=1.7):
        ax.annotate('',xy=pts[b],xytext=pts[a],arrowprops={'arrowstyle':'-|>',
            'color':color,'lw':width,'linestyle':style,'shrinkA':9,'shrinkB':9})
    for i,ax in enumerate(axes):
        ax.set(xlim=(-.55,3.65),ylim=(-.65,3.6),aspect='equal');ax.axis('off')
        for lab,(x,y) in pts.items():
            ax.plot(x,y,'o',c=BLUE if lab=='N' else '#333333',ms=8)
            ax.text(x-.12,y+.2,lab,fontsize=19,ha='right')
        edge(ax,'S','A');edge(ax,'A','B',GRAY,'--' if i==2 else '-')
        ax.text(-.42,1.45,'3',fontsize=16);ax.text(1.45,3.2,'3',fontsize=16)
        if i==0:
            edge(ax,'B','N',AMBER,':')
            ax.text(2.88,1.95,f'{d("B","N"):.3f}',fontsize=15)
            title='1 Nearest is only a candidate'
            note='Nearest node: B; old '+r'$g(\mathrm{B})=6$'+'\nVia B: '+rf'$g(\mathrm{{N}})=7.931$'
        elif i==1:
            edge(ax,'B','N',AMBER,':');edge(ax,'S','N',BLUE,width=2.3)
            ax.text(.95,.16,f'{gn:.3f}',fontsize=15,color=BLUE)
            title='2 Select the cheaper parent'
            note='Compare free local connections\nChoose S: '+rf'$g(\mathrm{{N}})={gn:.3f}$'
        else:
            edge(ax,'S','N',GREEN,'-.',width=2.3);edge(ax,'N','B',GREEN,'-.',width=2.3)
            ax.text(2.83,1.95,f'{d("N","B"):.3f}',fontsize=15,color=GREEN)
            title='3 Rewire B through N'
            note='New '+r'$g(\mathrm{B})=g(\mathrm{N})+1.931$'+'\n'+rf'$\approx {new:.3f}<6$'+'; remove A to B'
        ax.set_title(title,fontsize=17,fontweight='bold',pad=12)
        ax.text(.5,-.015,note,transform=ax.transAxes,ha='center',va='top',fontsize=16,linespacing=1.3)
    fig.text(.5,.95,'Local radius = 3; all drawn connections are collision-free',ha='center',fontsize=17)
    fig.legend(handles=[Line2D([],[],c=GRAY,lw=1.7,label='Existing edge'),
        Line2D([],[],c=AMBER,lw=1.7,ls=':',label='Candidate'),Line2D([],[],c=BLUE,lw=2,label='Chosen parent'),
        Line2D([],[],c=GREEN,lw=2,ls='-.',label='Rewired route'),Line2D([],[],c=GRAY,ls='--',label='Removed edge')],
        loc='lower center',bbox_to_anchor=(.5,.005),ncol=3,frameon=False,fontsize=16)
    save(fig,'fig_4_9_rewiring')
    return {'local_radius':3,'g_N_via_B':old,'g_N_via_S':gn,'g_B_rewired':new}

def primitive_fan(report):
    fig,axes=plt.subplots(1,2,figsize=(10.1,5.0))
    fig.subplots_adjust(left=.05,right=.98,top=.77,bottom=.30,wspace=.29)
    ax=axes[0];ax.set(xlim=(-.5,4.5),ylim=(0,4),aspect='equal');ax.axis('off')
    ax.plot(2,2,'o',ms=9,c='#303030')
    ax.text(2,2.36,'Same position: '+r'$\mathbf{p}=(2,2)$',ha='center',fontsize=18)
    for end,label,y,color in [(4,r'$\mathbf{v}_1=(2,0)$',3.2,BLUE),
                             (0,r'$\mathbf{v}_2=(-2,0)$',1.25,GREEN)]:
        ax.annotate('',xy=(end,2),xytext=(2,2),arrowprops={'arrowstyle':'-|>','lw':3,'color':color})
        ax.text((2+end)/2,y,label,ha='center',fontsize=17,color=color)
    ax.text(2,.15,'Different velocities lead to different\nreachable successors in the next second.',
            ha='center',fontsize=16,linespacing=1.25)
    ax.set_title('(a) Position alone is insufficient',fontsize=18,fontweight='bold',pad=17)
    ax=axes[1];scene={'width':6,'height':5,'obstacles':[(3.3,4.6,1.7,2.3)]};map_axes(ax,scene,obstacle_fill='#D0D0D0')
    state=(2,2,2,0);times=np.linspace(0,1,100)
    style={'legal':(BLUE,'-',2.5),'collision':(RED,'--',1.7),'speed':(AMBER,':',2.2)}
    for item in report['fan']:
        u=item['u'];path=np.array([propagate(state,u,t) for t in times]);color,ls,lw=style[item['status']]
        ax.plot(path[:,0],path[:,1],c=color,ls=ls,lw=lw,zorder=4)
        marker={'legal':'o','collision':'x','speed':'s'}[item['status']]
        ax.plot(*path[-1,:2],marker,c=color,ms=5,zorder=5)
        if item['status']=='legal':
            destination=(4.15,3.3 if u[1]>0 else .7)
            ax.annotate(r'$\mathbf{u}=(-1,'+('1' if u[1]>0 else '-1')+')$',
                xy=path[-1,:2],xytext=destination,fontsize=15,color=BLUE,
                arrowprops={'arrowstyle':'-','lw':.8,'color':BLUE},ha='center')
    ax.plot(2,2,'o',c='#303030',ms=7,zorder=5)
    ax.annotate('',xy=(3,2),xytext=(2,2),arrowprops={'arrowstyle':'-|>','color':'#303030','lw':2})
    ax.text(.4,4.5,r'$\mathbf{v}=(2,0)$ m/s; '+r'$\tau=1$ s',fontsize=16)
    ax.set_title('(b) Propagate 9 acceleration choices',fontsize=18,fontweight='bold',pad=17)
    fig.text(.5,.935,r'$u_x,u_y\in\{-1,0,1\}$ m/s$^2$; speed limit = 2.5 m/s; acceleration limit = 1.5 m/s$^2$',
             ha='center',fontsize=17)
    fig.legend(handles=[Line2D([],[],c=BLUE,lw=2.5,marker='o',label='Legal (2)'),
        Line2D([],[],c=RED,ls='--',lw=1.7,marker='x',label='Collision (4)'),
        Line2D([],[],c=AMBER,ls=':',lw=2.2,marker='s',label='Speed violation (3)'),
        Rectangle((0,0),1,1,fc='#D0D0D0',ec='black',label='Inflated obstacle')],
        loc='lower center',bbox_to_anchor=(.5,.095),ncol=4,frameon=False,fontsize=15)
    fig.text(.5,.015,'Arrow = initial velocity; markers = primitive endpoints. Speed is checked first.',
             ha='center',fontsize=15.5)
    save(fig,'fig_4_10_primitives')

def kino_result(report):
    result=report['kino'];tt=[];xy=[];speed=[]
    for i,(x,u) in enumerate(zip(result['states'],result['controls'])):
        for t in np.linspace(0,1,51):
            y=propagate(x,u,t);tt.append(i+t);xy.append(y[:2]);speed.append(math.hypot(*y[2:]))
    xy=np.array(xy);fig=plt.figure(figsize=(10.1,5.6))
    gs=fig.add_gridspec(2,2,width_ratios=[1.1,1],hspace=.55,wspace=.32)
    fig.subplots_adjust(left=.055,right=.985,top=.80,bottom=.29)
    ax=fig.add_subplot(gs[:,0]);map_axes(ax,report['scene'])
    poly=np.array(report['sampling'][0]['path']);ax.plot(poly[:,0],poly[:,1],c=GRAY,ls='--',lw=1.5)
    ax.plot(xy[:,0],xy[:,1],c=BLUE,lw=2.5,zorder=5)
    states=np.array(result['states']);ax.plot(states[:,0],states[:,1],'o',c=BLUE,ms=4,zorder=5)
    endpoints(ax,report['start'],report['goal'])
    ax.annotate('',xy=(3,3),xytext=(1,3),arrowprops={'arrowstyle':'-|>','lw':2,'color':GREEN})
    ax.text(.8,1.6,'Initial speed:\n2 m/s',fontsize=15,va='top');ax.text(8.8,1.6,'Terminal speed:\n0 m/s',fontsize=15,va='top')
    ax.set_title('(a) Search in position and velocity',fontsize=18,fontweight='bold',pad=10)
    axv=fig.add_subplot(gs[0,1]);axa=fig.add_subplot(gs[1,1])
    axv.plot(tt,speed,c=BLUE,lw=2.2);axv.axhline(2.5,c=RED,ls='--',lw=1.5)
    axv.set(ylim=(-.08,3.1),xlim=(0,8),ylabel='Speed (m/s)')
    axv.set_title('(b) Motion limits over the entire route',fontsize=18,fontweight='bold',pad=12)
    axv.text(4.2,2.7,'Speed limit: 2.5',fontsize=14,color=RED)
    effort=[math.hypot(*u) for u in result['controls']]
    axa.step(range(9),effort+[effort[-1]],where='post',c=BLUE,lw=2.2)
    axa.axhline(1.5,c=RED,ls='--',lw=1.5)
    axa.set(ylim=(-.06,1.95),xlim=(0,8),ylabel='Acceleration (m/s²)',xlabel='Time (s)')
    axa.text(.3,1.66,'Acceleration limit: 1.5',fontsize=14,color=RED)
    for a in [axv,axa]:a.tick_params(labelsize=14);a.yaxis.label.set_size(16);a.xaxis.label.set_size(16);a.grid(alpha=.15)
    fig.text(.5,.94,'8 one-second primitives; duration = 8 s; time + weighted effort = 10.4 s',ha='center',fontsize=18)
    fig.legend(handles=[Line2D([],[],c=BLUE,lw=2.5,label='Kinodynamic trajectory'),
        Line2D([],[],c=GRAY,ls='--',lw=1.5,label='PRM geometric reference'),
        Line2D([],[],c=BLUE,marker='o',ls='none',label='Primitive junction'),
        Line2D([],[],c=RED,ls='--',label='Motion limit')],
        loc='lower center',bbox_to_anchor=(.5,.025),ncol=2,frameon=False,fontsize=15.5)
    save(fig,'fig_4_11_kinodynamic_result')
    return {'max_speed':max(speed),'max_acceleration':max(effort),
            'time':result['duration'],'cost':result['cost'],'expanded':result['expanded']}

if __name__=='__main__':
    configure_fonts()
    report=json.loads((ROOT/'results/sampling_dynamics.json').read_text(encoding='utf-8'))
    sampling(report);local=rewiring();primitive_fan(report);limits=kino_result(report)
    (ROOT/'results/new_figure_metrics.json').write_text(json.dumps({'rewiring':local,'kino':limits},indent=2))
    print(json.dumps({'rewiring':local,'kino':limits},indent=2))
