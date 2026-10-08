"""Reproduce the concise chapter's English teaching figures.

Run: python visual_chapter_figures.py
Core planning implementations are in planning.py. Matplotlib is needed here.
"""
from pathlib import Path
import heapq
import itertools
import json
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.lines import Line2D
from matplotlib.transforms import Bbox
import numpy as np
from planning import Grid, static_scene, dynamic_scene, search, DStarLite
from planning import shortcut_path, length, validate_path
from figure_support import configure_fonts, architecture, source_demonstrations
configure_fonts()

ROOT = Path(__file__).resolve().parent
FIG = ROOT/'visual_figures'
RESULTS = ROOT/'results'
FIG.mkdir(exist_ok=True)
RESULTS.mkdir(exist_ok=True)
BLUE = '#1261A0'
ORANGE = '#C95621'
GREEN = '#19734A'
GRAY = '#D8DDE2'
DARK = '#222B34'
plt.rcParams.update({'font.family':'Times New Roman', 'font.size':17,
                     'axes.titlesize':18, 'axes.labelsize':16,
                     'savefig.dpi':260, 'svg.fonttype':'none'})

def save(fig, name):
    fig.canvas.draw()
    bound=fig.get_tightbbox(fig.canvas.get_renderer())
    fixed=Bbox.from_extents(min(0,bound.x0),min(0,bound.y0),max(fig.get_figwidth(),bound.x1),max(fig.get_figheight(),bound.y1))
    fig.savefig(FIG/(name+'.png'), facecolor='white', bbox_inches=fixed, pad_inches=.08)
    fig.savefig(FIG/(name+'.svg'), facecolor='white', bbox_inches=fixed, pad_inches=.08)
    plt.close(fig)

def arrow(ax, a, b, color=DARK, lw=1.5, style='->'):
    ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle=style, color=color, lw=lw))

def history():
    fig, ax = plt.subplots(figsize=(9.2,9.2))
    fig.subplots_adjust(left=.015,right=.985,top=.98,bottom=.02)
    ax.set(xlim=(0,3), ylim=(0,11));ax.axis('off')
    columns = [
        ('SEARCH AND SAMPLING', BLUE, [
          ('1959 / 1968','Dijkstra / A*','Shortest graph paths'),
          ('1996 / 1998','PRM / RRT','Sample continuous space'),
          ('2002 / 2010–2011','D* Lite / Theta* / JPS','Repair / any angle / prune'),
          ('2011','RRT*','Improve as samples grow')]),
        ('TRAJECTORY OPTIMIZATION', GREEN, [
          ('2011','Minimum snap','Smooth polynomial motion'),
          ('2019','Fast-Planner','Search + B-splines + ESDF'),
          ('2021 / 2022','MINCO / GCOPTER','Optimize waypoints + time'),
          ('2022','EGO-Planner-v2','Online motion in clutter')]),
        ('LEARNING AND SEMANTICS', ORANGE, [
          ('2011 / 2017','DAgger / PPO','Imitation / reward learning'),
          ('2021 / 2023','Agile flight / Swift','Learned navigation / racing'),
          ('2024','OpenVLA','Vision + language → action'),
          ('2025 / 2026','OpenFly','Aerial language navigation')])]
    for col,(title,color,boxes) in enumerate(columns):
        x=col+.5
        title=title.replace(' AND ','\nAND ').replace('TRAJECTORY ','TRAJECTORY\n')
        ax.text(x,10.57,title,ha='center',va='center',fontsize=16.5,fontweight='bold',color=color)
        for j,(date,name,purpose) in enumerate(boxes):
            top=9.94-j*2.03
            ax.add_patch(FancyBboxPatch((col+.035,top-1.59),.93,1.59,
                 boxstyle='round,pad=0.01,rounding_size=0.04',facecolor='#F8FAFC',edgecolor=color,lw=1.4))
            ax.text(x,top-.26,date,ha='center',va='center',fontsize=14.2,color=color)
            ax.text(x,top-.72,name.replace(' / ',' /\n') if len(name)>22 else name,
                    ha='center',va='center',fontsize=15.8,fontweight='bold',linespacing=1.15)
            words=purpose.split(' ');mid=len(words)//2
            purpose=' '.join(words[:mid])+'\n'+' '.join(words[mid:])
            ax.text(x,top-1.25,purpose,ha='center',va='center',fontsize=15.1,linespacing=1.1)
            if j<3:arrow(ax,(x,top-1.64),(x,top-1.94),color)
        ax.text(x,1.51,['§4.2\nFind a route','§4.3–4.4\nFly and coordinate','§4.5\nLearn and interpret'][col],
                ha='center',va='center',fontsize=15.4,fontweight='bold',color=color)
        arrow(ax,(x,1.27),(1.5,.83),color)
    ax.add_patch(FancyBboxPatch((.16,.17),2.68,.60,boxstyle='round,pad=.02',facecolor='#EDF2F6',edgecolor=DARK))
    ax.text(1.5,.47,'HYBRID SYSTEM: propose → check → execute',ha='center',va='center',fontsize=17,fontweight='bold')
    ax.text(1.5,-.06,'Legend: blue = search/sampling; green = trajectories; orange = learning/semantics.\nRead columns downward; selected milestones, not a common time scale.',ha='center',va='top',fontsize=14.5)
    save(fig,'fig_4_2_development')

def small_map(ax, grid, start, goal, labels=('S','T')):
    ax.set(xlim=(-.5,grid.width-.5),ylim=(-.5,grid.height-.5),aspect='equal')
    for x,y in grid.blocked:ax.add_patch(Rectangle((x-.5,y-.5),1,1,fc=DARK))
    ax.set_xticks(np.arange(-.5,grid.width,1),minor=True);ax.set_yticks(np.arange(-.5,grid.height,1),minor=True)
    ax.grid(which='minor',color='#CBD1D8',lw=.5)
    ax.set_xticks([]);ax.set_yticks([]);ax.tick_params(which='minor',length=0)
    for p,label,c in [(start,labels[0],BLUE),(goal,labels[1],ORANGE)]:
        ax.scatter(*p,s=500,c=c,zorder=6,edgecolors='white',linewidths=1.0)
        ax.text(*p,label,ha='center',va='center',color='white',fontsize=14,fontweight='normal',fontstyle='italic' if label in ['u','v'] else 'normal',zorder=7)

def environment():
    # Original rectangle is [3.5,4.5] x [1.5,4.5]; inflate by 0.4 m.
    blocked={(x,y) for x in range(9) for y in range(7)
             if x+.5>=3.1 and x-.5<=4.9 and y+.5>=1.1 and y-.5<=4.9}
    grid=Grid(9,7,blocked);s=(1,3);t=(7,3);r=search(grid,s,t)
    validate_path(grid,r.path,s,t)
    fig,axs=plt.subplots(2,2,figsize=(9.2,7.7))
    fig.subplots_adjust(left=.025,right=.975,top=.89,bottom=.12,hspace=.40,wspace=.16)
    fig.suptitle('From obstacle geometry to a searchable route',fontweight='bold',y=.995,fontsize=20)
    ax=axs[0,0];small_map(ax,Grid(9,7),s,t);ax.grid(False,which='minor')
    ax.add_patch(Rectangle((3.1,1.1),1.8,3.8,fc=GRAY,ec='#888888',lw=1.1))
    ax.add_patch(Rectangle((3.5,1.5),1,3,fc=DARK))
    ax.text(4,5.6,'0.4 m clearance',ha='center',fontsize=15)
    arrow(ax,(4,5.2),(4,4.7))
    ax.set_title('1  Inflate the obstacle',loc='left',fontsize=17,pad=9)
    ax.text(.5,-.10,'Gray = vehicle radius + margin',transform=ax.transAxes,ha='center',fontsize=15)
    ax=axs[0,1];small_map(ax,grid,s,t)
    for x in range(9):
        for y in range(7):
            if (x,y) not in blocked and (x,y) not in [s,t]:ax.plot(x,y,'.',color='#718096',ms=4)
    ax.set_title('2  Mark occupied cells',loc='left',fontsize=17,pad=9)
    ax.text(.5,-.10,f'1 m cells: 63 cells → {63-len(blocked)} free nodes',transform=ax.transAxes,ha='center',fontsize=15)
    ax=axs[1,0];corner=Grid(3,3,{(1,0)})
    small_map(ax,corner,(0,0),(1,1),labels=('u','v'))
    ax.plot([0,1],[0,1],'--',c=ORANGE,lw=3,zorder=4)
    ax.text(.50,.53,'×',c=ORANGE,fontsize=40,ha='center',va='center',zorder=8)
    ax.plot([0,0,1],[0,1,1],c=BLUE,lw=3,zorder=3)
    ax.text(1.7,2.1,'Use two\nlegal edges',ha='center',va='center',fontsize=15,color=BLUE)
    ax.set_title('3  Check whole connections',loc='left',fontsize=17,pad=9)
    ax.text(.5,-.10,'Free endpoints do not make a diagonal safe',transform=ax.transAxes,ha='center',fontsize=14.6)
    ax=axs[1,1];small_map(ax,grid,s,t)
    ax.plot(*zip(*r.path),c=BLUE,lw=3,zorder=4)
    ax.set_title('4  Search the legal graph',loc='left',fontsize=17,pad=9)
    ax.text(.5,-.10,f'A*: collision-free route, length {r.cost:.2f} m',transform=ax.transAxes,ha='center',fontsize=15)
    fig.text(.5,.012,'S = start   T = goal   Black = occupied   Blue = accepted connection / route',ha='center',fontsize=14.3)
    save(fig,'fig_4_4_discretization')
    return {'shape':[9,7],'blocked':sorted(blocked),'path':r.path,'cost':r.cost}

def trace_search(use_h):
    s=(0,0);t=(6,0);count=itertools.count()
    h=lambda p:abs(p[0]-6)+abs(p[1]) if use_h else 0
    q=[(h(s),h(s),next(count),s)];g={s:0};order=[]
    while q:
        f,_,_,u=heapq.heappop(q)
        if u in order:continue
        order.append(u)
        if u==t:break
        for dx,dy in [(1,0),(0,1),(-1,0),(0,-1)]:
            v=(u[0]+dx,u[1]+dy)
            if 0<=v[0]<=6 and 0<=v[1]<=1 and g[u]+1<g.get(v,math.inf):
                g[v]=g[u]+1;heapq.heappush(q,(g[v]+h(v),h(v),next(count),v))
    return order,g

def selection():
    fig=plt.figure(figsize=(9.2,7.10));gs=fig.add_gridspec(3,1,height_ratios=[1,1,.70],hspace=.62,left=.045,right=.965,top=.80,bottom=.09)
    fig.suptitle('Why A* processes fewer nodes',fontsize=21,fontweight='bold',y=.995)
    fig.text(.5,.932,'4-neighbor grid, unit edges. $g$: cost from S; $h$: Manhattan distance to T.',ha='center',fontsize=14.1)
    fig.text(.5,.887,'Stop when T is removed. Ties: smaller $h$, then east-first insertion.',ha='center',fontsize=13.8)
    allorders=[]
    for i,use_h in enumerate([False,True]):
        ax=fig.add_subplot(gs[i]);order,g=trace_search(use_h);allorders.append(order)
        for x in range(7):
            for y in range(2):
                p=(x,y);chosen=p in order;known=p in g
                fc='#D8E8F4' if chosen else ('#FFF0DE' if known else 'white')
                ax.add_patch(Rectangle((x-.5,y-.5),1,1,facecolor=fc,edgecolor='#9EABB6',lw=1))
                name='S' if p==(0,0) else ('T' if p==(6,0) else ('A' if p==(1,0) else ('B' if p==(0,1) else '')))
                if chosen:label=f'{name+": " if name else ""}#{order.index(p)+1}\n$g={g[p]}$'
                elif known:label=f'{name+": " if name else ""}waiting\n$f={g[p]+abs(x-6)+y}$' if use_h else 'waiting'
                else:label='unseen'
                ax.text(x,y,label,ha='center',va='center',fontsize=15.2,linespacing=1.35)
        ax.set(xlim=(-.5,6.5),ylim=(-.5,1.5),aspect='auto');ax.axis('off')
        title=f'{"A*: choose minimum $g+h$" if use_h else "Dijkstra: choose minimum $g$"}   |   {len(order)} nodes processed'
        ax.set_title(title,loc='left',fontsize=17.8,fontweight='bold',pad=12)
    ax=fig.add_subplot(gs[2]);ax.axis('off')
    ax.text(.02,.98,'First choice after S',transform=ax.transAxes,fontweight='bold',fontsize=17)
    ax.text(.02,.64,'Dijkstra: $g(\\mathrm{A})=g(\\mathrm{B})=1$\nBoth have equal priority.',transform=ax.transAxes,va='top',fontsize=16.5,linespacing=1.35)
    ax.text(.55,.64,'A*: $f(\\mathrm{A})=1+5=6$\n       $f(\\mathrm{B})=1+7=8$ → choose A',transform=ax.transAxes,va='top',fontsize=16.5,linespacing=1.35,color=BLUE)
    fig.text(.5,.017,'# = queue removal order (including T)   Blue = processed   Orange = discovered, still waiting',ha='center',fontsize=13.4)
    save(fig,'fig_4_5_astar_selection')
    return {'dijkstra_order':allorders[0],'astar_order':allorders[1],'tie_break':'smaller h, then east-first insertion order'}

def map_ax(ax,grid,s,t,visited=None):
    data=np.zeros((grid.height,grid.width))
    for x,y in visited or []:data[y,x]=.30
    for x,y in grid.blocked:data[y,x]=1
    ax.imshow(data,origin='lower',cmap='Greys',vmin=0,vmax=1,interpolation='nearest')
    ax.set(xlim=(-.5,grid.width-.5),ylim=(-.5,grid.height-.5))
    ax.set_xticks([]);ax.set_yticks([])
    for p,l,c,offset in [(s,'S',BLUE,(-.3,-2.7)),(t,'T',ORANGE,(0,2.6))]:
        ax.scatter(*p,s=65,color=c,edgecolors='white',zorder=6)
        ax.text(p[0]+offset[0],p[1]+offset[1],l,ha='center',va='center',fontweight='bold',fontsize=15,color=c,zorder=8,
                bbox=dict(facecolor='white',edgecolor='none',pad=.25,alpha=.9))

def route(ax,path,color=BLUE,style='-'):
    ax.plot(*zip(*path),lw=2.3,color=color,ls=style,zorder=5)

def static_figures():
    rows=[]
    for name,w,theta in [('Dijkstra',0,False),('A*',1,False),('Weighted A*',2,False),('Theta*',1,True)]:
        grid,s,t=static_scene();r=search(grid,s,t,w,theta);validate_path(grid,r.path,s,t)
        rows.append(dict(algorithm=name,cost=r.cost,processed=len(r.expanded),path=r.path,visited=r.expanded))
    grid,s,t=static_scene();sp=shortcut_path(grid,rows[1]['path']);validate_path(grid,sp,s,t)
    fig,axs=plt.subplots(2,2,figsize=(9.2,7.5));fig.subplots_adjust(left=.025,right=.98,top=.86,bottom=.13,hspace=.43,wspace=.12)
    comparisons=[('Dijkstra',rows[0],'1393 processed; graph optimum'),
                 ('A*',rows[1],'234 processed; same optimum'),
                 ('Weighted A* ($w=2$)',rows[2],'51 processed; +8.2% length'),
                 ('Theta*',rows[3],'286 processed; any-angle route')]
    for ax,(name,row,note) in zip(axs.flat,comparisons):
        map_ax(ax,grid,s,t,row['visited']);route(ax,row['path'])
        ax.set_title(f'{name} | length {row["cost"]:.3f}',fontsize=16.7,pad=8)
        ax.text(.5,-.1,note,transform=ax.transAxes,ha='center',fontsize=14.6)
    fig.suptitle('Two different goals: search less or shorten the route',fontsize=20,fontweight='bold',y=.99)
    fig.text(.5,.045,'48 × 32 cells • S=(3,5) • T=(44,26) • 8 neighbors • no corner cutting',ha='center',fontsize=14)
    fig.text(.5,.012,'Black = obstacle   Gray = processed   Blue = route   Length in cell units',ha='center',fontsize=14.4)
    save(fig,'fig_4_6_method_comparison')
    return rows+[dict(algorithm='A* + shortcut',cost=length(sp),path=sp)]

def dynamic_figure():
    grid,s,t=dynamic_scene();p=DStarLite(grid,s,t);p.compute();p.move_start((18,8));before=search(grid,(18,8),t)
    prior=Grid(grid.width,grid.height,grid.blocked);events=[]
    for label,changes in [('Close passage',[((24,y),True) for y in range(6,10)]),
                          ('Reopen passage',[((24,y),False) for y in range(6,10)]),
                          ('Remote obstacle',[((4,28),True)])]:
        p.update_cells(changes);dr=p.compute();ar=search(grid,(18,8),t)
        assert abs(dr.cost-ar.cost)<1e-8
        validate_path(grid,dr.path,(18,8),t)
        events.append(dict(event=label,dstar=len(dr.expanded),astar=len(ar.expanded),cost=dr.cost,path=dr.path,blocked=sorted(grid.blocked)))
    fig=plt.figure(figsize=(9.2,7.5));gs=fig.add_gridspec(2,2,height_ratios=[1,.70],left=.08,right=.98,top=.85,bottom=.12,hspace=.53,wspace=.14)
    ax=fig.add_subplot(gs[0,0]);map_ax(ax,prior,(18,8),t);route(ax,before.path)
    ax.set_title(f'Before: lower passage open\nLength {before.cost:.3f}',fontsize=17,pad=9)
    closed=Grid(48,32,events[0]['blocked']);ax=fig.add_subplot(gs[0,1]);map_ax(ax,closed,(18,8),t);route(ax,events[0]['path'])
    for y in range(6,10):ax.add_patch(Rectangle((23.5,y-.5),1,1,fc=ORANGE,zorder=4))
    ax.annotate('New blockage',xy=(24,8),xytext=(32,1),color=ORANGE,ha='center',fontsize=13.5,arrowprops=dict(arrowstyle='->',color=ORANGE,lw=1.3))
    ax.set_title(f'After: detour via upper passage\nLength {events[0]["cost"]:.3f}',fontsize=17,pad=9)
    ax=fig.add_subplot(gs[1,:]);x=np.arange(3);w=.34
    for dx,k,label,color in [(-w/2,'dstar','D* Lite repair',BLUE),(w/2,'astar','Fresh A*',GRAY)]:
        bars=ax.bar(x+dx,[e[k] for e in events],w,label=label,color=color,edgecolor=DARK,lw=.5)
        ax.bar_label(bars,padding=3,fontsize=15)
    ax.set_xticks(x,['Close passage','Reopen passage','Remote change']);ax.tick_params(axis='x',labelsize=15)
    ax.set(ylabel='Nodes processed',ylim=(0,810));ax.set_yticks([0,300,600]);ax.tick_params(axis='y',labelsize=14)
    ax.spines[['top','right']].set_visible(False);ax.legend(loc='upper right',fontsize=14.5,frameon=False,ncol=2)
    fig.suptitle('Reuse a search, then repair what the update invalidates',fontsize=20,fontweight='bold',y=.995)
    fig.text(.5,.056,'S=(18,8), T=(44,8) for every update; D* Lite retains its previous state.',ha='center',fontsize=14.7)
    fig.text(.5,.021,'Repair can be expensive after a critical closure; a remote change may need no queue work.',ha='center',fontsize=13.8)
    save(fig,'fig_4_7_replanning')
    return events

if __name__=='__main__':
    # Retain the accepted architecture's original implicit arrow-head scale.
    with plt.rc_context({'font.size':10}):
        architecture()
    frames = ROOT / 'source_frames'
    have_frames = all((frames / name).is_file() for name in
                      ('ego_frame_160s.png', 'agile_frame_37.png'))
    if have_frames:
        source_demonstrations()
    else:
        print('Figure 4.3 uses external media; see docs/external_projects.md.')
    history()
    report={'environment':environment(),'selection':selection(),
            'static':static_figures(),'dynamic':dynamic_figure()}
    (RESULTS/'visual_revision.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    from algorithm_exposition_figures import main as revised_mechanisms
    revised_mechanisms()
    print('Created the available Times New Roman figures; all plotted routes passed collision checks.')
