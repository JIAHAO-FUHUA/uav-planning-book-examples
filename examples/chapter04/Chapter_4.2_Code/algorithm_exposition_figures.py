"""Editable mechanism figures for Sections 4.2.3 and 4.2.4.

Run after visual_chapter_figures.py to obtain the revised publication figures.
Core simulation algorithms and recorded benchmark inputs remain unchanged.
"""
from pathlib import Path
import json, math, heapq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.lines import Line2D
from figure_support import configure_fonts, save

configure_fonts()
ROOT=Path(__file__).resolve().parent
BLUE='#111111'; ORANGE='#333333'; GREEN='#555555'; GRAY='#888888'; DARK='#202020'
plt.rcParams.update({'font.size':17,'axes.titlesize':18,'svg.fonttype':'none'})

def arrow(ax,a,b,color=DARK,ls='-',lw=2.2):
    ax.annotate('',xy=b,xytext=a,arrowprops={'arrowstyle':'->','color':color,'lw':lw,'linestyle':ls,'shrinkA':24,'shrinkB':24,'mutation_scale':18})

def improvements():
    fig=plt.figure(figsize=(10.5,6.9))
    fig.suptitle('Selection and connection rules',fontsize=23,fontweight='bold',y=.985)
    ax=fig.add_axes([.04,.65,.92,.24]);ax.set(xlim=(0,10),ylim=(0,3));ax.axis('off')
    ax.text(0,2.6,'(a) Weighted A*: change priority, keep relaxation',fontweight='bold',fontsize=19)
    labels=[('Candidate a','g = 2,  h = 4',1.4),('Candidate b','g = 5,  h = 2',4.7)]
    for name,cost,x in labels:
        ax.text(x,1.9,name,ha='center',fontsize=19)
        ax.text(x,1.22,cost,ha='center')
    ax.text(7.5,1.85,'w = 1:  f(a) = 6 < 7  →  choose a',ha='center',fontsize=16.5,color=BLUE)
    ax.text(7.5,.95,'w = 2:  f(a) = 10 > 9  →  choose b',ha='center',fontsize=16.5,color=ORANGE)
    gs=fig.add_gridspec(1,2,left=.07,right=.96,bottom=.18,top=.58,wspace=.30)
    for i,title in enumerate(['(b) Shortcut after search','(c) Theta*: relax through the parent']):
        ax=fig.add_subplot(gs[0,i]);ax.set(xlim=(-.4,2.6),ylim=(-.5,2.8),aspect='equal');ax.axis('off')
        a=(0,0);u=(0,2);v=(2,2)
        ax.set_title(title,pad=12,fontweight='bold',fontsize=17.5)
        ax.plot([0,0,2],[0,2,2],color=GRAY,lw=3,ls='--')
        ax.plot([0,2],[0,2],color=BLUE,lw=3)
        for pos,label,offset in [(a,'a',(-.16,-.23)),(u,'u',(-.17,.20)),(v,'v',(.13,.13))]:
            ax.scatter(*pos,s=140,c=DARK,zorder=4)
            ax.text(pos[0]+offset[0],pos[1]+offset[1],label,fontsize=22,fontstyle='italic',ha='center')
        ax.text(-.18,1,'2',ha='right',color=GRAY)
        ax.text(1,2.18,'2',ha='center',color=GRAY)
        ax.text(1.30,.90,'2.828',ha='center',color=BLUE,rotation=45)
        if i==0:
            ax.text(1,-.47,'Free a–v: delete u\nLength 4 → 2.828',ha='center',va='top',fontsize=17,color=BLUE)
        else:
            ax.text(1,-.47,'a = parent(u), g(a) = 1\nVia u: 5; via a: 3.828 → parent(v) = a',ha='center',va='top',fontsize=16.3,color=BLUE)
    fig.legend(handles=[Line2D([0],[0],color=GRAY,lw=2,ls='--',label='Existing / ordinary connection'),Line2D([0],[0],color=BLUE,lw=3,label='Checked direct connection')],loc='lower center',bbox_to_anchor=(.5,.015),ncol=2,frameon=False,fontsize=16)
    save(fig,'fig_4_6_method_comparison')

def shortest_to_goal(blocked=False):
    # Independently solve the small directed graph, backward from T.
    edges=[('S','u',1),('u','T',math.inf if blocked else 2),('S','v',2),('v','T',4)]
    incoming={x:[] for x in ['S','u','v','T']}
    for a,b,c in edges:incoming[b].append((a,c))
    d={x:math.inf for x in incoming};d['T']=0;q=[(0,'T')]
    while q:
        g,u=heapq.heappop(q)
        if g!=d[u]:continue
        for v,c in incoming[u]:
            if g+c<d[v]:d[v]=g+c;heapq.heappush(q,(d[v],v))
    return d

def repair():
    before=shortest_to_goal(False);after=shortest_to_goal(True)
    assert before=={'S':3,'u':2,'v':4,'T':0}
    assert after=={'S':6,'u':math.inf,'v':4,'T':0}
    fig,axs=plt.subplots(1,2,figsize=(10.5,6.6))
    fig.subplots_adjust(left=.03,right=.97,bottom=.37,top=.83,wspace=.10)
    fig.suptitle('D* Lite: invalidate old costs, then repair',fontsize=22,fontweight='bold',y=.99)
    coords={'S':(0,0),'u':(2,1.5),'v':(2,-1.5),'T':(4,0)}
    edges=[('S','u',1),('u','T',2),('S','v',2),('v','T',4)]
    for i,(ax,d,title) in enumerate(zip(axs,[before,after],['(a) Before: route cost 3','(b) Close u–T: route cost 6'])):
        ax.set(xlim=(-.6,4.6),ylim=(-2.0,2.0),aspect='equal');ax.axis('off');ax.set_title(title,fontweight='bold',fontsize=18,pad=9)
        active={('S','u'),('u','T')} if i==0 else {('S','v'),('v','T')}
        for a,b,c in edges:
            pa,pb=coords[a],coords[b]
            blocked=i==1 and (a,b)==('u','T')
            color=ORANGE if blocked else (BLUE if (a,b) in active else GRAY)
            arrow(ax,pa,pb,color,ls='--' if blocked else ('-' if (a,b) in active else ':'),lw=3 if (a,b) in active else 1.8)
            x,y=((pa[0]+pb[0])/2,(pa[1]+pb[1])/2)
            if blocked:ax.plot(x,y,'x',color='black',ms=9,mew=2,zorder=6)
            ax.text(x,y+(.18 if y>0 else -.30),'∞' if blocked else str(c),ha='center',color=color,fontsize=21)
        for n,p in coords.items():
            ax.add_patch(Circle(p,.29,fc='white',ec=DARK,lw=1.7,zorder=4))
            ax.text(*p,n,ha='center',va='center',fontsize=22,fontstyle='italic' if n in ['u','v'] else 'normal',zorder=5)
            value='∞' if not math.isfinite(d[n]) else str(d[n])
            ax.text(p[0],p[1]+(.44 if n in ['u','S','T'] else -.45),'g = '+value,ha='center',va='center',fontsize=17)
    fig.text(.5,.33,'Close: u has g = 2 < rhs = ∞  →  invalidate; S changes from 3 to 6.\nReopen: u has g = ∞ > rhs = 2  →  accept 2; S changes from 6 to 3.',ha='center',va='top',fontsize=17,linespacing=1.5)
    for x,text,color in [(.18,'g = rhs\nConsistent: no queue entry',DARK),(.5,'g > rhs\nCheaper: set g ← rhs',BLUE),(.82,'g < rhs\nObsolete: set g ← ∞, recompute',ORANGE)]:
        fig.text(x,.16,text,ha='center',va='center',fontsize=16.5,color=color,linespacing=1.4)
    fig.text(.5,.035,'Arrows = directed edges; labels = edge costs; g = stored cost to T.\nThick solid = route; thin dotted = unused; dashed with x = blocked.',ha='center',va='bottom',fontsize=15.5)
    save(fig,'fig_4_7_replanning')
    return {'before':before,'closed':{k:v if math.isfinite(v) else 'infinity' for k,v in after.items()},'reopened':before}

def main():
    improvements();values=repair()
    (ROOT/'results'/'algorithm_illustrations.json').write_text(json.dumps({'weighted_candidates':{'a':{'g':2,'h':4},'b':{'g':5,'h':2}},'shortcut_length':math.sqrt(8),'theta_candidate_cost':1+math.sqrt(8),'directed_repair':values},indent=2),encoding='utf-8')
    print('Created revised Figures 4.6–4.7; illustrative costs verified independently.')

if __name__=='__main__':main()
