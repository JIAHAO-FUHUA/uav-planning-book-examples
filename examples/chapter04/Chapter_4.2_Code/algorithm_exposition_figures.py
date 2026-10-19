"""Editable grayscale mechanism figures; toy graph costs are checked independently."""
from pathlib import Path
import heapq,json,math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle
from matplotlib.lines import Line2D
from figure_support import configure_fonts,save
configure_fonts()
ROOT=Path(__file__).resolve().parent
plt.rcParams.update({'font.size':18,'svg.fonttype':'none'})
BLACK='#111111';GRAY='#888888'

def arrow(ax,a,b,ls='-',lw=2.3,color=BLACK,shrink=12):
    ax.annotate('',xy=b,xytext=a,arrowprops={'arrowstyle':'->','color':color,'lw':lw,'linestyle':ls,'shrinkA':shrink,'shrinkB':shrink,'mutation_scale':15})

def grid(ax,blocked=()):
    ax.set(xlim=(-.6,3.6),ylim=(-.6,2.6),aspect='equal');ax.axis('off')
    for x in range(4):
        for y in range(3):ax.add_patch(Rectangle((x-.5,y-.5),1,1,fc='#444444' if (x,y) in blocked else 'white',ec='#BBBBBB',lw=.65))

def local_cost(start,goal,blocked=()):
    blocked=set(blocked);q=[(0.,start)];d={start:0.}
    while q:
        cost,u=heapq.heappop(q)
        if cost!=d[u]:continue
        if u==goal:return cost
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                v=(u[0]+dx,u[1]+dy)
                if (dx,dy)==(0,0) or not(0<=v[0]<=3 and 0<=v[1]<=2) or v in blocked:continue
                if dx and dy and ((u[0]+dx,u[1]) in blocked or (u[0],u[1]+dy) in blocked):continue
                candidate=cost+math.hypot(dx,dy)
                if candidate<d.get(v,math.inf):d[v]=candidate;heapq.heappush(q,(candidate,v))
    return math.inf

def improvements():
    fig=plt.figure(figsize=(10.5,7.65));fig.suptitle('Four ways to change a grid search',fontsize=23,weight='bold',y=.988)
    ax=fig.add_axes([.035,.64,.43,.24]);ax.axis('off');ax.set_title('(a) Weighted A*: change priority',fontsize=19,weight='bold',pad=17)
    for y,line in [(.83,'a: g = 2, h = 4     b: g = 5, h = 2'),(.56,'w = 1:  f(a) = 6 < 7  →  choose a'),(.29,'w = 2:  f(a) = 10 > 9  →  choose b')]:ax.text(.5,y,line,ha='center',fontsize=17)
    ax.text(.5,-.02,'Relaxation is unchanged.',ha='center',fontsize=17)
    fig.text(.755,.913,'(b) JPS: remove symmetric work',ha='center',fontsize=19,weight='bold')
    left=fig.add_axes([.52,.657,.215,.21]);grid(left)
    for route,style in [([(0,0),(1,0),(2,0),(3,1)],'-'), ([(0,0),(1,1),(2,1),(3,1)],'--')]:left.plot(*zip(*route),color=BLACK,lw=2.5,ls=style)
    left.plot([0,3],[0,1],'o',color='black',ms=5);left.text(.05,.22,'A',ha='center',fontsize=17);left.text(3,.12,'B',ha='center',fontsize=17)
    left.set_title('Equal-cost move orders',fontsize=15,pad=4)
    right=fig.add_axes([.765,.657,.215,.21]);grid(right,{(1,2)})
    arrow(right,(1,1),(2,1),shrink=5);arrow(right,(2,1),(2,2),shrink=5);arrow(right,(2,1),(3,1),ls=':',color=GRAY,shrink=5)
    right.plot([1,2],[1,2],ls='--',c=GRAY,lw=1.5);right.plot(1.48,1.48,'x',c='black',ms=8,mew=1.5)
    for (x,y),label,marker in [((1,1),'p','o'),((2,1),'x','s'),((2,2),'n','o')]:right.plot(x,y,marker,ms=7,c='black');right.text(x+.12,y-.36,label,fontsize=17)
    right.set_title('Blocked bypass: retain n',fontsize=15,pad=4)
    fig.text(.627,.625,'Both costs = 2 + √2',ha='center',fontsize=16);fig.text(.875,.625,'x is a jump point.',ha='center',fontsize=16)
    fig.text(.752,.585,'8-neighbor grid; no corner cutting.\nScan cells; queue jump points.',ha='center',va='top',fontsize=16,linespacing=1.25)
    for i,title in enumerate(['(c) Shortcut after search','(d) Theta*: relax through the parent']):
        ax=fig.add_axes([.07+i*.48,.17,.40,.27]);ax.set(xlim=(-.6,2.6),ylim=(-.5,2.8),aspect='equal');ax.axis('off');ax.set_title(title,fontsize=18,weight='bold',pad=13)
        ax.plot([0,0,2],[0,2,2],color=GRAY,lw=2.8,ls='--');ax.plot([0,2],[0,2],color=BLACK,lw=2.8)
        for (x,y),label,offset in [((0,0),'a',(-.16,-.23)),((0,2),'u',(-.17,.20)),((2,2),'v',(.13,.13))]:ax.plot(x,y,'o',ms=8,c='black');ax.text(x+offset[0],y+offset[1],label,fontsize=21,fontstyle='italic',ha='center')
        ax.text(-.2,1,'2',ha='right',fontsize=18);ax.text(1,2.18,'2',ha='center',fontsize=18);ax.text(1.32,.86,'2.828',rotation=45,ha='center',fontsize=17,bbox={'facecolor':'white','edgecolor':'none','pad':.3})
        note='Free a–v: delete u\nLength 4 → 2.828' if i==0 else 'a = parent(u), g(a) = 1\nVia u: 5; via a: 3.828 → parent(v) = a'
        ax.text(1,-.49,note,ha='center',va='top',fontsize=16.5,linespacing=1.3)
    fig.legend(handles=[Line2D([],[],c=BLACK,lw=2.5,label='Selected / direct connection'),Line2D([],[],c=GRAY,lw=2,ls='--',label='Alternative / previous connection'),Line2D([],[],c=BLACK,marker='s',ls='',label='Jump point')],loc='lower center',bbox_to_anchor=(.5,.012),ncol=3,frameon=False,fontsize=15)
    save(fig,'fig_4_6_method_comparison')
    assert abs(local_cost((0,0),(3,1))-(2+math.sqrt(2)))<1e-12
    assert abs(local_cost((1,1),(2,2),{(1,2)})-2)<1e-12
    assert local_cost((1,1),(2,2),{(1,2),(2,1)})>2
    return {'equal_move_order_cost':2+math.sqrt(2),'forced_neighbor_cost_via_x':2,'corner_cutting':False}

def shortest_to_goal(blocked=False):
    edges=[('S','u',1),('u','T',math.inf if blocked else 2),('S','v',2),('v','T',4)];incoming={x:[] for x in ['S','u','v','T']}
    for a,b,c in edges:incoming[b].append((a,c))
    d={x:math.inf for x in incoming};d['T']=0;q=[(0,'T')]
    while q:
        cost,u=heapq.heappop(q)
        if cost!=d[u]:continue
        for v,c in incoming[u]:
            if cost+c<d[v]:d[v]=cost+c;heapq.heappush(q,(d[v],v))
    return d

def repair():
    before=shortest_to_goal();closed=shortest_to_goal(True)
    assert before=={'S':3,'u':2,'v':4,'T':0} and closed=={'S':6,'u':math.inf,'v':4,'T':0}
    g=dict(before);rhs=dict(before);rhs['u']=math.inf;stages=[{'g':dict(g),'rhs':dict(rhs)}]
    assert g['u']<rhs['u'];g['u']=math.inf;rhs['S']=min(1+g['u'],2+g['v']);stages.append({'g':dict(g),'rhs':dict(rhs)})
    assert g['S']<rhs['S'];g['S']=math.inf;assert g['S']>rhs['S'];g['S']=rhs['S'];stages.append({'g':dict(g),'rhs':dict(rhs)});assert g==closed
    fig,axs=plt.subplots(1,3,figsize=(10.5,5.4));fig.subplots_adjust(left=.025,right=.975,bottom=.45,top=.84,wspace=.16)
    fig.suptitle('D* Lite: follow a cost change back to the start',fontsize=22,weight='bold',y=.985)
    coords={'S':(0,0),'u':(2,1.5),'v':(2,-1.5),'T':(4,0)}
    for i,(ax,stage,title) in enumerate(zip(axs,stages,['1 Detect a stale value','2 Invalidate u; update S','3 Accept the new route'])):
        ax.set(xlim=(-.6,4.6),ylim=(-2,2.15),aspect='equal');ax.axis('off');ax.set_title(title,weight='bold',fontsize=16,pad=13)
        for a,b,c in [('S','u',1),('u','T',math.inf),('S','v',2),('v','T',4)]:
            blocked=(a,b)==('u','T');selected=i==2 and (a,b) in {('S','v'),('v','T')};arrow(ax,coords[a],coords[b],ls='--' if blocked else ('-' if selected else ':'),lw=2.8 if selected else 1.6,color=BLACK if blocked or selected else GRAY,shrink=13)
            x=(coords[a][0]+coords[b][0])/2;y=(coords[a][1]+coords[b][1])/2
            if blocked:ax.plot(x,y,'x',c='black',ms=8,mew=1.7)
            ax.text(x,y+(.25 if y>0 else -.31),'∞' if blocked else str(c),ha='center',fontsize=19)
        for name,(x,y) in coords.items():ax.add_patch(Circle((x,y),.29,fc='white',ec=BLACK,lw=1.4,zorder=4));ax.text(x,y,name,ha='center',va='center',fontsize=21,zorder=5)
        def f(v):return '∞' if not math.isfinite(v) else str(v)
        ax.text(.5,-.09,'u: g / rhs = '+f(stage['g']['u'])+' / '+f(stage['rhs']['u'])+'\nS: g / rhs = '+f(stage['g']['S'])+' / '+f(stage['rhs']['S']),transform=ax.transAxes,ha='center',va='top',fontsize=17,linespacing=1.3)
    fig.text(.5,.235,'At S: 3 → ∞ → 6. Reopening u–T reverses the repair: u = 2, then S = 3.',ha='center',fontsize=17)
    fig.text(.5,.157,'g = rhs: keep     g > rhs: accept rhs     g < rhs: invalidate, then recompute',ha='center',fontsize=16.5)
    fig.text(.5,.085,'Unchanged values: g(T) = 0 and g(v) = 4. A predecessor has an edge into the updated node.',ha='center',fontsize=15.5)
    fig.text(.5,.025,'Legend: arrows = directed edges; labels = costs; dashed with × = blocked; thick solid = new route.',ha='center',fontsize=15.5)
    save(fig,'fig_4_7_replanning')
    return {'before':before,'closed':{k:v if math.isfinite(v) else 'infinity' for k,v in closed.items()},'reopened':before,'trace':[{kind:{k:v if math.isfinite(v) else 'infinity' for k,v in values.items()} for kind,values in s.items()} for s in stages]}

def main():
    jps=improvements();trace=repair();report={'weighted_candidates':{'a':{'g':2,'h':4},'b':{'g':5,'h':2}},'shortcut_length':math.sqrt(8),'theta_candidate_cost':1+math.sqrt(8),'directed_repair':trace,'jps_illustration':jps}
    (ROOT/'results'/'algorithm_illustrations.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('Figures4.6–4.7 regenerated; symmetry, corner rule and numerical repair trace verified.')

if __name__=='__main__':main()
