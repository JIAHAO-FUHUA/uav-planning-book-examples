"""PRM, RRT/RRT*, and a planar double-integrator motion-primitive search.

Standard library only. Distances are metres; primitive durations are seconds.
Obstacles represent already inflated forbidden regions for the vehicle centre.
"""
from dataclasses import dataclass, field
import heapq
import itertools
import json
import math
from pathlib import Path

EPS=1e-9
Point=tuple[float,float]
State=tuple[float,float,float,float]

class Random32:
    """Specified 32-bit generator, shared exactly with the C++ implementation."""
    def __init__(self,seed):self.state=seed & 0xffffffff
    def uniform(self):
        self.state=(1664525*self.state+1013904223)&0xffffffff
        return self.state/4294967296.0

def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def path_length(path):return sum(distance(a,b) for a,b in zip(path,path[1:]))

@dataclass
class Scene:
    width:float=14.0
    height:float=10.0
    obstacles:tuple= ((6.0,7.0,0.0,6.0),)
    def free(self,p):
        return -EPS<=p[0]<=self.width+EPS and -EPS<=p[1]<=self.height+EPS and not any(
            x0-EPS<=p[0]<=x1+EPS and y0-EPS<=p[1]<=y1+EPS for x0,x1,y0,y1 in self.obstacles)
    def segment_free(self,a,b):
        if not self.free(a) or not self.free(b):return False
        # Closed-rectangle slab intersection tests the entire segment.
        for rect in self.obstacles:
            low,high=0.0,1.0
            for axis,(r0,r1) in enumerate([(rect[0],rect[1]),(rect[2],rect[3])]):
                delta=b[axis]-a[axis]
                if abs(delta)<EPS:
                    if a[axis]<r0-EPS or a[axis]>r1+EPS:low=2.0;break
                else:
                    enter,leave=sorted(((r0-a[axis])/delta,(r1-a[axis])/delta))
                    low=max(low,enter);high=min(high,leave)
            if low<=high+EPS:return False
        return True
    def sample(self,rng):
        for _ in range(100000):
            p=(self.width*rng.uniform(),self.height*rng.uniform())
            if self.free(p):return p
        raise RuntimeError('Cannot sample free space')

@dataclass
class SamplingResult:
    algorithm:str
    path:list[Point]
    nodes:list[Point]
    edges:list[tuple[int,int]]
    cost:float
    draws:int
    rewires:int=0
    history:list=field(default_factory=list)

def validate_geometric(scene,result,start,goal):
    assert result.path and distance(result.path[0],start)<EPS and distance(result.path[-1],goal)<EPS
    assert all(scene.segment_free(a,b) for a,b in zip(result.path,result.path[1:]))
    assert abs(path_length(result.path)-result.cost)<1e-7
    assert all(scene.segment_free(result.nodes[i],result.nodes[j]) for i,j in result.edges)

def prm(scene,start,goal,n=250,k=12,seed=17):
    """Build a reusable roadmap, then add and collision-check query connections."""
    if not scene.free(start) or not scene.free(goal):raise ValueError('Invalid endpoint')
    rng=Random32(seed);nodes=[scene.sample(rng) for _ in range(n)]+[start,goal]
    graph=[[] for _ in nodes];edges=[];seen=set()
    # Roadmap connections never depend on the query endpoints.
    for i in range(n):
        near=sorted((distance(nodes[i],nodes[j]),j) for j in range(n) if j!=i)[:k]
        for d,j in near:
            pair=(min(i,j),max(i,j))
            if pair not in seen and scene.segment_free(nodes[i],nodes[j]):
                seen.add(pair);edges.append(pair);graph[i].append((j,d));graph[j].append((i,d))
    for i in [n,n+1]:
        for d,j in sorted((distance(nodes[i],nodes[j]),j) for j in range(n))[:k]:
            if scene.segment_free(nodes[i],nodes[j]):
                edges.append((i,j));graph[i].append((j,d));graph[j].append((i,d))
    if scene.segment_free(start,goal):
        d=distance(start,goal);edges.append((n,n+1));graph[n].append((n+1,d));graph[n+1].append((n,d))
    q=[(0.0,0,n)];count=itertools.count(1);cost={n:0.0};parent={n:None}
    while q:
        c,_,i=heapq.heappop(q)
        if c>cost[i]+EPS:continue
        if i==n+1:
            ids=[];v=i
            while v is not None:ids.append(v);v=parent[v]
            path=[nodes[v] for v in reversed(ids)]
            result=SamplingResult('PRM',path,nodes,edges,c,n)
            validate_geometric(scene,result,start,goal);return result
        for j,d in graph[i]:
            candidate=c+d
            if candidate<cost.get(j,math.inf)-EPS:
                cost[j]=candidate;parent[j]=i;heapq.heappush(q,(candidate,next(count),j))
    return SamplingResult('PRM',[],nodes,edges,math.inf,n)

def rrt(scene,start,goal,budget=600,eta=1.0,seed=17,star=False,gamma=20.0,goal_bias=.07):
    if not scene.free(start) or not scene.free(goal):raise ValueError('Invalid endpoint')
    rng=Random32(seed);nodes=[start];parents=[None];costs=[0.0];children=[set()]
    rewires=0;history=[];first=None
    def goal_candidate():
        candidates=[(costs[i]+distance(p,goal),i) for i,p in enumerate(nodes)
                    if distance(p,goal)<=eta+EPS and scene.segment_free(p,goal)]
        return min(candidates) if candidates else (math.inf,None)
    def extract(i):
        ids=[]
        while i is not None:ids.append(i);i=parents[i]
        path=[nodes[j] for j in reversed(ids)]
        if distance(path[-1],goal)>EPS:path.append(goal)
        return path
    for draw in range(1,budget+1):
        target=goal if rng.uniform()<goal_bias else scene.sample(rng)
        nearest=min(range(len(nodes)),key=lambda i:distance(nodes[i],target))
        p=nodes[nearest];d=distance(p,target)
        if d<EPS:continue
        scale=min(1.0,eta/d);new=(p[0]+scale*(target[0]-p[0]),p[1]+scale*(target[1]-p[1]))
        if not scene.segment_free(p,new):continue
        if any(distance(new,v)<EPS for v in nodes):continue
        parent=nearest;g=costs[nearest]+distance(p,new);near=[]
        if star:
            n=len(nodes)+1;radius=min(eta,gamma*math.sqrt(math.log(n)/n))
            near=[i for i,v in enumerate(nodes) if distance(v,new)<=radius+EPS]
            for i in near:
                candidate=costs[i]+distance(nodes[i],new)
                if candidate<g-EPS and scene.segment_free(nodes[i],new):parent=i;g=candidate
        idx=len(nodes);nodes.append(new);parents.append(parent);costs.append(g);children.append(set());children[parent].add(idx)
        if star:
            for i in near:
                if i==0 or i==parent:continue
                candidate=g+distance(new,nodes[i])
                if candidate<costs[i]-EPS and scene.segment_free(new,nodes[i]):
                    # Updating descendants keeps every stored root-to-node cost valid.
                    old=parents[i];children[old].remove(i);parents[i]=idx;children[idx].add(i)
                    delta=candidate-costs[i];stack=[i]
                    while stack:
                        j=stack.pop();costs[j]+=delta;stack.extend(sorted(children[j]))
                    rewires+=1
        best,node=goal_candidate()
        if node is not None:
            if first is None:first=draw
            history.append((draw,best))
            if not star:
                result=SamplingResult('RRT',extract(node),nodes,[(p,i) for i,p in enumerate(parents) if p is not None],best,draw)
                validate_geometric(scene,result,start,goal);return result
    best,node=goal_candidate();path=extract(node) if node is not None else []
    edges=[(p,i) for i,p in enumerate(parents) if p is not None]
    for i,p in enumerate(parents):
        if p is not None:assert abs(costs[p]+distance(nodes[p],nodes[i])-costs[i])<1e-7
    result=SamplingResult('RRT*' if star else 'RRT',path,nodes,edges,best,budget,rewires,history)
    if path:validate_geometric(scene,result,start,goal)
    if star:assert all(b[1]<=a[1]+1e-7 for a,b in zip(history,history[1:]))
    return result

def propagate(x,u,tau=1.0):
    return (x[0]+x[2]*tau+.5*u[0]*tau*tau,x[1]+x[3]*tau+.5*u[1]*tau*tau,
            x[2]+u[0]*tau,x[3]+u[1]*tau)

def roots(a,b,c):
    if abs(a)<EPS:return [] if abs(b)<EPS else [-c/b]
    delta=b*b-4*a*c
    if delta<-EPS:return []
    delta=math.sqrt(max(0.0,delta));return [(-b-delta)/(2*a),(-b+delta)/(2*a)]

def primitive_status(scene,x,u,tau=1.0,vmax=2.5,amax=1.5):
    if tau<=0:raise ValueError('Duration must be positive')
    end=propagate(x,u,tau)
    if math.hypot(*u)>amax+EPS:return 'acceleration'
    # Velocity is affine in time; its norm's maximum is at an interval endpoint.
    if max(math.hypot(x[2],x[3]),math.hypot(end[2],end[3]))>vmax+EPS:return 'speed'
    if not scene.free(x[:2]) or not scene.free(end[:2]):return 'collision'
    for axis,limit in [(0,scene.width),(1,scene.height)]:
        times=[0.0,tau]
        if abs(u[axis])>EPS:
            extremum=-x[axis+2]/u[axis]
            if 0<extremum<tau:times.append(extremum)
        positions=[propagate(x,u,t)[axis] for t in times]
        if min(positions)<-EPS or max(positions)>limit+EPS:return 'bounds'
    # Rectangle boundaries partition time into intervals with a constant inside/outside status.
    # Testing all boundary times and one midpoint per interval certifies the full quadratic arc.
    for rect in scene.obstacles:
        times=[0.0,tau]
        for axis,bounds in [(0,rect[:2]),(1,rect[2:])]:
            for boundary in bounds:
                times.extend(t for t in roots(.5*u[axis],x[axis+2],x[axis]-boundary) if 0<=t<=tau)
        times=sorted(set(times));checks=times+[(a+b)/2 for a,b in zip(times,times[1:])]
        for t in checks:
            p=propagate(x,u,t)
            if rect[0]-EPS<=p[0]<=rect[1]+EPS and rect[2]-EPS<=p[1]<=rect[3]+EPS:return 'collision'
    return 'legal'

CONTROLS=tuple((a,b) for a in [-1.0,0.0,1.0] for b in [-1.0,0.0,1.0])

@dataclass
class KinoResult:
    states:list[State]
    controls:list[Point]
    cost:float
    expanded:int
    duration:float
    effort:float

def state_key(x):
    key=(round(2*x[0]),round(2*x[1]),round(x[2]),round(x[3]))
    assert max(abs(x[i]-(key[i]/2 if i<2 else key[i])) for i in range(4))<EPS
    return key

def kinodynamic_search(scene,start,goal,tau=1.0,vmax=2.5,amax=1.5,weight=.2,use_h=True):
    """Exact 0.5 m position / 1 m/s velocity lattice for integer controls and tau=1."""
    if tau!=1.0:raise ValueError('This exact lattice implementation uses tau=1 s')
    if weight<0 or not scene.free(start[:2]) or not scene.free(goal[:2]):raise ValueError('Invalid problem')
    if max(math.hypot(*start[2:]),math.hypot(*goal[2:]))>vmax+EPS:raise ValueError('Endpoint exceeds speed bound')
    initial=state_key(start);target=state_key(goal)
    def h(x):return distance(x,goal)/vmax if use_h else 0.0
    order=itertools.count();queue=[(h(start),0.0,next(order),initial)];g={initial:0.0};states={initial:start};parents={initial:None}
    expanded=0
    while queue:
        f,c,_,key=heapq.heappop(queue)
        if c>g[key]+EPS:continue
        expanded+=1
        if key==target:
            path=[];controls=[];node=key
            while parents[node] is not None:
                path.append(states[node]);parent,u=parents[node];controls.append(u);node=parent
            path.append(start);path.reverse();controls.reverse()
            effort=sum((u[0]**2+u[1]**2)*tau for u in controls)
            result=KinoResult(path,controls,c,expanded,len(controls)*tau,effort)
            validate_kino(scene,result,start,goal,tau,vmax,amax,weight);return result
        x=states[key]
        for u in CONTROLS:
            if primitive_status(scene,x,u,tau,vmax,amax)!='legal':continue
            end=propagate(x,u,tau);nk=state_key(end);candidate=c+tau+weight*(u[0]**2+u[1]**2)*tau
            if candidate<g.get(nk,math.inf)-EPS:
                g[nk]=candidate;states[nk]=end;parents[nk]=(key,u)
                heapq.heappush(queue,(candidate+h(end),candidate,next(order),nk))
    return KinoResult([],[],math.inf,expanded,0.0,0.0)

def validate_kino(scene,result,start,goal,tau=1.0,vmax=2.5,amax=1.5,weight=.2):
    assert result.states[0]==start and result.states[-1]==goal
    assert len(result.states)==len(result.controls)+1
    for a,b,u in zip(result.states,result.states[1:],result.controls):
        assert primitive_status(scene,a,u,tau,vmax,amax)=='legal'
        assert max(abs(i-j) for i,j in zip(propagate(a,u,tau),b))<EPS
    assert abs(result.duration+weight*result.effort-result.cost)<1e-7

def run_examples():
    scene=Scene();start=(1.0,3.0);goal=(13.0,3.0)
    samples=[prm(scene,start,goal),rrt(scene,start,goal),rrt(scene,start,goal,star=True)]
    for r in samples:
        if not r.path:raise RuntimeError(r.algorithm+' failed; inspect the recorded seed and budget')
        print(r.algorithm,round(r.cost,3),'nodes',len(r.nodes),'draws',r.draws,'rewires',r.rewires)
    kstart=(*start,2.0,0.0);kgoal=(*goal,0.0,0.0)
    kino=kinodynamic_search(scene,kstart,kgoal)
    if not kino.states:raise RuntimeError('Primitive search failed')
    print('Kinodynamic A*','cost',round(kino.cost,3),'time',kino.duration,'effort',kino.effort,'expanded',kino.expanded)
    fan_scene=Scene(6.0,5.0,((3.3,4.6,1.7,2.3),));fan_state=(2.0,2.0,2.0,0.0)
    fan=[{'u':u,'status':primitive_status(fan_scene,fan_state,u)} for u in CONTROLS]
    counts={s:sum(r['status']==s for r in fan) for s in ['legal','collision','speed']}
    print('Primitive fan',counts)
    report={'scene':{'width':scene.width,'height':scene.height,'obstacles':scene.obstacles},'start':start,'goal':goal,
      'settings':{'seed':17,'prm_samples':250,'prm_k':12,'rrt_budget':600,'eta':1.0,'gamma':20.0,'goal_bias':.07,
                  'tau':1.0,'vmax':2.5,'amax':1.5,'weight':.2},
      'sampling':[r.__dict__ for r in samples],'kino':kino.__dict__,'fan':fan,'fan_counts':counts}
    return report

if __name__=='__main__':
    report=run_examples();out=Path(__file__).resolve().parent/'results';out.mkdir(exist_ok=True)
    (out/'sampling_dynamics.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
