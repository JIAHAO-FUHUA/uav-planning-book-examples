"""Reproduce Sections 4.2.7/4.2.8: map-event execution and 3D sampling.

Standard library only. Run from the directory containing planning.py.
Maps/boxes are already inflated forbidden regions for the vehicle centre.
"""
from dataclasses import dataclass,field,asdict
import argparse,json,math
from pathlib import Path
from planning import dynamic_scene,static_scene,search,DStarLite,validate_path,equal,length
from sampling_dynamics import Random32

EPS=1e-9
ROOT=Path(__file__).resolve().parent

def moving_replanning():
    grid,start,goal=dynamic_scene();planner=DStarLite(grid,start,goal)
    current=start;executed=[start];old_path=[];snapshots=[]
    def observe(label,changes):
        nonlocal old_path
        planner.update_cells(changes);result=planner.compute();fresh=search(grid,current,goal)
        assert equal(result.cost,fresh.cost)
        assert bool(result.path)==bool(fresh.path)
        if result.path:
            validate_path(grid,result.path,current,goal)
            assert abs(length(result.path)-result.cost)<EPS
        snapshot={'label':label,'step':len(executed)-1,'current':current,'goal':goal,
            'changes':changes,'blocked':sorted(grid.blocked),'executed':list(executed),
            'previous_plan':old_path,'path':result.path,'status':'route' if result.path else 'hold',
            'remaining_cost':result.cost if result.path else None,
            'dstar_processed':len(result.expanded),'astar_processed':len(fresh.expanded)}
        snapshots.append(snapshot);old_path=list(result.path)
        return result
    def advance(result,edges):
        nonlocal current
        assert len(result.path)>edges
        for a,b in zip(result.path[:edges],result.path[1:edges+1]):
            # Validate under the current snapshot before every simulated move.
            assert grid.visible(a,b) and math.isfinite(grid.cost(a,b))
            assert executed[-1]==a
            executed.append(b);current=b;planner.move_start(current)
    initial=observe('Initial map',[])
    advance(initial,14)
    closed=observe('Close lower passage',[((24,y),True) for y in range(6,10)])
    advance(closed,6)
    reopened=observe('Reopen lower passage',[((24,y),False) for y in range(6,10)])
    advance(reopened,3)
    blocked=observe('Close both passages',[((24,y),True) for y in list(range(6,10))+list(range(24,28))])
    assert not blocked.path
    assert current[0]<24 and grid.free(current)
    # The abstract simulator holds at current; it does not follow the obsolete route.
    return {'width':grid.width,'height':grid.height,'start':start,'goal':goal,
            'snapshots':snapshots,'total_executed_edges':len(executed)-1,'held_position':current}

P3=tuple[float,float,float]

@dataclass
class Scene3D:
    low:P3=(0.0,0.0,0.5)
    high:P3=(14.0,10.0,5.5)
    # (xmin,xmax,ymin,ymax,zmin,zmax), closed boxes, already inflated.
    boxes:tuple=((6.0,7.0,0.0,10.0,0.5,3.4),(9.0,10.0,0.0,10.0,2.2,5.5))
    segment_checks:int=0
    def free(self,p):
        return all(lo-EPS<=x<=hi+EPS for x,lo,hi in zip(p,self.low,self.high)) and not any(
            all(box[2*i]-EPS<=p[i]<=box[2*i+1]+EPS for i in range(3)) for box in self.boxes)
    def segment_free(self,a,b):
        self.segment_checks+=1
        if not self.free(a) or not self.free(b):return False
        for box in self.boxes:
            lo,hi=0.,1.
            for i in range(3):
                delta=b[i]-a[i]
                if abs(delta)<EPS:
                    if a[i]<box[2*i]-EPS or a[i]>box[2*i+1]+EPS:lo=2.;break
                else:
                    enter,leave=sorted(((box[2*i]-a[i])/delta,(box[2*i+1]-a[i])/delta))
                    lo=max(lo,enter);hi=min(hi,leave)
            if lo<=hi+EPS:return False
        return True
    def sample(self,rng):
        for _ in range(100000):
            p=tuple(lo+(hi-lo)*rng.uniform() for lo,hi in zip(self.low,self.high))
            if self.free(p):return p
        raise RuntimeError('No free sample; inspect scene bounds and obstacles')

@dataclass
class Result3D:
    algorithm:str
    seed:int
    path:list[P3]
    nodes:list[P3]
    parents:list[int|None]
    cost:float|None
    draws:int
    first_draw:int|None
    first_cost:float|None
    first_path:list[P3]
    rewires:int
    segment_checks:int
    history:list=field(default_factory=list)

def validate3d(scene,result,start,goal):
    if not result.path:
        assert result.cost is None;return
    assert result.path[0]==start and result.path[-1]==goal
    assert all(scene.segment_free(a,b) for a,b in zip(result.path,result.path[1:]))
    assert abs(sum(math.dist(a,b) for a,b in zip(result.path,result.path[1:]))-result.cost)<1e-7

def rrt3d(scene,start=(1.,5.,1.),goal=(13.,5.,1.),budget=2000,seed=17,
          star=True,eta=1.,gamma=20.,goal_bias=.07):
    """Both methods continue for the full budget, retaining their best goal route.

    Identical samples/nearest-node extensions give identical accepted coordinates;
    RRT* changes parents/costs. This explicitly continuing RRT differs from the
    first-route stopping rule of Section 4.2.5.
    """
    if not scene.free(start) or not scene.free(goal):raise ValueError('Invalid endpoint')
    if eta<=0 or gamma<=0 or budget<1 or not 0<=goal_bias<=1:raise ValueError('Invalid settings')
    rng=Random32(seed);nodes=[start];parents=[None];costs=[0.];children=[set()]
    checks0=scene.segment_checks;rewires=0;history=[]
    first_draw=None;first_cost=None;first_path=[]
    best=math.inf;best_node=None
    def extract(i):
        ids=[]
        while i is not None:ids.append(i);i=parents[i]
        path=[nodes[j] for j in reversed(ids)]
        if math.dist(path[-1],goal)>EPS:path.append(goal)
        return path
    def candidate():
        options=[(costs[i]+math.dist(p,goal),i) for i,p in enumerate(nodes)
            if math.dist(p,goal)<=eta+EPS and scene.segment_free(p,goal)]
        return min(options) if options else (math.inf,None)
    for draw in range(1,budget+1):
        target=goal if rng.uniform()<goal_bias else scene.sample(rng)
        near=min(range(len(nodes)),key=lambda i:math.dist(nodes[i],target))
        a=nodes[near];dist=math.dist(a,target)
        if dist<EPS:continue
        scale=min(1.,eta/dist);new=tuple(a[i]+scale*(target[i]-a[i]) for i in range(3))
        if not scene.segment_free(a,new) or any(math.dist(new,p)<EPS for p in nodes):continue
        parent=near;g=costs[near]+math.dist(a,new);neighbors=[]
        if star:
            n=len(nodes)+1;radius=min(eta,gamma*(math.log(n)/n)**(1/3))
            neighbors=[i for i,p in enumerate(nodes) if math.dist(p,new)<=radius+EPS]
            for i in neighbors:
                c=costs[i]+math.dist(nodes[i],new)
                if c<g-EPS and scene.segment_free(nodes[i],new):parent=i;g=c
        j=len(nodes);nodes.append(new);parents.append(parent);costs.append(g);children.append(set());children[parent].add(j)
        if star:
            for i in neighbors:
                if i==0 or i==parent:continue
                c=g+math.dist(new,nodes[i])
                if c<costs[i]-EPS and scene.segment_free(new,nodes[i]):
                    children[parents[i]].remove(i);parents[i]=j;children[j].add(i)
                    delta=c-costs[i];todo=[i]
                    while todo:
                        k=todo.pop();costs[k]+=delta;todo.extend(sorted(children[k]))
                    rewires+=1
        best,best_node=candidate()
        if best_node is not None:
            if first_draw is None:first_draw=draw;first_cost=best;first_path=extract(best_node)
            history.append((draw,best))
    best,best_node=candidate();path=extract(best_node) if best_node is not None else []
    for i,p in enumerate(parents):
        if p is not None:assert abs(costs[p]+math.dist(nodes[p],nodes[i])-costs[i])<1e-7
    result=Result3D('RRT*' if star else 'RRT',seed,path,nodes,parents,best if path else None,
        budget,first_draw,first_cost,first_path,rewires,scene.segment_checks-checks0,history)
    assert all(b[1]<=a[1]+1e-7 for a,b in zip(history,history[1:]))
    validate3d(scene,result,start,goal)
    return result

def run_3d(seeds=(3,17,41,73,101),budget=2000):
    records=[];display=[]
    for seed in seeds:
        pair=[]
        for star in [False,True]:
            scene=Scene3D();r=rrt3d(scene,budget=budget,seed=seed,star=star)
            pair.append(r)
            records.append({'seed':seed,'algorithm':r.algorithm,'success':bool(r.path),
                'cost':r.cost,'nodes':len(r.nodes),'draws':r.draws,'first_draw':r.first_draw,
                'first_cost':r.first_cost,'rewires':r.rewires,'segment_checks':r.segment_checks})
            print('3D',seed,r.algorithm,'length',round(r.cost,3) if r.cost is not None else 'no_path',
                  'nodes',len(r.nodes),'first',r.first_draw,'rewires',r.rewires,flush=True)
        assert pair[0].nodes==pair[1].nodes,'Comparison must use identical accepted coordinates'
        if seed==17:display=[asdict(r) for r in pair]
    # Lowering the ceiling seals the only route above the first full-width wall.
    low=Scene3D(high=(14.,10.,3.4));failure=rrt3d(low,budget=budget,seed=17)
    assert not failure.path
    print('3D low ceiling 3.4 m: no_path; budget',budget,'nodes',len(failure.nodes),flush=True)
    return {'scene':{'low':Scene3D().low,'high':Scene3D().high,'boxes':Scene3D().boxes},
        'start':(1.,5.,1.),'goal':(13.,5.,1.),'settings':{'eta':1.,'gamma':20.,'goal_bias':.07,'budget':budget,'seeds':seeds},
        'display':display,'seed_results':records,'low_ceiling':{'high':low.high,'status':'budget_exhausted','nodes':len(failure.nodes),'draws':failure.draws}}

def write_outputs(data,name):
    out=ROOT/'results';out.mkdir(exist_ok=True)
    (out/(name+'.json')).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--case',choices=['grid','3d','all'],default='all')
    ap.add_argument('--seed',type=int,help='Run one 3D seed instead of the documented five-seed sweep')
    ap.add_argument('--budget',type=int,default=2000)
    args=ap.parse_args()
    if args.case in ['grid','all']:
        grid,s,t=static_scene()
        rows=[]
        for name,w,th in [('Dijkstra',0,False),('A*',1,False),('Weighted A*',2,False),('Theta*',1,True)]:
            result=search(grid,s,t,w,th);validate_path(grid,result.path,s,t)
            rows.append({'algorithm':name,'cost':result.cost,'processed':len(result.expanded)})
        data=moving_replanning();data['static']=rows;write_outputs(data,'simulation_grid')
        for e in data['snapshots']:
            print('GRID',e['label'],'step',e['step'],'current',e['current'],'remaining',
                round(e['remaining_cost'],3) if e['remaining_cost'] is not None else 'no_path',
                'D*',e['dstar_processed'],'A*',e['astar_processed'],'status',e['status'])
    if args.case in ['3d','all']:
        seeds=(args.seed,) if args.seed is not None else (3,17,41,73,101)
        write_outputs(run_3d(seeds,args.budget),'simulation_3d')

if __name__=='__main__':main()
