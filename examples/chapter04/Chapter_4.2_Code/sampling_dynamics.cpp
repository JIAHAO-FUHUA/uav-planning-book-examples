// C++17 teaching implementations. Units: metres and seconds.
// Obstacles are already inflated forbidden regions for the vehicle centre.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

constexpr double EPS = 1e-9;
const double INF = std::numeric_limits<double>::infinity();
using Point = std::array<double, 2>;
using State = std::array<double, 4>;
using Key = std::array<int, 4>;
using Rect = std::array<double, 4>;
double distance(Point a, Point b) { return std::hypot(a[0]-b[0], a[1]-b[1]); }
Point position(State x) { return {x[0], x[1]}; }
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }

struct Random32 {
    uint32_t state;
    explicit Random32(uint32_t seed) : state(seed) {}
    double uniform() {
        state = 1664525u * state + 1013904223u;
        return static_cast<double>(state) / 4294967296.0;
    }
};

struct Scene {
    double width=14, height=10;
    std::vector<Rect> obstacles={{6,7,0,6}};
    bool free(Point p) const {
        if (p[0]<-EPS || p[0]>width+EPS || p[1]<-EPS || p[1]>height+EPS) return false;
        for (auto r : obstacles)
            if (r[0]-EPS<=p[0] && p[0]<=r[1]+EPS && r[2]-EPS<=p[1] && p[1]<=r[3]+EPS) return false;
        return true;
    }
    bool segment_free(Point a, Point b) const {
        if (!free(a) || !free(b)) return false;
        for (auto r : obstacles) {
            double low=0, high=1;
            for (int axis=0; axis<2; ++axis) {
                double delta=b[axis]-a[axis], r0=r[2*axis], r1=r[2*axis+1];
                if (std::abs(delta)<EPS) {
                    if (a[axis]<r0-EPS || a[axis]>r1+EPS) { low=2; break; }
                } else {
                    double enter=(r0-a[axis])/delta, leave=(r1-a[axis])/delta;
                    if (enter>leave) std::swap(enter,leave);
                    low=std::max(low,enter); high=std::min(high,leave);
                }
            }
            if (low<=high+EPS) return false;
        }
        return true;
    }
    Point sample(Random32& rng) const {
        for (int i=0; i<100000; ++i) {
            Point p={width*rng.uniform(), height*rng.uniform()};
            if (free(p)) return p;
        }
        throw std::runtime_error("Cannot sample free space");
    }
};

struct SamplingResult {
    std::string algorithm;
    std::vector<Point> path, nodes;
    std::vector<std::pair<int,int>> edges;
    double cost=INF;
    int draws=0, rewires=0;
};
void validate(const Scene& scene, const SamplingResult& r, Point start, Point goal) {
    require(!r.path.empty(), "Empty path");
    require(distance(r.path.front(),start)<EPS && distance(r.path.back(),goal)<EPS,"Wrong endpoints");
    double cost=0;
    for (size_t i=1; i<r.path.size(); ++i) {
        require(scene.segment_free(r.path[i-1],r.path[i]),"Segment collision");
        cost+=distance(r.path[i-1],r.path[i]);
    }
    require(std::abs(cost-r.cost)<1e-7,"Incorrect route length");
    for (auto [i,j] : r.edges) require(scene.segment_free(r.nodes[i],r.nodes[j]),"Invalid graph edge");
}

SamplingResult prm(const Scene& scene, Point start, Point goal, int n=250, int k=12, uint32_t seed=17) {
    require(scene.free(start) && scene.free(goal), "Invalid endpoint");
    Random32 rng(seed); SamplingResult r; r.algorithm="PRM"; r.draws=n;
    for (int i=0; i<n; ++i) r.nodes.push_back(scene.sample(rng));
    r.nodes.push_back(start); r.nodes.push_back(goal);
    std::vector<std::vector<std::pair<int,double>>> graph(n+2);
    std::set<std::pair<int,int>> seen;
    auto edge=[&](int i,int j,double d) {
        r.edges.emplace_back(i,j); graph[i].emplace_back(j,d); graph[j].emplace_back(i,d);
    };
    for (int i=0; i<n; ++i) {
        std::vector<std::pair<double,int>> near;
        for (int j=0; j<n; ++j) if (i!=j) near.emplace_back(distance(r.nodes[i],r.nodes[j]),j);
        std::sort(near.begin(),near.end());
        for (int a=0; a<std::min(k,static_cast<int>(near.size())); ++a) {
            auto [d,j]=near[a]; std::pair<int,int> pair={std::min(i,j),std::max(i,j)};
            if (!seen.count(pair) && scene.segment_free(r.nodes[i],r.nodes[j])) {
                seen.insert(pair); edge(pair.first,pair.second,d);
            }
        }
    }
    for (int i : {n,n+1}) {
        std::vector<std::pair<double,int>> near;
        for (int j=0; j<n; ++j) near.emplace_back(distance(r.nodes[i],r.nodes[j]),j);
        std::sort(near.begin(),near.end());
        for (int a=0; a<std::min(k,n); ++a) {
            auto [d,j]=near[a]; if (scene.segment_free(r.nodes[i],r.nodes[j])) edge(i,j,d);
        }
    }
    if (scene.segment_free(start,goal)) edge(n,n+1,distance(start,goal));
    using Item=std::tuple<double,uint64_t,int>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> q;
    std::vector<double> cost(n+2,INF); std::vector<int> parent(n+2,-1); uint64_t order=0;
    cost[n]=0; q.emplace(0,order++,n);
    while (!q.empty()) {
        auto [c,_,i]=q.top(); q.pop(); if (c>cost[i]+EPS) continue;
        if (i==n+1) {
            for (int v=i; v!=-1; v=parent[v]) r.path.push_back(r.nodes[v]);
            std::reverse(r.path.begin(),r.path.end()); r.cost=c; validate(scene,r,start,goal); return r;
        }
        for (auto [j,d] : graph[i]) if (c+d<cost[j]-EPS) {
            cost[j]=c+d; parent[j]=i; q.emplace(cost[j],order++,j);
        }
    }
    return r;
}

SamplingResult rrt(const Scene& scene, Point start, Point goal, bool star=false,
                   int budget=600, double eta=1, uint32_t seed=17, double gamma=20, double goal_bias=.07) {
    require(scene.free(start) && scene.free(goal),"Invalid endpoint");
    Random32 rng(seed); SamplingResult r; r.algorithm=star?"RRT*":"RRT";
    r.nodes={start}; std::vector<int> parents={-1}; std::vector<double> costs={0};
    std::vector<std::set<int>> children(1); double last_best=INF;
    auto goal_candidate=[&]() {
        std::pair<double,int> best={INF,-1};
        for (int i=0; i<static_cast<int>(r.nodes.size()); ++i) {
            double d=distance(r.nodes[i],goal);
            if (d<=eta+EPS && scene.segment_free(r.nodes[i],goal))
                best=std::min(best,std::pair<double,int>{costs[i]+d,i});
        }
        return best;
    };
    auto finish=[&](double best,int node,int draw) {
        r.cost=best; r.draws=draw;
        if (node!=-1) {
            for (int i=node; i!=-1; i=parents[i]) r.path.push_back(r.nodes[i]);
            std::reverse(r.path.begin(),r.path.end());
            if (distance(r.path.back(),goal)>EPS) r.path.push_back(goal);
        }
        for (int i=1; i<static_cast<int>(parents.size()); ++i) {
            r.edges.emplace_back(parents[i],i);
            require(std::abs(costs[parents[i]]+distance(r.nodes[parents[i]],r.nodes[i])-costs[i])<1e-7,"Stale descendant cost");
        }
        if (!r.path.empty()) validate(scene,r,start,goal);
    };
    for (int draw=1; draw<=budget; ++draw) {
        Point target= rng.uniform()<goal_bias ? goal : scene.sample(rng);
        int nearest=0; double d=distance(r.nodes[0],target);
        for (int i=1; i<static_cast<int>(r.nodes.size()); ++i) {
            double candidate=distance(r.nodes[i],target);
            if (candidate<d) { nearest=i; d=candidate; }
        }
        if (d<EPS) continue;
        Point p=r.nodes[nearest]; double scale=std::min(1.0,eta/d);
        Point added={p[0]+scale*(target[0]-p[0]),p[1]+scale*(target[1]-p[1])};
        if (!scene.segment_free(p,added)) continue;
        bool duplicate=false; for (auto v : r.nodes) if (distance(added,v)<EPS) duplicate=true;
        if (duplicate) continue;
        int parent=nearest; double g=costs[nearest]+distance(p,added); std::vector<int> near;
        if (star) {
            double n=r.nodes.size()+1.0, radius=std::min(eta,gamma*std::sqrt(std::log(n)/n));
            for (int i=0; i<static_cast<int>(r.nodes.size()); ++i)
                if (distance(r.nodes[i],added)<=radius+EPS) near.push_back(i);
            for (int i : near) {
                double candidate=costs[i]+distance(r.nodes[i],added);
                if (candidate<g-EPS && scene.segment_free(r.nodes[i],added)) { parent=i; g=candidate; }
            }
        }
        int idx=r.nodes.size(); r.nodes.push_back(added); parents.push_back(parent); costs.push_back(g);
        children.emplace_back(); children[parent].insert(idx);
        if (star) for (int i : near) {
            if (i==0 || i==parent) continue;
            double candidate=g+distance(added,r.nodes[i]);
            if (candidate<costs[i]-EPS && scene.segment_free(added,r.nodes[i])) {
                children[parents[i]].erase(i); parents[i]=idx; children[idx].insert(i);
                double delta=candidate-costs[i]; std::vector<int> stack={i};
                while (!stack.empty()) {
                    int j=stack.back(); stack.pop_back(); costs[j]+=delta;
                    for (int child : children[j]) stack.push_back(child);
                }
                ++r.rewires;
            }
        }
        auto [best,node]=goal_candidate();
        if (node!=-1) {
            require(best<=last_best+1e-7,"RRT* cost increased"); last_best=best;
            if (!star) { finish(best,node,draw); return r; }
        }
    }
    auto [best,node]=goal_candidate(); finish(best,node,budget); return r;
}

State propagate(State x,Point u,double tau) {
    return {x[0]+x[2]*tau+.5*u[0]*tau*tau,x[1]+x[3]*tau+.5*u[1]*tau*tau,
            x[2]+u[0]*tau,x[3]+u[1]*tau};
}
std::vector<double> roots(double a,double b,double c) {
    if (std::abs(a)<EPS) return std::abs(b)<EPS ? std::vector<double>{} : std::vector<double>{-c/b};
    double delta=b*b-4*a*c; if (delta<-EPS) return {};
    delta=std::sqrt(std::max(0.0,delta)); return {(-b-delta)/(2*a),(-b+delta)/(2*a)};
}
std::string primitive_status(const Scene& scene,State x,Point u,double tau=1,double vmax=2.5,double amax=1.5) {
    require(tau>0,"Duration must be positive"); State end=propagate(x,u,tau);
    if (std::hypot(u[0],u[1])>amax+EPS) return "acceleration";
    if (std::max(std::hypot(x[2],x[3]),std::hypot(end[2],end[3]))>vmax+EPS) return "speed";
    if (!scene.free(position(x)) || !scene.free(position(end))) return "collision";
    for (int axis=0; axis<2; ++axis) {
        std::vector<double> times={0,tau}; double limit=axis==0?scene.width:scene.height;
        if (std::abs(u[axis])>EPS) {
            double extremum=-x[axis+2]/u[axis]; if (0<extremum && extremum<tau) times.push_back(extremum);
        }
        for (double t : times) {
            double v=propagate(x,u,t)[axis]; if (v<-EPS || v>limit+EPS) return "bounds";
        }
    }
    for (auto rect : scene.obstacles) {
        std::vector<double> times={0,tau};
        for (int axis=0; axis<2; ++axis) for (int b=0; b<2; ++b)
            for (double t : roots(.5*u[axis],x[axis+2],x[axis]-rect[2*axis+b]))
                if (0<=t && t<=tau) times.push_back(t);
        std::sort(times.begin(),times.end()); times.erase(std::unique(times.begin(),times.end()),times.end());
        std::vector<double> checks=times;
        for (size_t i=1; i<times.size(); ++i) checks.push_back((times[i-1]+times[i])/2);
        for (double t : checks) {
            State p=propagate(x,u,t);
            if (rect[0]-EPS<=p[0] && p[0]<=rect[1]+EPS && rect[2]-EPS<=p[1] && p[1]<=rect[3]+EPS) return "collision";
        }
    }
    return "legal";
}
Key state_key(State x) {
    Key key={static_cast<int>(std::round(2*x[0])),static_cast<int>(std::round(2*x[1])),
             static_cast<int>(std::round(x[2])),static_cast<int>(std::round(x[3]))};
    for (int i=0; i<4; ++i) require(std::abs(x[i]-(i<2?key[i]/2.0:key[i]))<EPS,"State is off lattice");
    return key;
}
struct KinoResult {
    std::vector<State> states;
    std::vector<Point> controls;
    double cost=INF, duration=0, effort=0;
    int expanded=0;
};
KinoResult kinodynamic_search(const Scene& scene,State start,State goal,bool use_h=true,double weight=.2) {
    require(weight>=0 && scene.free(position(start)) && scene.free(position(goal)),"Invalid problem");
    require(std::max(std::hypot(start[2],start[3]),std::hypot(goal[2],goal[3]))<=2.5+EPS,"Endpoint exceeds speed bound");
    Key initial=state_key(start), target=state_key(goal);
    auto h=[&](State x) { return use_h?distance(position(x),position(goal))/2.5:0.0; };
    using Item=std::tuple<double,double,uint64_t,Key>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> q;
    std::map<Key,double> g; std::map<Key,State> states;
    std::map<Key,std::pair<Key,Point>> parent; uint64_t order=0;
    g[initial]=0; states[initial]=start; q.emplace(h(start),0,order++,initial); KinoResult r;
    while (!q.empty()) {
        auto [f,c,_,key]=q.top(); q.pop(); if (c>g[key]+EPS) continue;
        ++r.expanded;
        if (key==target) {
            for (Key node=key; node!=initial; node=parent.at(node).first) {
                r.states.push_back(states[node]); r.controls.push_back(parent.at(node).second);
            }
            r.states.push_back(start); std::reverse(r.states.begin(),r.states.end());
            std::reverse(r.controls.begin(),r.controls.end()); r.cost=c; r.duration=r.controls.size();
            for (size_t i=0; i<r.controls.size(); ++i) {
                Point u=r.controls[i]; r.effort+=u[0]*u[0]+u[1]*u[1];
                require(primitive_status(scene,r.states[i],u)=="legal","Invalid primitive");
                require(propagate(r.states[i],u,1)==r.states[i+1],"Propagation mismatch");
            }
            require(std::abs(r.duration+weight*r.effort-r.cost)<1e-7,"Incorrect trajectory cost"); return r;
        }
        State x=states[key];
        for (double a : {-1.0,0.0,1.0}) for (double b : {-1.0,0.0,1.0}) {
            Point u={a,b}; if (primitive_status(scene,x,u)!="legal") continue;
            State end=propagate(x,u,1); Key nk=state_key(end); double candidate=c+1+weight*(a*a+b*b);
            if (!g.count(nk) || candidate<g[nk]-EPS) {
                g[nk]=candidate; states[nk]=end; parent[nk]={key,u}; q.emplace(candidate+h(end),candidate,order++,nk);
            }
        }
    }
    return r;
}

int main(int argc,char** argv) {
    try {
        Scene scene; Point start={1,3}, goal={13,3};
        auto p=prm(scene,start,goal), r=rrt(scene,start,goal), s=rrt(scene,start,goal,true);
        std::cout<<std::setprecision(17);
        for (const auto* result : {&p,&r,&s}) {
            validate(scene,*result,start,goal);
            std::cout<<result->algorithm<<" cost "<<result->cost<<" nodes "<<result->nodes.size()
                     <<" draws "<<result->draws<<" rewires "<<result->rewires<<"\n";
        }
        State initial={1,3,2,0}, target={13,3,0,0}; auto k=kinodynamic_search(scene,initial,target);
        require(!k.states.empty(),"Primitive search failed");
        std::cout<<"KINODYNAMIC cost "<<k.cost<<" time "<<k.duration<<" effort "<<k.effort<<" expanded "<<k.expanded<<"\n";
        Scene fan{6,5,{{3.3,4.6,1.7,2.3}}}; std::map<std::string,int> counts;
        for (double a : {-1.0,0.0,1.0}) for (double b : {-1.0,0.0,1.0}) ++counts[primitive_status(fan,{2,2,2,0},{a,b})];
        std::cout<<"FAN legal "<<counts["legal"]<<" collision "<<counts["collision"]<<" speed "<<counts["speed"]<<"\n";
        if (argc>1 && std::string(argv[1])=="--self-test") {
            auto truth=kinodynamic_search(scene,initial,target,false);
            require(std::abs(truth.cost-k.cost)<1e-7,"A* differs from zero-heuristic search");
            Scene thin{4,3,{{.9,1.1,.4,.6}}};
            require(primitive_status(thin,{0,.5,2,0},{0,0})=="collision","Missed interior collision");
            require(counts["legal"]==2 && counts["collision"]==4 && counts["speed"]==3,"Wrong primitive classification");
            std::cout<<"SELF_TEST passed\n";
        }
        return 0;
    } catch (const std::exception& e) { std::cerr<<e.what()<<"\n"; return 1; }
}
