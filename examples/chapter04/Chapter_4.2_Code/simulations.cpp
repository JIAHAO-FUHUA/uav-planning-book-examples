// Sections 4.2.7/4.2.8. Compile beside the supplied shared grid header.
#include "grid_planning_core.hpp"
#include <cstdint>
#include <set>
#include <stdexcept>

using P3=array<double,3>;
double distance3(P3 a,P3 b){double z=0;for(int i=0;i<3;i++)z+=(a[i]-b[i])*(a[i]-b[i]);return sqrt(z);}
void check(bool ok,const string& message){if(!ok)throw runtime_error(message);}
struct Random3 {
    uint32_t state;
    explicit Random3(uint32_t seed):state(seed){}
    double uniform(){state=1664525u*state+1013904223u;return double(state)/4294967296.0;}
};
struct Scene3 {
    P3 low{0,0,.5},high{14,10,5.5};
    vector<array<double,6>> boxes{{6,7,0,10,.5,3.4},{9,10,0,10,2.2,5.5}};
    long checks=0;
    bool free(P3 p)const{
        for(int i=0;i<3;i++)if(p[i]<low[i]-EPS||p[i]>high[i]+EPS)return false;
        for(auto b:boxes){bool inside=true;for(int i=0;i<3;i++)inside=inside&&p[i]>=b[2*i]-EPS&&p[i]<=b[2*i+1]+EPS;if(inside)return false;}
        return true;
    }
    bool segment(P3 a,P3 b){
        checks++;if(!free(a)||!free(b))return false;
        for(auto box:boxes){
            double lo=0,hi=1;
            for(int i=0;i<3;i++){
                double delta=b[i]-a[i];
                if(abs(delta)<EPS){if(a[i]<box[2*i]-EPS||a[i]>box[2*i+1]+EPS){lo=2;break;}}
                else{double t0=(box[2*i]-a[i])/delta,t1=(box[2*i+1]-a[i])/delta;if(t0>t1)swap(t0,t1);lo=max(lo,t0);hi=min(hi,t1);}
            }
            if(lo<=hi+EPS)return false;
        }
        return true;
    }
    P3 sample(Random3& rng){
        for(int k=0;k<100000;k++){P3 p;for(int i=0;i<3;i++)p[i]=low[i]+(high[i]-low[i])*rng.uniform();if(free(p))return p;}
        throw runtime_error("No free sample");
    }
};
struct Tree3 {
    string algorithm;
    vector<P3> path,nodes,first_path;
    vector<int> parents;
    double cost=INF,first_cost=INF;
    int seed=17,draws=0,first_draw=-1,rewires=0;
    long segment_checks=0;
    vector<pair<int,double>> history;
};
vector<P3> extract3(const vector<P3>& nodes,const vector<int>& parents,int i,P3 goal){
    vector<P3> p;for(;i>=0;i=parents[i])p.push_back(nodes[i]);reverse(p.begin(),p.end());
    if(distance3(p.back(),goal)>EPS)p.push_back(goal);return p;
}
void validate3(Scene3& scene,const Tree3& r,P3 start,P3 goal){
    if(r.path.empty()){check(!isfinite(r.cost),"Failure has finite cost");return;}
    check(r.path.front()==start&&r.path.back()==goal,"Wrong endpoint");double cost=0;
    for(size_t i=1;i<r.path.size();i++){check(scene.segment(r.path[i-1],r.path[i]),"Collision in returned path");cost+=distance3(r.path[i-1],r.path[i]);}
    check(abs(cost-r.cost)<1e-7,"Path cost mismatch");
}
Tree3 rrt3(Scene3& scene,int seed=17,int budget=2000,bool star=true){
    P3 start{1,5,1},goal{13,5,1};check(scene.free(start)&&scene.free(goal),"Invalid endpoint");
    Random3 rng(seed);Tree3 r;r.algorithm=star?"RRT*":"RRT";r.seed=seed;r.draws=budget;r.nodes.push_back(start);r.parents.push_back(-1);
    vector<double> costs{0};vector<set<int>> children(1);long initial=scene.checks;
    auto candidate=[&](){
        pair<double,int> best{INF,-1};
        for(int i=0;i<int(r.nodes.size());i++){double d=distance3(r.nodes[i],goal);if(d<=1+EPS&&scene.segment(r.nodes[i],goal)){pair<double,int> p{costs[i]+d,i};if(p<best)best=p;}}
        return best;
    };
    for(int draw=1;draw<=budget;draw++){
        P3 target=rng.uniform()<.07?goal:scene.sample(rng);int near=0;
        for(int i=1;i<int(r.nodes.size());i++)if(distance3(r.nodes[i],target)<distance3(r.nodes[near],target))near=i;
        P3 a=r.nodes[near];double d=distance3(a,target);if(d<EPS)continue;
        double scale=min(1.,1./d);P3 p;for(int i=0;i<3;i++)p[i]=a[i]+scale*(target[i]-a[i]);
        if(!scene.segment(a,p))continue;bool duplicate=false;
        for(auto q:r.nodes)if(distance3(p,q)<EPS){duplicate=true;break;}if(duplicate)continue;
        int parent=near;double g=costs[near]+distance3(a,p);vector<int> neighbors;
        if(star){
            int n=int(r.nodes.size())+1;double radius=min(1.,20*pow(log(double(n))/n,1./3));
            for(int i=0;i<int(r.nodes.size());i++)if(distance3(r.nodes[i],p)<=radius+EPS)neighbors.push_back(i);
            for(int i:neighbors){double c=costs[i]+distance3(r.nodes[i],p);if(c<g-EPS&&scene.segment(r.nodes[i],p)){g=c;parent=i;}}
        }
        int j=int(r.nodes.size());r.nodes.push_back(p);r.parents.push_back(parent);costs.push_back(g);children.emplace_back();children[parent].insert(j);
        if(star)for(int i:neighbors){
            if(i==0||i==parent)continue;double c=g+distance3(p,r.nodes[i]);
            if(c<costs[i]-EPS&&scene.segment(p,r.nodes[i])){
                children[r.parents[i]].erase(i);r.parents[i]=j;children[j].insert(i);double delta=c-costs[i];vector<int> todo{i};
                while(!todo.empty()){int k=todo.back();todo.pop_back();costs[k]+=delta;for(int q:children[k])todo.push_back(q);}r.rewires++;
            }
        }
        auto best=candidate();
        if(best.second>=0){
            if(r.first_draw<0){r.first_draw=draw;r.first_cost=best.first;r.first_path=extract3(r.nodes,r.parents,best.second,goal);}
            r.history.emplace_back(draw,best.first);
        }
    }
    auto best=candidate();r.cost=best.first;if(best.second>=0)r.path=extract3(r.nodes,r.parents,best.second,goal);
    for(int i=1;i<int(r.nodes.size());i++)check(abs(costs[r.parents[i]]+distance3(r.nodes[r.parents[i]],r.nodes[i])-costs[i])<1e-7,"Invalid descendant cost");
    for(size_t i=1;i<r.history.size();i++)check(r.history[i].second<=r.history[i-1].second+1e-7,"Best cost increased");
    r.segment_checks=scene.checks-initial;validate3(scene,r,start,goal);return r;
}

void grid_case(){
    Grid grid(48,32);for(int y=0;y<32;y++)if(!((6<=y&&y<=9)||(24<=y&&y<=27)))grid.blocked[grid.id(24,y)]=true;
    int current=grid.id(4,8),goal=grid.id(44,8),steps=0;DStarLite planner(grid,current,goal);
    auto observe=[&](const string& label,const vector<pair<int,bool>>& changes){
        planner.change(changes);auto r=planner.compute(),a=search(grid,current,goal);
        check(eq(r.cost,a.cost),"Repair cost differs from A*");
        for(size_t i=1;i<r.path.size();i++)check(grid.visible(r.path[i-1],r.path[i]),"Repair collision");
        auto p=grid.p(current);cout<<"GRID "<<label<<" step "<<steps<<" current ("<<p.x<<","<<p.y<<") remaining ";
        if(isfinite(r.cost))cout<<r.cost;else cout<<"no_path";
        cout<<" D* "<<r.expanded<<" A* "<<a.expanded<<" status "<<(r.path.empty()?"hold":"route")<<"\n";
        return r;
    };
    auto move=[&](int x,int y){
        int next=grid.id(x,y);check(grid.visible(current,next)&&isfinite(grid.cost(current,next)),"Unsafe move");
        // Fixed equal-cost waypoints keep the two language versions at identical event positions.
        auto before=search(grid,current,goal),after=search(grid,next,goal);
        check(eq(before.cost,grid.cost(current,next)+after.cost),"Scripted step is not on a shortest route");
        current=next;planner.move(current);steps++;
    };
    observe("Initial map",{});for(int x=5;x<=18;x++)move(x,8);
    vector<pair<int,bool>> c;for(int y=6;y<=9;y++)c.push_back({grid.id(24,y),true});observe("Close lower passage",c);
    for(int y=9;y<=14;y++)move(18,y);
    c.clear();for(int y=6;y<=9;y++)c.push_back({grid.id(24,y),false});observe("Reopen lower passage",c);
    move(19,13);move(20,12);move(21,11);
    c.clear();for(int y:{6,7,8,9,24,25,26,27})c.push_back({grid.id(24,y),true});
    auto failure=observe("Close both passages",c);check(failure.path.empty(),"Expected blocked wall failure");
}
int main(int argc,char** argv){
    try{
        string which="all";int budget=2000;bool self=false;
        for(int i=1;i<argc;i++){
            string arg=argv[i];if(arg=="--case"&&i+1<argc)which=argv[++i];
            else if(arg=="--budget"&&i+1<argc)budget=stoi(argv[++i]);
            else if(arg=="--self-test")self=true;else throw runtime_error("Use --case grid|3d|all --budget N --self-test");
        }
        check(budget>0&&(which=="grid"||which=="3d"||which=="all"),"Invalid arguments");
        cout<<setprecision(12);
        if(which=="grid"||which=="all")grid_case();
        if(which=="3d"||which=="all"){
            for(int seed:{3,17,41,73,101}){
                Scene3 a,b;auto plain=rrt3(a,seed,budget,false),star=rrt3(b,seed,budget,true);
                check(plain.nodes==star.nodes,"Different accepted coordinates");
                for(const auto* r:{&plain,&star})cout<<"3D "<<seed<<" "<<r->algorithm<<" length "<<r->cost<<" nodes "<<r->nodes.size()<<" first "<<r->first_draw<<" rewires "<<r->rewires<<"\n";
            }
            Scene3 low;low.high[2]=3.4;auto fail=rrt3(low,17,budget);check(fail.path.empty(),"Expected sealed-ceiling failure");
            cout<<"3D low ceiling 3.4 m: no_path; budget "<<budget<<" nodes "<<fail.nodes.size()<<"\n";
        }
        if(self){
            Scene3 s;check(!s.segment({5,5,1},{8,5,1}),"Missed interior collision");
            check(!s.segment({5,5,3.4},{8,5,3.4}),"Accepted wall contact");
            check(s.segment({5,5,3.41},{8,5,3.41}),"Rejected clear wall crossing");
            check(!s.segment({8,5,2.2},{11,5,2.2}),"Accepted beam contact");
            check(s.segment({8,5,2.19},{11,5,2.19}),"Rejected clear beam crossing");
            cout<<"SELF_TEST passed\n";
        }
        return 0;
    }catch(const exception& e){cerr<<e.what()<<"\n";return 1;}
}
