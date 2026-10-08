#pragma once
// Shared grid implementation extracted from planning.cpp; its program main is omitted.
// Original C++17 teaching implementations. All coordinates are cell centers.
// Compile: g++ -O2 -std=c++17 planning.cpp -o planning
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <string>
#include <tuple>
#include <vector>
using namespace std;
const double INF=numeric_limits<double>::infinity(), EPS=1e-9;
bool eq(double a,double b){return a==b || abs(a-b)<=EPS;}
struct Point {int x,y;};
double dist(Point a,Point b){return hypot(a.x-b.x,a.y-b.y);}
double oct(Point a,Point b){int x=abs(a.x-b.x),y=abs(a.y-b.y);return max(x,y)+(sqrt(2.)-1)*min(x,y);}
bool box(Point a,Point b,double x0,double x1,double y0,double y1){
    double lo=0,hi=1;
    for(auto axis: {array<double,4>{double(a.x),double(b.x-a.x),x0,x1},array<double,4>{double(a.y),double(b.y-a.y),y0,y1}}){
        auto [s,d,mn,mx]=axis;
        if(d==0){if(s<mn || s>mx)return false;}
        else {double p=(mn-s)/d,q=(mx-s)/d;if(p>q)swap(p,q);lo=max(lo,p);hi=min(hi,q);if(lo>hi+1e-12)return false;}
    }return true;
}
struct Grid {
    int w,h; vector<bool> blocked;
    Grid(int width,int height):w(width),h(height),blocked(w*h,false){}
    int id(int x,int y)const{return y*w+x;}
    Point p(int u)const{return {u%w,u/w};}
    bool inside(int x,int y)const{return x>=0 && x<w && y>=0 && y<h;}
    bool free(int x,int y)const{return inside(x,y)&&!blocked[id(x,y)];}
    vector<int> adj(int u)const{
        static const int dx[]={1,0,-1,0,1,-1,-1,1},dy[]={0,1,0,-1,1,1,-1,-1};
        vector<int> out;auto a=p(u);
        for(int k=0;k<8;k++)if(inside(a.x+dx[k],a.y+dy[k]))out.push_back(id(a.x+dx[k],a.y+dy[k]));
        return out;
    }
    double cost(int u,int v)const{
        if(blocked[u]||blocked[v])return INF;
        auto a=p(u),b=p(v);int dx=b.x-a.x,dy=b.y-a.y;
        if(max(abs(dx),abs(dy))!=1)return INF;
        if(dx&&dy&&(!free(a.x+dx,a.y)||!free(a.x,a.y+dy)))return INF;
        return hypot(dx,dy);
    }
    bool visible(int u,int v)const{
        if(blocked[u]||blocked[v])return false;
        auto a=p(u),b=p(v);
        for(int y=min(a.y,b.y);y<=max(a.y,b.y);y++)for(int x=min(a.x,b.x);x<=max(a.x,b.x);x++)
            if(blocked[id(x,y)]&&box(a,b,x-.5,x+.5,y-.5,y+.5))return false;
        return true;
    }
};
struct Result {vector<int> path; double cost=INF; int expanded=0;};
Result search(const Grid& m,int start,int goal,double weight=1,bool theta=false){
    Result r;if(m.blocked[start]||m.blocked[goal])return r;
    vector<double> g(m.w*m.h,INF);vector<int> parent(m.w*m.h,-1);
    using Entry=tuple<double,double,int,double,int>;
    priority_queue<Entry,vector<Entry>,greater<Entry>> q;int serial=0;
    auto h=[&](int u){return weight*(theta?dist(m.p(u),m.p(goal)):oct(m.p(u),m.p(goal)));};
    g[start]=0;parent[start]=start;q.emplace(h(start),h(start),serial++,0,start);
    while(!q.empty()){
        auto [f,hv,n,old,u]=q.top();q.pop();if(old>g[u]+EPS)continue;r.expanded++;
        if(u==goal){r.cost=g[u];for(int v=goal;;v=parent[v]){r.path.push_back(v);if(v==start)break;}reverse(r.path.begin(),r.path.end());return r;}
        for(int v:m.adj(u)){
            double c=m.cost(u,v);if(!isfinite(c))continue;
            double candidate=g[u]+c;int pred=u,pu=parent[u];
            if(theta&&m.visible(pu,v)){double z=g[pu]+dist(m.p(pu),m.p(v));if(z<candidate){candidate=z;pred=pu;}}
            if(candidate+EPS<g[v]){g[v]=candidate;parent[v]=pred;q.emplace(candidate+h(v),h(v),serial++,candidate,v);}
        }
    }return r;
}
using Key=pair<double,double>;
bool lesskey(Key a,Key b){if(!eq(a.first,b.first))return a.first<b.first;return !eq(a.second,b.second)&&a.second<b.second;}
class DStarLite {
    Grid& m;int start,goal,last;double km=0;vector<double> g,rhs;vector<int> active;
    using Entry=tuple<double,double,int,int>;
    priority_queue<Entry,vector<Entry>,greater<Entry>> q;int serial=0;
    Key key(int u)const{double z=min(g[u],rhs[u]);return {z+oct(m.p(start),m.p(u))+km,z};}
    void insert(int u){auto [a,b]=key(u);active[u]=++serial;q.emplace(a,b,serial,u);}
    void clean(){while(!q.empty()&&active[get<3>(q.top())]!=get<2>(q.top()))q.pop();}
    Key top(){clean();if(q.empty())return {INF,INF};return {get<0>(q.top()),get<1>(q.top())};}
    void update(int u){
        if(u!=goal){rhs[u]=INF;for(int v:m.adj(u))rhs[u]=min(rhs[u],m.cost(u,v)+g[v]);}
        active[u]=0;if(!eq(g[u],rhs[u]))insert(u);
    }
public:
    DStarLite(Grid& grid,int s,int t):m(grid),start(s),goal(t),last(s),g(m.w*m.h,INF),rhs(m.w*m.h,INF),active(m.w*m.h,0){rhs[goal]=0;insert(goal);}
    void move(int s){km+=oct(m.p(last),m.p(s));start=last=s;}
    void change(const vector<pair<int,bool>>& changes){
        vector<int> affected;
        for(auto [u,b]:changes){m.blocked[u]=b;affected.push_back(u);for(int v:m.adj(u))affected.push_back(v);}
        sort(affected.begin(),affected.end());affected.erase(unique(affected.begin(),affected.end()),affected.end());
        for(int u:affected)update(u);
    }
    Result compute(){
        Result r;
        while(lesskey(top(),key(start))||!eq(rhs[start],g[start])){
            clean();if(q.empty())throw runtime_error("empty inconsistent queue");
            auto [a,b,n,u]=q.top();q.pop();active[u]=0;
            if(lesskey({a,b},key(u))){insert(u);continue;}
            r.expanded++;
            if(g[u]>rhs[u]){g[u]=rhs[u];for(int v:m.adj(u))update(v);}
            else{g[u]=INF;update(u);for(int v:m.adj(u))update(v);}
        }
        if(m.blocked[start]||m.blocked[goal]||!isfinite(g[start]))return r;
        vector<bool> seen(m.w*m.h,false);int u=start;r.cost=0;r.path.push_back(u);seen[u]=true;
        while(u!=goal){
            double best=INF;int next=-1;
            for(int v:m.adj(u)){double z=m.cost(u,v)+g[v];if(z<best){best=z;next=v;}}
            if(next<0||seen[next])throw runtime_error("invalid extracted path");
            r.cost+=m.cost(u,next);u=next;seen[u]=true;r.path.push_back(u);
        }return r;
    }
};
Grid static_scene(){
    Grid m(48,32);
    for(int y=0;y<25;y++)if(y<7||y>10)m.blocked[m.id(18,y)]=true;
    for(int y=7;y<32;y++)if(y<23||y>26)m.blocked[m.id(32,y)]=true;
    for(int x=8;x<12;x++)for(int y=13;y<19;y++)m.blocked[m.id(x,y)]=true;
    for(int x=38;x<42;x++)for(int y=12;y<18;y++)m.blocked[m.id(x,y)]=true;
    return m;
}
void self_test(){
    mt19937 rng(20260916);int count=0;
    for(int k=0;k<60;k++){
        Grid m(8,8);int s=0,t=63;
        for(int i=1;i<63;i++)m.blocked[i]=(rng()%100<20);
        DStarLite ds(m,s,t);
        for(int e=0;e<12;e++){
            auto base=search(m,s,t,0),a=search(m,s,t),r=ds.compute(),theta=search(m,s,t,1,true);
            if(!eq(base.cost,a.cost)||!eq(base.cost,r.cost))throw runtime_error("cost disagreement");
            if(theta.path.empty()!=base.path.empty())throw runtime_error("Theta feasibility disagreement");
            for(size_t j=1;j<theta.path.size();j++)if(!m.visible(theta.path[j-1],theta.path[j]))throw runtime_error("Theta collision");
            if(r.path.size()>1&&e%3==0){s=r.path[1];ds.move(s);}
            vector<pair<int,bool>> changes;
            for(int z=0;z<2;z++){int u=int(rng()%64);if(u!=s&&u!=t)changes.push_back({u,!m.blocked[u]});}
            ds.change(changes);count++;
        }
    }cout<<"PASS: "<<count<<" C++ random update comparisons\n";
}
