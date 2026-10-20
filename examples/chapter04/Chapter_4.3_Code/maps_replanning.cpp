// Sections 4.3.5--4.3.6: analytic geometry and admission logic, not a flight stack.
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using V=std::array<double,2>;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
double update(double l,bool hit){return std::clamp(l+(hit?.85:-.4),-2.,3.5);}
std::string occupancy(double l,bool observed){
    if(!observed)return "unknown";
    double p=1/(1+std::exp(-l));return p>=.65?"occupied":p<=.35?"free":"unknown";
}
struct Query{double distance;V gradient;};
Query circle(V point,V center={2,0},double radius=.4){
    V d={point[0]-center[0],point[1]-center[1]};double length=std::hypot(d[0],d[1]);
    check(length>=1e-14,"gradient is not unique at circle center");
    return {length-radius,{d[0]/length,d[1]/length}};
}
struct Penalty{double value;V gradient;};
Penalty penalty(V point,double required=.3,double weight=1){
    auto q=circle(point);double e=std::max(0.,required-q.distance);
    return {weight*e*e,{-2*weight*e*q.gradient[0],-2*weight*e*q.gradient[1]}};
}
bool interval(const std::vector<double>& distances,double speed,double step,double required,double error=0){
    check(std::min({speed,step,required,error})>=0,"nonnegative bounds required");
    if(distances.empty())return false;
    for(double d:distances)if(!std::isfinite(d)||d<required+error+speed*step/2)return false;
    return true;
}
struct Ticket{int parent,map;double switch_time;std::string frame="world";};
std::string admit(Ticket t,double now,int parent,int map,double mismatch,
                  bool prefix=true,bool candidate=true,bool tracking=true,
                  std::string frame="world",double reserve=.05){
    if(!std::isfinite(now)||now>t.switch_time-reserve)return "late";
    if(parent!=t.parent)return "stale-parent";
    if(frame!=t.frame)return "wrong-frame";
    if(map!=t.map)return "revalidate-map";
    if(!std::isfinite(mismatch)||mismatch>1e-7)return "discontinuous";
    if(!prefix||!tracking)return "recover-prefix";
    if(!candidate)return "reject-candidate";
    return "scheduled";
}
int main(int argc,char** argv){try{
    auto p=penalty({1.4,0});Ticket t{7,12,1.6};
    if(argc>1&&std::string(argv[1])=="--self-test"){
        check(occupancy(0,false)=="unknown","unknown became free");
        check(occupancy(update(0,true),true)=="occupied","hit update");
        check(occupancy(-1.2,true)=="free","miss update");
        check(update(3.5,true)==3.5,"confidence clamp");
        V x{1.4,.1};auto g=penalty(x);
        for(int j=0;j<2;++j){auto a=x,b=x;a[j]+=1e-6;b[j]-=1e-6;
            check(std::abs((penalty(a).value-penalty(b).value)/2e-6-g.gradient[j])<1e-8,"gradient");}
        check(!interval({1.6,1.6},1,4,.3),"missed collision admitted");
        std::vector<double> ds;for(int i=0;i<=40;++i)ds.push_back(circle({i*.1,1}).distance);
        check(interval(ds,1,.1,.3),"separated line check");
        check(admit(t,1.3,7,12,0)=="scheduled","on-time gate");
        check(admit(t,1.58,7,12,0)=="late","late gate");
        check(admit(t,1.3,8,12,0)=="stale-parent","parent gate");
        check(admit(t,1.3,7,13,0)=="revalidate-map","map gate");
        check(admit(t,1.3,7,12,0,true,true,true,"odom")=="wrong-frame","frame gate");
        check(admit(t,1.3,7,12,.1)=="discontinuous","continuity gate");
        check(admit(t,1.3,7,12,0,false)=="recover-prefix","prefix gate");
        check(admit(t,1.3,7,12,0,true,false)=="reject-candidate","candidate gate");
        std::cout<<"C++ map, gradient, interval and admission checks passed\n";
    }
    std::cout<<std::setprecision(16)<<"PENALTY "<<p.value<<' '<<p.gradient[0]<<' '<<p.gradient[1]
        <<"\nINTERVAL "<<interval({1.6,1.6},1,4,.3)<<"\nSTATUSES "
        <<admit(t,1.3,7,12,0)<<' '<<admit(t,1.58,7,12,0)<<' '
        <<admit(t,1.3,7,13,0)<<' '<<admit(t,1.3,8,12,0)<<'\n';
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
