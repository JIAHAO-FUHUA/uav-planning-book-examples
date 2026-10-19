// Section 4.3.4: original fixed-band MINCO s=4 teaching implementation.
// Physical local time, fixed endpoint p/v/a/j, no collision/vehicle constraints.
// No-pivot LU requires moderate scales; residual checks are not a condition bound.
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <string>
using Vec=std::array<double,3>;
using Rows=std::vector<Vec>;
double dot(Vec a,Vec b){double s=0;for(int j=0;j<3;++j)s+=a[j]*b[j];return s;}
double falling(int k,int r){if(k<r)return 0;double a=1;for(int i=0;i<r;++i)a*=k-i;return a;}
double basis(int k,int r,double t){return k<r?0:falling(k,r)*std::pow(t,k-r);}
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
struct Band {
    int n;std::vector<std::array<double,17>> data;
    explicit Band(int size):n(size),data(size){}
    double get(int i,int j)const{return std::abs(i-j)>8?0:data[i][j-i+8];}
    void add(int i,int j,double x){if(std::abs(i-j)>8){require(x==0,"outside band");return;}data[i][j-i+8]+=x;}
    Band factor()const{
        Band a=*this;
        for(int k=0;k<n;++k){require(std::abs(a.get(k,k))>1e-13,"small band pivot");
            for(int i=k+1;i<std::min(n,k+9);++i){double f=a.get(i,k)/a.get(k,k);a.data[i][k-i+8]=f;
                for(int j=k+1;j<std::min(n,k+9);++j)a.add(i,j,-f*a.get(k,j));}}
        return a;
    }
    Rows solve(Rows x,bool transpose=false)const{
        if(!transpose){
            for(int i=0;i<n;++i)for(int j=std::max(0,i-8);j<i;++j)for(int d=0;d<3;++d)x[i][d]-=get(i,j)*x[j][d];
            for(int i=n-1;i>=0;--i){for(int j=i+1;j<std::min(n,i+9);++j)for(int d=0;d<3;++d)x[i][d]-=get(i,j)*x[j][d];for(double& v:x[i])v/=get(i,i);}
        }else{
            for(int i=0;i<n;++i){for(int j=std::max(0,i-8);j<i;++j)for(int d=0;d<3;++d)x[i][d]-=get(j,i)*x[j][d];for(double& v:x[i])v/=get(i,i);}
            for(int i=n-1;i>=0;--i)for(int j=i+1;j<std::min(n,i+9);++j)for(int d=0;d<3;++d)x[i][d]-=get(j,i)*x[j][d];
        }return x;
    }
};
struct Result {double energy=0;Rows q;std::vector<double> t;};
struct Minco {
    int m;std::vector<double> times;Band matrix,lu;Rows c,rhs;
    std::vector<std::vector<std::array<int,2>>> timeRows;
    double residual=0;
    Minco(Rows w,std::vector<double> ts,Rows head=Rows(3),Rows tail=Rows(3)):
        m(static_cast<int>(ts.size())),times(ts),matrix(8*m),lu(8*m),rhs(8*m),timeRows(m){
        require(m>0 && static_cast<int>(w.size())==m+1,"invalid shape");
        require(head.size()==3 && tail.size()==3,"invalid derivatives");
        for(double t:times)require(t>0 && std::isfinite(t),"positive finite times required");
        for(const auto& a:w)for(double v:a)require(std::isfinite(v),"finite waypoints required");
        for(const Rows* a:{&head,&tail})for(const auto& row:*a)for(double v:row)require(std::isfinite(v),"finite derivatives required");
        auto row=[&](int k,int i,int r,bool atEnd,bool continuous){
            for(int j=0;j<8;++j)matrix.add(k,8*i+j,basis(j,r,atEnd?times[i]:0));
            if(atEnd)timeRows[i].push_back({k,r});
            if(continuous)for(int j=0;j<8;++j)matrix.add(k,8*(i+1)+j,-basis(j,r,0));
        };
        for(int r=0;r<4;++r){row(r,0,r,false,false);rhs[r]=r==0?w.front():head[r-1];}
        for(int i=0;i<m-1;++i){int b=8*i+4;
            for(int j=0;j<3;++j)row(b+j,i,j+4,true,true);
            row(b+3,i,0,true,false);rhs[b+3]=w[i+1];
            for(int r=0;r<4;++r)row(b+4+r,i,r,true,true);
        }
        for(int r=0;r<4;++r){row(8*m-4+r,m-1,r,true,false);rhs[8*m-4+r]=r==0?w.back():tail[r-1];}
        lu=matrix.factor();c=lu.solve(rhs);
        for(int i=0;i<8*m;++i)for(int d=0;d<3;++d){double e=-rhs[i][d];for(int j=std::max(0,i-8);j<std::min(8*m,i+9);++j)e+=matrix.get(i,j)*c[j][d];residual=std::max(residual,std::abs(e));}
        require(residual<1e-7,"solve residual too large");
    }
    Vec value(int i,double theta,int r=0)const{Vec v{};for(int k=0;k<8;++k)for(int d=0;d<3;++d)v[d]+=basis(k,r,theta)*c[8*i+k][d];return v;}
    Result evaluate(double timeWeight=0)const{
        Rows gc(8*m);Result out;out.t.resize(m);out.q.resize(m-1);
        for(int i=0;i<m;++i){double tau=times[i];
            for(int k=4;k<8;++k)for(int l=4;l<8;++l){double h=falling(k,4)*falling(l,4)*std::pow(tau,k+l-7)/(k+l-7);out.energy+=h*dot(c[8*i+k],c[8*i+l]);for(int d=0;d<3;++d)gc[8*i+k][d]+=2*h*c[8*i+l][d];}
            Vec snap=value(i,tau,4);out.t[i]=dot(snap,snap)+timeWeight;out.energy+=timeWeight*tau;
        }
        Rows adj=lu.solve(gc,true);
        for(int i=0;i<m-1;++i)out.q[i]=adj[8*i+7];
        for(int i=0;i<m;++i)for(auto row:timeRows[i])out.t[i]-=dot(adj[row[0]],value(i,times[i],row[1]+1));
        return out;
    }
};
int main(int argc,char** argv){try{
    Rows w={Vec{0,0,1},Vec{1,.8,1},Vec{2,0,1}};std::vector<double> ts={1.2,1.8};Minco tr(w,ts);auto result=tr.evaluate(2);
    if(argc>1 && std::string(argv[1])=="--self-test"){
        for(int r=0;r<7;++r){auto a=tr.value(0,ts[0],r),b=tr.value(1,0,r);for(int d=0;d<3;++d)require(std::abs(a[d]-b[d])<1e-7,"C6 failed");}
        for(int i=0;i<2;++i){auto p=ts,n=ts;p[i]+=2e-5;n[i]-=2e-5;double fd=(Minco(w,p).evaluate(2).energy-Minco(w,n).evaluate(2).energy)/4e-5;require(std::abs(fd-result.t[i])/std::max(1.,std::abs(fd))<2e-6,"time gradient failed");}
        for(int d=0;d<3;++d){auto p=w,n=w;p[1][d]+=2e-5;n[1][d]-=2e-5;double fd=(Minco(p,ts).evaluate(2).energy-Minco(n,ts).evaluate(2).energy)/4e-5;require(std::abs(fd-result.q[0][d])/std::max(1.,std::abs(fd))<2e-6,"position gradient failed");}
        Rows h={Vec{.1,.2,-.1},Vec{0,.1,0},Vec{.05,0,0}};Minco moving(w,ts,h,h);
        for(int r=1;r<4;++r)for(int d=0;d<3;++d)require(std::abs(moving.value(0,0,r)[d]-h[r-1][d])<1e-8,"nonzero boundary failed");
        bool rejected=false;try{Minco invalid(w,{0,1});}catch(const std::runtime_error&){rejected=true;}require(rejected,"zero time accepted");
        std::cout<<"MINCO C++ checks passed\n";
    }
    std::cout<<std::setprecision(16)<<"ENERGY "<<tr.evaluate().energy<<"\nRESIDUAL "<<tr.residual<<"\nCOEFFICIENTS";
    for(auto row:tr.c)for(double x:row)std::cout<<' '<<x;
    std::cout<<"\nGRADIENT";for(auto row:result.q)for(double x:row)std::cout<<' '<<x;for(double x:result.t)std::cout<<' '<<x;std::cout<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
