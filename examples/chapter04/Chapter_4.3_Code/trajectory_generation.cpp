// Chapter 4.3: independent fixed-time equality QP, normalized septic polynomials.
// Compile: g++ -O2 -std=c++17 trajectory_generation.cpp -o trajectory.exe
// Run: ./trajectory.exe --self-test
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
using V=std::vector<double>;using Mat=std::vector<V>;
double fact(int k){double x=1;for(int i=2;i<=k;++i)x*=i;return x;}
V basis(int r,double s,double d,int degree=7){V a(degree+1);for(int k=r;k<=degree;++k)a[k]=fact(k)/fact(k-r)*std::pow(s,k-r)/std::pow(d,r);return a;}
Mat solve(Mat a,Mat b){
 int n=int(a.size()),dim=int(b[0].size());
 for(int k=0;k<n;++k){int pivot=k;for(int i=k+1;i<n;++i)if(std::abs(a[i][k])>std::abs(a[pivot][k]))pivot=i;
  if(std::abs(a[pivot][k])<1e-18)throw std::runtime_error("singular linear system");std::swap(a[k],a[pivot]);std::swap(b[k],b[pivot]);
  for(int i=k+1;i<n;++i){double f=a[i][k]/a[k][k];for(int j=k;j<n;++j)a[i][j]-=f*a[k][j];for(int j=0;j<dim;++j)b[i][j]-=f*b[k][j];}
 }
 Mat x(n,V(dim));for(int k=n-1;k>=0;--k)for(int j=0;j<dim;++j){double v=b[k][j];for(int i=k+1;i<n;++i)v-=a[k][i]*x[i][j];x[k][j]=v/a[k][k];}return x;
}
double evaluate(const V&c,double x){double y=0;for(auto i=c.rbegin();i!=c.rend();++i)y=y*x+*i;return y;}
V derivative(V c){if(c.size()<2)return V{0};V d(c.size()-1);for(size_t k=1;k<c.size();++k)d[k-1]=k*c[k];return d;}
V multiply(const V&a,const V&b){V c(a.size()+b.size()-1);for(size_t i=0;i<a.size();++i)for(size_t j=0;j<b.size();++j)c[i+j]+=a[i]*b[j];return c;}
V roots(V c){
 // Recursively isolate real roots on [0,1] using derivative-root partitions.
 while(c.size()>1 && std::abs(c.back())<1e-12)c.pop_back();if(c.size()<2)return {};
 V critical=roots(derivative(c)),cuts{0};cuts.insert(cuts.end(),critical.begin(),critical.end());cuts.push_back(1);std::sort(cuts.begin(),cuts.end());V out;
 for(double x:cuts)if(std::abs(evaluate(c,x))<1e-9)out.push_back(x);
 for(size_t i=1;i<cuts.size();++i){double a=cuts[i-1],b=cuts[i],fa=evaluate(c,a),fb=evaluate(c,b);if(fa*fb>=0)continue;
  for(int k=0;k<70;++k){double m=(a+b)/2,fm=evaluate(c,m);if(fa*fm<=0){b=m;fb=fm;}else{a=m;fa=fm;}}out.push_back((a+b)/2);
 }
 std::sort(out.begin(),out.end());out.erase(std::unique(out.begin(),out.end(),[](double a,double b){return std::abs(a-b)<1e-8;}),out.end());return out;
}
struct Trajectory{
 std::vector<Mat> c;V durations;double residual=0;
 V value(int i,double s,int r=0)const{V row=basis(r,s,durations[i]),v(c[i][0].size());for(int k=0;k<8;++k)for(size_t j=0;j<v.size();++j)v[j]+=row[k]*c[i][k][j];return v;}
 double time()const{double x=0;for(double d:durations)x+=d;return x;}
 double snap()const{double j=0;for(size_t i=0;i<c.size();++i)for(int k=4;k<8;++k)for(int l=4;l<8;++l){double h=fact(k)/fact(k-4)*fact(l)/fact(l-4)/(k+l-7)/std::pow(durations[i],7);for(size_t a=0;a<c[i][0].size();++a)j+=c[i][k][a]*h*c[i][l][a];}return j;}
 double maximum(int r)const{
  double best=0;
  for(size_t i=0;i<c.size();++i){V norm2(2*(8-r)-1);for(size_t a=0;a<c[i][0].size();++a){V v(8-r);for(int k=r;k<8;++k)v[k-r]=c[i][k][a]*fact(k)/fact(k-r)/std::pow(durations[i],r);V sq=multiply(v,v);for(size_t j=0;j<sq.size();++j)norm2[j]+=sq[j];}
   V points=roots(derivative(norm2));points.push_back(0);points.push_back(1);for(double s:points){V v=value(int(i),s,r);double q=0;for(double x:v)q+=x*x;best=std::max(best,std::sqrt(q));}
  }return best;
 }
 Trajectory dilated(double rho)const{auto t=*this;for(double&d:t.durations)d*=rho;return t;}
 bool collision(const V&low,const V&high)const{
  for(size_t i=0;i<c.size();++i){V cuts{0,1};for(size_t a=0;a<low.size();++a)for(double boundary:{low[a],high[a]}){V cc(8);for(int k=0;k<8;++k)cc[k]=c[i][k][a];cc[0]-=boundary;V rr=roots(cc);cuts.insert(cuts.end(),rr.begin(),rr.end());}std::sort(cuts.begin(),cuts.end());
   auto inside=[&](double s){V p=value(int(i),s);for(size_t a=0;a<low.size();++a)if(p[a]<low[a]-1e-8 || p[a]>high[a]+1e-8)return false;return true;};
   for(double s:cuts)if(inside(s))return true;for(size_t k=1;k<cuts.size();++k)if(inside((cuts[k-1]+cuts[k])/2))return true;
  }return false;
 }
};
Trajectory minimum_snap(const Mat&w,const V&ds){
 int m=int(ds.size()),n=m*8,dim=int(w[0].size());for(double d:ds)if(d<=0)throw std::invalid_argument("positive times required");
 Mat q(n,V(n)),a,b;
 auto add=[&](int i,int r,double s,const V&target){V row(n);V rr=basis(r,s,ds[i]);for(int k=0;k<8;++k)row[i*8+k]=rr[k];a.push_back(row);b.push_back(target);};
 for(int i=0;i<m;++i){for(int k=4;k<8;++k)for(int l=4;l<8;++l)q[8*i+k][8*i+l]=2*fact(k)/fact(k-4)*fact(l)/fact(l-4)/(k+l-7)/std::pow(ds[i],7);add(i,0,0,w[i]);add(i,0,1,w[i+1]);}
 for(auto endpoint:std::array<std::pair<int,double>,2>{{{0,0},{m-1,1}}})for(int r=1;r<=3;++r)add(endpoint.first,r,endpoint.second,V(dim));
 for(int i=0;i<m-1;++i)for(int r=1;r<=3;++r){V row(n),left=basis(r,1,ds[i]),right=basis(r,0,ds[i+1]);for(int k=0;k<8;++k){row[i*8+k]=left[k];row[(i+1)*8+k]=-right[k];}a.push_back(row);b.push_back(V(dim));}
 int constraints=int(a.size()),size=n+constraints;double qscale=0;for(auto row:q)for(double x:row)qscale=std::max(qscale,std::abs(x));Mat system(size,V(size)),rhs(size,V(dim));
 for(int i=0;i<n;++i)for(int j=0;j<n;++j)system[i][j]=q[i][j]/qscale;
 for(int k=0;k<constraints;++k){double norm=0;for(double x:a[k])norm+=x*x;norm=std::sqrt(norm);for(int j=0;j<n;++j)system[n+k][j]=system[j][n+k]=a[k][j]/norm;for(int j=0;j<dim;++j)rhs[n+k][j]=b[k][j]/norm;}
 Mat x=solve(system,rhs);Trajectory out;out.durations=ds;out.c.assign(m,Mat(8,V(dim)));for(int i=0;i<m;++i)for(int k=0;k<8;++k)out.c[i][k]=x[8*i+k];
 for(int k=0;k<constraints;++k)for(int j=0;j<dim;++j){double v=0;for(int i=0;i<n;++i)v+=a[k][i]*x[i][j];out.residual=std::max(out.residual,std::abs(v-b[k][j]));}return out;
}
Trajectory stopped(const Mat&w,const V&ds){Trajectory t;t.durations=ds;V shape{0,0,0,0,35,-84,70,-20};for(size_t i=0;i<ds.size();++i){Mat c(8,V(w[0].size()));for(size_t a=0;a<w[0].size();++a){c[0][a]=w[i][a];for(int k=1;k<8;++k)c[k][a]=(w[i+1][a]-w[i][a])*shape[k];}t.c.push_back(c);}return t;}
void check(bool value,const char*message){if(!value)throw std::runtime_error(message);}
void print(const char*name,const Trajectory&t){std::cout<<name<<" time "<<t.time()<<" snap "<<t.snap()<<" speed "<<t.maximum(1)<<" accel "<<t.maximum(2)<<'\n';}
int main(int argc,char**argv){try{
 std::cout<<std::setprecision(15);Mat w{{1,3,1},{4,7,1},{9,7,1},{13,3,1}};V ds{2,2,2};auto through=minimum_snap(w,ds),stop=stopped(w,ds);
 double rho=std::max({1.,through.maximum(1)/2.,std::sqrt(through.maximum(2)/2.5)})*1.01;auto scaled=through.dilated(rho);
 double sr=std::max({1.,stop.maximum(1)/2.,std::sqrt(stop.maximum(2)/2.5)})*1.01;auto safe=stop.dilated(sr);
 auto corner=minimum_snap({{0,0},{2,0},{2,2}},{2,2}),corner_stop=stopped({{0,0},{2,0},{2,2}},{2,2});
 print("through",through);print("stop",stop);print("scaled",scaled);print("safe_stop",safe);print("corner",corner);print("corner_stop",corner_stop);
 std::filesystem::create_directories("results");std::ofstream csv("results/cpp_coefficients.csv");csv<<std::setprecision(17);for(size_t i=0;i<through.c.size();++i)for(int k=0;k<8;++k)csv<<i<<','<<k<<','<<through.c[i][k][0]<<','<<through.c[i][k][1]<<','<<through.c[i][k][2]<<'\n';
 if(argc>1){
  check(through.residual<1e-7,"constraint residual");check(through.snap()<stop.snap(),"joint cost");check(scaled.maximum(1)<2 && scaled.maximum(2)<2.5,"dilation bounds");
  check(!through.collision({6,0,.5},{7,6,1.5}),"lower box");check(through.collision({5.8,9.1,.5},{6.6,9.7,1.5}),"overshoot collision");check(scaled.collision({5.8,9.1,.5},{6.6,9.7,1.5}),"retiming keeps collision");check(!safe.collision({5.8,9.1,.5},{6.6,9.7,1.5}),"safe stopped geometry");
  for(int i=0;i<2;++i)for(int r=0;r<4;++r){V a=through.value(i,1,r),b=through.value(i+1,0,r);for(size_t j=0;j<a.size();++j)check(std::abs(a[j]-b[j])<1e-7,"C3 continuity");}
  auto single=minimum_snap({{0},{1}},{1});check(std::abs(single.snap()-100800)<1e-5,"known single cost");V expected{0,0,0,0,35,-84,70,-20};for(int k=0;k<8;++k)check(std::abs(single.c[0][k][0]-expected[k])<1e-7,"known coefficients");
  Trajectory line;line.durations={1};line.c={Mat(8,V(2))};line.c[0][1][0]=1;check(line.collision({.4999,-.1},{.5001,.1}),"thin box");line.c[0][0][1]=1;check(line.collision({.4,0},{.6,1}),"boundary contact");
  std::cout<<"SELF_TEST passed\n";
 }
 return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
