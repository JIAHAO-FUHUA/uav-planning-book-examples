"""English editable diagrams showing trajectory algorithms, rather than run metrics."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from print_style import make_print_ready
from trajectory_generation import build_cases, collision_intervals
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'visual_figures';OUT.mkdir(exist_ok=True)
for name in ['times.ttf','timesi.ttf','timesbd.ttf','timesbi.ttf']:
    file=Path('C:/Windows/Fonts')/name
    if file.exists():font_manager.fontManager.addfont(str(file))
plt.rcParams.update({'font.family':'Times New Roman','font.size':19,'axes.labelsize':19,'axes.titlesize':20,'axes.titleweight':'bold','legend.fontsize':17,'xtick.labelsize':17,'ytick.labelsize':17,'svg.fonttype':'none','mathtext.fontset':'custom','mathtext.rm':'Times New Roman','mathtext.it':'Times New Roman:italic','mathtext.bf':'Times New Roman:bold','savefig.facecolor':'white'})
BLUE,GREEN,RED,GRAY='#111111','#444444','#222222','#999999'
def save(fig,name):
    make_print_ready(fig)
    fig.savefig(OUT/(name+'.svg'));fig.savefig(OUT/(name+'.png'),dpi=240);plt.close(fig)
def arrow(ax,start,end,color=GRAY,width=1.8):
    ax.annotate('',xy=end,xytext=start,arrowprops={'arrowstyle':'->','color':color,'lw':width})
def fig15():
    cases=build_cases();fig,axes=plt.subplots(1,3,figsize=(10.2,3.25));fig.subplots_adjust(left=.025,right=.98,top=.72,bottom=.31,wspace=.19)
    fig.text(.5,.88,'Corner handling determines the boundary conditions',ha='center',weight='bold',fontsize=22)
    for i,ax in enumerate(axes):
        ax.set_title(['1 Constant speed','2 Stop at W','3 Smooth passage'][i],pad=13);ax.set_aspect('equal');ax.set_xlim(-.5,2.8);ax.set_ylim(-.65,2.65);ax.axis('off')
        ax.plot([0,2,2],[0,0,2],color=GRAY,ls='--',lw=2.2)
        if i<2:ax.plot([0,2,2],[0,0,2],color=BLUE,lw=2.6)
        else:
            _,p=cases['corner'].sample();ax.plot(p[:,0],p[:,1],color=BLUE,lw=2.6)
        ax.scatter([0,2,2],[0,0,2],color='black',s=28,zorder=5)
        for x,y,label in [(0,0,'S'),(2,0,'W'),(2,2,'T')]:ax.text(x+.11,y-.28,label,fontsize=18)
        if i==0:
            arrow(ax,(1.2,0),(1.9,0),GREEN,2.2);arrow(ax,(2,.12),(2,.85),GREEN,2.2);ax.text(1.15,-.6,'Velocity direction jumps',ha='center',color=RED,fontsize=15)
        elif i==1:
            ax.text(1.15,-.6,'Velocity at W = 0',ha='center',fontsize=15);arrow(ax,(1.15,0),(1.65,0),GREEN,2.2)
        else:
            velocity=cases['corner'].value(0,1,1);direction=.52*velocity/np.linalg.norm(velocity)
            arrow(ax,np.array([2.,0.])-direction,np.array([2.,0.])+direction,GREEN,2.2);ax.text(1.15,-.6,'Shared nonzero velocity',ha='center',fontsize=15)
    fig.legend(handles=[Line2D([],[],color=BLUE,lw=2,label='Timed curve'),Line2D([],[],color=GRAY,ls='--',label='Geometric polyline'),Line2D([],[],color=GREEN,marker='>',label='Velocity direction'),Line2D([],[],color='black',marker='o',ls='',label='Waypoint')],loc='lower center',bbox_to_anchor=(.5,.035),ncol=2,frameon=False)
    fig.text(.5,.009,'S = start; W = mandatory waypoint; T = goal. Smooth passage keeps W but changes the curve.',ha='center',fontsize=16)
    save(fig,'fig_4_15_path_to_trajectory')
def fig16():
    fig=plt.figure(figsize=(10.2,4));fig.text(.5,.91,'Endpoint values become equations in the coefficients',ha='center',weight='bold',fontsize=22)
    for x,label in [(.02,'1 Specify endpoints'),(.305,'2 Write four equations'),(.735,'3 Solve and evaluate')]:fig.text(x,.855,label,weight='bold',fontsize=19)
    ax=fig.add_axes([0,0,1,1]);ax.axis('off')
    left=['$p(0)=0$','$v(0)=0$','$p(1)=1$','$v(1)=0$'];middle=['$c_0=0$','$c_1=0$','$c_0+c_1+c_2+c_3=1$','$c_1+2c_2+3c_3=0$']
    for y,first,second in zip([.70,.57,.44,.31],left,middle):
        ax.text(.06,y,first,fontsize=21,va='center');ax.text(.315,y,second,fontsize=21,va='center');arrow(ax,(.225,y),(.288,y))
    ax.text(.035,.16,'1 m in 1 s; local time $s=t$.',fontsize=17);ax.text(.33,.16,'$p(s)=c_0+c_1s+c_2s^2+c_3s^3$',fontsize=20);arrow(ax,(.645,.525),(.708,.525))
    chart=fig.add_axes([.754,.38,.217,.32]);ss=np.linspace(0,1,151);chart.plot(ss,3*ss**2-2*ss**3,color=BLUE,lw=2.5);chart.scatter([0,1],[0,1],color='black',s=22,zorder=3);chart.set_xticks([0,1]);chart.set_yticks([0,1]);chart.set_xlabel('$s$',labelpad=1);chart.set_ylabel('$p$ (m)');chart.grid(alpha=.16)
    ax.text(.83,.765,'$(c_0,c_1,c_2,c_3)=(0,0,3,-2)$',ha='center',fontsize=17);ax.text(.865,.16,'$p(s)=3s^2-2s^3$',ha='center',color=BLUE,fontsize=21)
    fig.text(.5,.025,'Legend: arrows = computation order; solid curve = solved position; black dots = endpoints.',ha='center',fontsize=17)
    save(fig,'fig_4_16_boundary_equations')
def fig17():
    fig=plt.figure(figsize=(10.2,5.15));fig.text(.5,.93,'A junction shares derivatives; it does not prescribe a stop',ha='center',weight='bold',fontsize=22)
    route=fig.add_axes([.08,.60,.85,.26]);route.axis('off');route.set_xlim(-.5,5.5);route.set_ylim(-.2,3.9)
    shape=lambda x:.06*x*x+.26*x+.45
    for low,high,color,name in [(0,2,BLUE,'Segment 1'),(2,5,GREEN,'Segment 2')]:
        xx=np.linspace(low,high,80);route.plot(xx,shape(xx),color=color,lw=3,ls='-' if low==0 else '--');mid=(low+high)/2;route.text(mid,shape(mid)+.48,name,color=color,fontsize=19,ha='center')
    for x,label in [(0,'$w_0$'),(2,'$w_1$'),(5,'$w_2$')]:route.scatter([x],[shape(x)],color='black',s=32,zorder=5);route.text(x,shape(x)-.38,label,ha='center',fontsize=19)
    arrow(route,(1.5,shape(2)),(2.5,shape(2)),GRAY)
    fig.text(.05,.54,'Start: $p,v,a,j$ fixed',fontsize=17);fig.text(.33,.54,'Join: $p$ fixed; $v,a,j$ shared',fontsize=17);fig.text(.77,.54,'End: $p,v,a,j$ fixed',fontsize=17)
    fig.text(.5,.465,'One continuity row in $\\mathbf{A}$ for derivative order $r=1,2,3$',ha='center',fontsize=20,weight='bold')
    canvas=fig.add_axes([0,0,1,1]);canvas.axis('off')
    for x,color,expr,label in [(.07,BLUE,'$\\tau_1^{-r}\\mathbf{h}_r(1)$','Columns for $\\mathbf{c}_1$'),(.42,GREEN,'$-\\tau_2^{-r}\\mathbf{h}_r(0)$','Columns for $\\mathbf{c}_2$')]:
        canvas.add_patch(Rectangle((x,.335),.35,.09,facecolor='#EEEEEE' if color==BLUE else 'white',edgecolor='black',ls='-' if color==BLUE else '--',lw=1.6));canvas.text(x+.175,.377,expr,ha='center',va='center',color=color,fontsize=23);canvas.text(x+.175,.28,label,ha='center',color=color,fontsize=18)
    canvas.text(.84,.377,'RHS = 0',ha='center',va='center',fontsize=20);canvas.text(.5,.205,'Two segments: 16 coefficients − 13 equalities = 3 free directions per coordinate.',ha='center',fontsize=18)
    canvas.text(.5,.115,'Waypoints and states → build $\\mathbf{Q},\\mathbf{A},\\mathbf{b}$ → solve KKT → query $\\mathbf{p}(t)$',ha='center',fontsize=19)
    fig.text(.5,.022,'Solid / dashed = segment blocks; dots = waypoints; arrows = matching or computation order.\nThe route is schematic; duration factors match physical time derivatives.',ha='center',fontsize=16)
    save(fig,'fig_4_17_junction_constraints')
def fig18():
    cases=build_cases();tr=cases['through'];w=cases['waypoints'];fig=plt.figure(figsize=(10.2,3.8));fig.text(.5,.89,'Joint optimization chooses the internal derivatives',ha='center',weight='bold',fontsize=22)
    ax=fig.add_axes([.07,.39,.49,.44]);ax.set_xlim(0,14);ax.set_ylim(0,10);ax.set_aspect('equal');ax.set_xlabel('$x$ (m)');ax.set_ylabel('$y$ (m)');ax.grid(alpha=.15)
    for name,key in [('A','box'),('B','upper_box')]:
        low,high=cases[key];ax.add_patch(Rectangle(low[:2],*(high-low)[:2],facecolor='#B2B8BD',edgecolor='black',lw=1))
        if name=='A':ax.text(6.5,3,name,ha='center',va='center',fontsize=18,weight='bold')
        else:ax.annotate('B',xy=(6.2,9.4),xytext=(4.2,9.35),fontsize=18,arrowprops={'arrowstyle':'->','lw':1})
    ax.plot(w[:,0],w[:,1],color=GREEN,ls='--',lw=2);ax.scatter(w[:,0],w[:,1],color='black',s=22,zorder=3);_,p=tr.sample();ax.plot(p[:,0],p[:,1],color=BLUE,lw=2.3)
    for i,a,b in collision_intervals(tr,*cases['upper_box']):
        if b-a>1e-8:
            bad=np.array([tr.value(i,s) for s in np.linspace(a,b,80)]);ax.plot(bad[:,0],bad[:,1],color=RED,lw=3,ls=':');mid=bad[len(bad)//2];ax.plot(mid[0],mid[1],'x',color='black',ms=8,mew=2,zorder=6)
    ax.text(.03,.77,'$z=1$ m',transform=ax.transAxes,fontsize=17)
    for y,label,color,size in [(.77,'Same waypoints and 6 s duration','black',18),(.64,'Stopped: internal $v,a,j$ are zero',GREEN,17),(.49,'Joint: internal derivatives\nare optimized',BLUE,17),(.35,'Snap cost (m²/s⁷):','black',18),(.24,f"{cases['stop'].snap_cost():.0f} → {tr.snap_cost():.2f}",'black',23)]:fig.text(.60,y,label,color=color,fontsize=size)
    fig.legend(handles=[Line2D([],[],color=GREEN,ls='--',label='Stopped route'),Line2D([],[],color=BLUE,label='Joint minimum-snap curve'),Line2D([],[],color=RED,marker='x',ls='none',label='Intersection with B'),Line2D([],[],color='black',marker='o',ls='',label='Waypoint')],loc='lower center',bbox_to_anchor=(.5,.002),ncol=2,frameon=False)
    save(fig,'fig_4_18_minimum_snap_example')
def main():
    for function in [fig15,fig16,fig17,fig18]:function()
    print('Four English Times New Roman diagrams regenerated; editable SVG text and geometry retained.')
if __name__=='__main__':main()
