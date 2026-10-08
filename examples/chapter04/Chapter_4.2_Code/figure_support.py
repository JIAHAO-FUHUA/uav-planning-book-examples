"""Publication figure typography and editable architecture/demo layouts."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
from matplotlib.transforms import Bbox
from PIL import Image
from print_style import make_print_ready

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'visual_figures'
OUT.mkdir(exist_ok=True)

def configure_fonts():
    # Register the real Times New Roman faces; do not silently substitute a font.
    windows=Path('C:/Windows/Fonts')
    for name in ['times.ttf','timesbd.ttf','timesi.ttf','timesbi.ttf']:
        f=windows/name
        if f.exists():font_manager.fontManager.addfont(str(f))
    font_manager.findfont('Times New Roman',fallback_to_default=False)
    plt.rcParams.update({'font.family':'Times New Roman',
        'font.serif':['Times New Roman'], 'svg.fonttype':'none', 'pdf.fonttype':42,
        'mathtext.fontset':'custom','mathtext.rm':'Times New Roman',
        'mathtext.it':'Times New Roman:italic','mathtext.bf':'Times New Roman:bold',
        'mathtext.bfit':'Times New Roman:bold:italic','mathtext.sf':'Times New Roman',
        'mathtext.tt':'Times New Roman','mathtext.cal':'Times New Roman:italic',
        'mathtext.fallback':'stix'})

def save(fig,name):
    make_print_ready(fig)
    fig.canvas.draw()
    bound=fig.get_tightbbox(fig.canvas.get_renderer())
    fixed=Bbox.from_extents(min(0,bound.x0),min(0,bound.y0),
        max(fig.get_figwidth(),bound.x1),max(fig.get_figheight(),bound.y1))
    for suffix in ['png','svg']:
        fig.savefig(OUT/(name+'.'+suffix),dpi=300,facecolor='white',
                    bbox_inches=fixed,pad_inches=.08)
    plt.close(fig)

def architecture():
    fig,ax=plt.subplots(figsize=(11,4.25))
    fig.subplots_adjust(left=.015,right=.985,bottom=.11,top=.985)
    ax.set(xlim=(0,11),ylim=(0,4.25));ax.axis('off')
    def box(x,y,w,h,label):
        ax.add_patch(Rectangle((x,y),w,h,facecolor='white',edgecolor='#555555',lw=1.2))
        ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=18)
    def arrow(a,b):
        ax.annotate('',xy=b,xytext=a,arrowprops={'arrowstyle':'-|>','lw':1.1,'color':'#555555'})
    box(2.75,3.42,3.2,.60,'Goals and flight constraints')
    ax.add_patch(Rectangle((2.3,1.63),4.0,1.43,fc='white',ec='#555555',lw=1.2))
    ax.text(4.3,2.79,'Path and trajectory planning',ha='center',va='center',fontsize=18)
    box(2.5,1.83,1.62,.66,'Path search')
    box(4.62,1.83,1.50,.66,'Trajectory\ngeneration')
    box(.08,1.88,1.75,.85,'State estimate\nand map')
    box(6.75,1.88,1.48,.85,'Tracking\ncontroller')
    box(8.84,1.88,1.95,.85,'UAV and\nenvironment')
    arrow((4.35,3.42),(4.35,3.06))
    arrow((1.83,2.31),(2.3,2.31));arrow((4.12,2.16),(4.62,2.16))
    arrow((6.3,2.31),(6.75,2.31));arrow((8.23,2.31),(8.84,2.31))
    ax.plot([9.82,9.82,.95,.95],[1.88,.78,.78,1.37],c='#555555',lw=1.1)
    arrow((.95,1.37),(.95,1.88));arrow((4.3,.78),(4.3,1.63))
    ax.text(5.04,1.14,'Map update or replanning',ha='left',fontsize=16.5)
    ax.text(5.4,.48,'Sensor observations and execution feedback',ha='center',fontsize=18)
    ax.text(5.4,.05,'Legend: boxes = processing stages; arrows = information flow',ha='center',fontsize=16.5)
    save(fig,'fig_4_1_architecture')

def source_demonstrations():
    # Preserve the two full source frames and their aspect ratios.
    fig = plt.figure(figsize=(11.6, 4.7))
    layout = fig.add_gridspec(
        3, 2, height_ratios=[.55, 3.12, .62],
        left=.01, right=.99, bottom=.01, top=.99,
        hspace=.04, wspace=.05)
    panels = [
        ('(a) EGO-Planner-v2\nSimulated trajectories among obstacles',
         'ego_frame_160s.png',
         'Legend: curves = planned trajectories;\nvertical shapes = obstacles'),
        ('(b) Agile Autonomy\nLearned flight around a tree',
         'agile_frame_37.png',
         'Legend: repeated UAVs = successive positions;\ntree = obstacle')]
    for column, (title, filename, legend) in enumerate(panels):
        title_ax = fig.add_subplot(layout[0, column])
        title_ax.axis('off')
        title_ax.text(.5, .5, title, ha='center', va='center',
                      fontsize=22, fontweight='bold', linespacing=1.05)
        picture_ax = fig.add_subplot(layout[1, column])
        with Image.open(ROOT / 'source_frames' / filename) as frame:
            picture_ax.imshow(frame.copy(), aspect='equal')
        picture_ax.axis('off')
        legend_ax = fig.add_subplot(layout[2, column])
        legend_ax.axis('off')
        legend_ax.text(.5, .5, legend, ha='center', va='center',
                       fontsize=20, linespacing=1.1)
    save(fig, 'fig_4_3_demonstrations')

if __name__=='__main__':
    configure_fonts();architecture();source_demonstrations()
