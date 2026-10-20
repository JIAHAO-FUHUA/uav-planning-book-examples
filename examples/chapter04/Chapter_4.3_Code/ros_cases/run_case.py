#!/usr/bin/env python3
"""ROS Noetic runner. Use its ROS Python interpreter after sourcing ONE workspace.

Ideal command tracking and a static spherical point-cloud model isolate planner
mechanisms. This deliberately does not simulate vehicle dynamics or occlusion.
"""
from pathlib import Path
import argparse,json,math,time,subprocess,os,signal,threading
import numpy as np
import rospy
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from quadrotor_msgs.msg import PositionCommand

def surface_points(boxes,step):
    points=[]
    for box in boxes:
        lo=np.array(box['low']);hi=np.array(box['high'])
        axes=[np.arange(lo[i],hi[i]+step*.25,step) for i in range(3)]
        for axis in range(3):
            other=[i for i in range(3) if i!=axis]
            for side in (lo[axis],hi[axis]):
                for a in axes[other[0]]:
                    for b in axes[other[1]]:
                        x=lo.copy();x[axis]=side;x[other[0]]=a;x[other[1]]=b;points.append(x)
    return np.unique(np.round(points,6),axis=0)

def distance(point,box):
    lo=np.array(box['low']);hi=np.array(box['high']);p=np.array(point)
    outside=np.maximum(np.maximum(lo-p,p-hi),0)
    if np.any(outside):return float(np.linalg.norm(outside))
    return -float(np.min(np.minimum(p-lo,hi-p)))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--planner',choices=['fast','ego'],required=True)
    parser.add_argument('--timeout',type=float,default=60.);parser.add_argument('--scene',default=str(Path(__file__).parent/'scene.json'))
    parser.add_argument('--output',required=True);args=parser.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    scene=json.loads(Path(args.scene).read_text());points=surface_points(scene['boxes'],scene['voxel'])
    log=(out/'roslaunch.log').open('w');launch=Path(__file__).parent/'generated'/f'{args.planner}_book.launch'
    process=subprocess.Popen(['roslaunch',str(launch)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    state={'p':np.array(scene['start']),'v':np.zeros(3),'a':np.zeros(3),'yaw':0.,'commands':0,'last_command':None}
    lock=threading.Lock();trace=[];trajectories=[]
    try:
        import rosgraph
        for attempt in range(100):
            try:rosgraph.Master('/book_probe').getPid();break
            except Exception:time.sleep(.2)
        else:raise RuntimeError('ROS master did not become ready')
        rospy.init_node('book_case_recorder',anonymous=True,disable_signals=True)
        odom_pub=rospy.Publisher('/book/odom',Odometry,queue_size=10)
        cloud_pub=rospy.Publisher('/book/cloud',PointCloud2,queue_size=1)
        global_pub=rospy.Publisher('/map_generator/global_cloud',PointCloud2,queue_size=1,latch=True)
        def command(msg):
            with lock:
                state.update(p=np.array([msg.position.x,msg.position.y,msg.position.z]),v=np.array([msg.velocity.x,msg.velocity.y,msg.velocity.z]),a=np.array([msg.acceleration.x,msg.acceleration.y,msg.acceleration.z]),yaw=msg.yaw,commands=state['commands']+1,last_command=time.monotonic())
        rospy.Subscriber('/book/pos_cmd',PositionCommand,command,queue_size=50)
        if args.planner=='fast':
            from plan_manage.msg import Bspline
            trajectory_topic='/planning/bspline';trajectory_type=Bspline
            goal_pub=rospy.Publisher('/waypoint_generator/waypoints',RosPath,queue_size=1,latch=True)
        else:
            from traj_utils.msg import PolyTraj
            from quadrotor_msgs.msg import GoalSet
            trajectory_topic='/book/trajectory';trajectory_type=PolyTraj
            goal_pub=rospy.Publisher('/goal_with_id',GoalSet,queue_size=1,latch=True)
        rospy.Subscriber(trajectory_topic,trajectory_type,lambda msg:trajectories.append({'received':time.monotonic(),'id':int(msg.traj_id)}),queue_size=100)
        started=time.monotonic();last_cloud=0.;sent=False;arrived=None
        rate=rospy.Rate(100)
        while not rospy.is_shutdown() and time.monotonic()-started<args.timeout:
            now=time.monotonic();elapsed=now-started
            with lock:p=state['p'].copy();v=state['v'].copy();a=state['a'].copy();yaw=state['yaw'];count=state['commands']
            stamp=rospy.Time.now();odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='world';odom.child_frame_id='base_link'
            odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=p
            odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
            odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=v;odom_pub.publish(odom)
            if now-last_cloud>.1:
                visible=points[np.linalg.norm(points-p,axis=1)<=scene['sensing_radius']]
                # Empty clouds are valid sensor outputs; maps may ignore them.
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='world'),visible))
                if not sent:global_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='world'),points))
                last_cloud=now
            if elapsed>3 and not sent and goal_pub.get_num_connections()>0:
                if args.planner=='fast':
                    goal=RosPath();goal.header.frame_id='world';pose=PoseStamped();pose.header=goal.header
                    pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=scene['goal'];pose.pose.orientation.w=1.;goal.poses=[pose]
                else:goal=GoalSet();goal.drone_id=0;goal.goal=scene['goal']
                goal_pub.publish(goal);sent=True
            trace.append([elapsed,*p,*v,*a,count])
            if sent and np.linalg.norm(p-scene['goal'])<.25 and np.linalg.norm(v)<.2:
                if arrived is None:arrived=elapsed
                if elapsed-arrived>1:break
            else:arrived=None
            rate.sleep()
        data=np.array(trace);length=float(np.linalg.norm(np.diff(data[:,1:4],axis=0),axis=1).sum())
        clearance=min(distance(p,box) for p in data[:,1:4] for box in scene['boxes'])
        report={'planner':args.planner,'scene':scene,'scope':'ideal position-command tracking; spherical range sensing; static boxes; no occlusion/controller',
            'goal_sent':sent,'arrived':arrived is not None,'elapsed':float(data[-1,0]),'goal_error':float(np.linalg.norm(data[-1,1:4]-scene['goal'])),
            'executed_length':length,'sampled_minimum_boundary_distance':clearance,'max_recorded_speed':float(np.linalg.norm(data[:,4:7],axis=1).max()),
            'trajectory_messages':len(trajectories),'commands':int(data[-1,-1]),'trace_columns':['elapsed','x','y','z','vx','vy','vz','ax','ay','az','command_count'],
            'trace':data[::2].tolist(),'trajectory_receipts':trajectories}
        (out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('trace','trajectory_receipts','scene')},indent=2))
    finally:
        rospy.signal_shutdown('book case complete')
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        log.close()

if __name__=='__main__':main()
