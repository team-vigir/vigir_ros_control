import rospy
import sys
import linecache
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from copy import deepcopy
import time

class VigirTrajectoryLimits(object):

    def __init__(self, v_max, a_max):
        self.v_max = v_max
        self.a_max = a_max

        self.vel  = np.zeros(6); self.vel[0]  =  5.0; self.vel[1]  = 4.0; self.vel[2]  = 3.0; self.vel[3] = 2.0; self.vel[4]=1.0
        self.acc  = np.zeros(6); self.acc[0]  = 20.0; self.acc[1]  =12.0; self.acc[2]  = 6.0; self.acc[3] = 2.0;
        self.jerk = np.zeros(6); self.jerk[0] = 60.0; self.jerk[1] =24.0; self.jerk[2] = 6.0;


        # Define some segment time limits
        self.dt_acc_min        = self.v_max/self.a_max    # Minimum time to accelerate to max velocity
        self.dt_blend_hump     = 0.015                    # Time to apply non-zero jerk
        self.dt_blend_interval = 2*self.dt_blend_hump     # Minimum time to blend velocity change
        self.dt_vel_min        = self.dt_blend_interval   # Minimum time (seconds) for trajectory segment

    def check_segment_limits(self, spline, dT, joint_name, seg):
        jnt_violation = False

        p_vel  = (spline*self.vel)[:-1]
        p_acc  = (spline*self.acc)[:-2]
        p_jerk = (spline*self.jerk)[:-3]

        dT_new = deepcopy(dT)
        acc_roots = np.roots(p_acc)
        found_extrema = False
        for root in acc_roots:
            if np.fabs(np.imag(root)) < 1e-6: # only concerned with real roots
                t = np.real(root)
                if (t >= 0.0 and t <= dT):
                    # Extrema of velocity in range
                    found_extrema = True
                    v_ext = np.polyval(p_vel,t)
                    v_max = np.fabs(v_ext)
                    #print"      found velocity extrema at t=",t," v_ext=",v_ext

                    if (v_max > self.v_max):
                        jnt_violation = True
                        factor = 1.1*(v_max/self.v_max)
                        #print "     jnt_splines=",spline
                        #print "     vel=",self.vel," p_vel=",p_vel
                        #print "     v[0]=", np.polyval(p_vel,0.0), "v[t]=", np.polyval(p_vel,t), " v[dt]=", np.polyval(p_vel,dT)

                        print "     found extrema at 0 < ",t," < ",dT," v=",v_ext, " v_max =",v_max, " time factor=",factor, " for joint=",joint_name," seg-=",seg
                        dT_new = max([dT_new,deepcopy(dT)*factor])

        if (found_extrema is False):

             # If we didn't find extrema in real time range, then check end points
             v_ext = p_vel[4]
             v_max = np.fabs(v_ext)
             if (v_max > self.v_max):
                 jnt_violation = True
                 factor = 1.1*(v_max/self.v_max)
                 print "        found extrema at t=0  v=",v_ext, " v_max =",v_max, " time factor=",factor, " for joint=",joint_name," seg-=",seg
                 dT_new = deepcopy(max([dT_new,dT*factor]))

             v_ext = np.polyval(p_vel,dT)
             v_max = np.fabs(v_ext)
             if (v_max > self.v_max):
                 jnt_violation = True
                 factor = 1.1*(v_max/self.v_max)
                 print "        found extrema at t=dT  v=",v_ext, " v_max =",v_max, " time factor=",factor, " for joint=",joint_name," seg-=",seg
                 dT_new = deepcopy(max([dT_new,dT*factor]))

        jerk_roots = np.roots(p_jerk)
        found_extrema = False
        for root in jerk_roots:
             #print "jerk_root=",root
             if np.fabs(np.imag(root)) < 1e-6: # only concerned with real roots
                 t = np.real(root)
                 if (t >= 0.0 and t <= dT):
                     # Extrema of velocity in range
                     found_extrema = True
                     a_ext = np.polyval(p_acc,t)
                     a_max = np.fabs(a_ext)
                     #print"     found accelertion extrema at t=",t," a_ext=",a_ext
                     if (a_max > self.a_max):
                         jnt_violation = True
                         factor = 1.1*(a_max/self.a_max)
                         print "        found extrema at 0 < ",t," < ",dT," a=",a_ext, " a_max =",a_max, " time factor=",factor, " for joint=",joint_name," seg-=",seg
                         dT_new = deepcopy(max([dT_new,dT*factor]))

        if (found_extrema is False):
              # If we didn't find extrema in real time range, then check end points
              a_ext = p_acc[3]
              a_max = np.fabs(a_ext)
              #print"       found accelertion extrema at t=",0," a_ext=",a_ext
              if (a_max > self.a_max):
                  jnt_violation = True
                  factor = 1.1*(a_max/self.a_max)
                  print "       found acc extrema at t=0  a=",a_ext, " a_max =",a_max, " time factor=",factor, " for joint=",joint_name," seg-=",seg
                  dT_new = deepcopy(max([dT_new,dT*factor]))

              a_ext = np.polyval(p_vel,dT)
              a_max = np.fabs(a_ext)
              #print"       found accelertion extrema at t= dT a_ext=",a_ext
              if (a_max > self.a_max):
                  jnt_violation = True
                  factor = 1.1*(a_max/self.a_max)
                  print "       found acc extrema at t=dT  a=",a_ext, " a_max =",a_max, " time factor=",factor, " for joint=",joint_name," seg-=",seg
                  dT_new = deepcopy(max([dT_new,dT*factor]))

        return jnt_violation, dT_new


"""
    This function defines a list of quintic splines for each joint assuming the position, velocity, and acceleration is known for each
    knot point given in trajectory.
"""
def define_trajectory_splines(trajectory):
  try:
    num_segments = len(trajectory.points) -1
    num_joints   = len(trajectory.joint_names)
    print "Define trajectory splines with ",num_segments," segments and ",num_joints," joints"
    #print trajectory
    spline_list = [np.zeros((num_segments,6)) for x in xrange(num_joints)]

    # Define the initial time intervals for each segment
    dT = [0 for x in xrange(num_segments)]
    for seg in range(0,num_segments):
        dT[seg] = (trajectory.points[seg+1].time_from_start -trajectory.points[seg].time_from_start).to_sec()

    if (num_segments < 1):
        print "Invalid number of segments =",num_segments," cannot spline!"
        raise Exception("No segments to spline!")

    # Calculate the splines given raw trajectory data
    if (len(trajectory.points[0].accelerations) > 0):
        spline_list = create_quintic_splines(spline_list, dT, trajectory, num_segments, num_joints)
    elif (len(trajectory.points[0].velocities) > 0):
        # cubic
        spline_list = create_cubic_splines(spline_list, dT, trajectory, num_segments, num_joints)
    else:
        # Piecewise linear
        spline_list = create_linear_splines(spline_list, dT, trajectory, num_segments, num_joints)
    return spline_list, dT, num_segments, num_joints
  except Exception as msg:
      print "Exception while defining the trajectory splines ..."
      print "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
      print trajectory.joint_names
      for jnt, joint_name in enumerate(trajectory.joint_names):
          print "----------------------------------------------------------"
          print "Joint ",joint_name," Trajectory "
          prior_time = 0.0
          for pnt, knot in enumerate(trajectory.points):
            dt = (knot.time_from_start.to_sec() - prior_time)
            prior_time = knot.time_from_start.to_sec()
            print "     ",pnt," : ",dt," (",knot.positions[jnt],", ",knot.velocities[jnt],", ",knot.accelerations[jnt],")"

      print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
      print msg
      print PrintException()
      raise Exception(msg)

def create_quintic_splines(spline_list, dT, trajectory, num_segments, num_joints):
    #print "Creating quintic splines for each segment ..."
    for seg in range(num_segments):
        Mi = create_M_segment(dT[seg]) # dT is same for all joints in segment

        for jnt, point in enumerate(trajectory.joint_names):
            # Define the spline per joint and append to spline_list
            x = create_x_vector_segment(seg, jnt, trajectory.points)
            jnt_splines = np.dot(Mi,x)
            jnt_splines.resize(jnt_splines.size/6,6)
            spline_list[jnt][seg] = jnt_splines
    return spline_list

def create_cubic_splines(spline_list, dT, trajectory, num_segments, num_joints):
    #print "Creating cubic splines for each segment  ..."
    for seg in range(num_segments):
        Mi = create_M_cubic_segment(dT[seg]) # dT is same for all joints in segment
        for jnt, point in enumerate(trajectory.joint_names):
            # Define the spline per joint and append to spline_list
            x = create_x_vector_cubic_segment(seg, jnt, trajectory.points)
            jnt_splines = np.dot(Mi,x)
            jnt_splines.resize(jnt_splines.size/4,4) # cubic has 4 unknowns
            spline_list[jnt][seg][2:] = jnt_splines  # Assign to 6 element quintic unknowns

    #print "Return cubic spline list ..."
    #print spline_list
    return spline_list

def create_linear_splines(spline_list, dT, trajectory, num_segments, num_joints):
    #print "Creating linear splines for each segment  ..."
    for seg in range(num_segments):
        for jnt, point in enumerate(trajectory.joint_names):
            # Define the spline per joint and append to spline_list
            spline_list[jnt][seg][5] = trajectory.points[seg].positions[jnt]
            if (dT[seg] > 0.0):
              spline_list[jnt][seg][4] = (trajectory.points[seg+1].positions[jnt] - trajectory.points[seg].positions[jnt])/dT[seg]

    return spline_list


def create_M_state(num_segments, dT):
    #print "Define the Matrix"
    #print dT
    unknowns = 6*num_segments
    M = np.zeros((unknowns, unknowns));

    eqns = 0
    # Use numpy format for polynomial:  p[0] * x**n + p[1] * x**(n-1) + ... + p[n-1]*x + p[n]

    # Initial point q=p[0], dq[0]=ddq=0
    M[0][5] = 1 # q[0]
    M[1][4] = 1 # dq[0]
    M[2][3] = 2 # ddq[0]
    eqns += 3


    for seg in range(0,num_segments-1):
        dT0 = deepcopy(dT[seg])
        # position at end of segment q[1]
        M[3+seg*6][seg*6 + 5] = 1.0
        M[3+seg*6][seg*6 + 4] = deepcopy(dT0)
        M[3+seg*6][seg*6 + 3] = pow(deepcopy(dT0),2)
        M[3+seg*6][seg*6 + 2] = pow(deepcopy(dT0),3)
        M[3+seg*6][seg*6 + 1] = pow(deepcopy(dT0),4)
        M[3+seg*6][seg*6 + 0] = pow(deepcopy(dT0),5)
        eqns  +=1

        # Velocities at end of segment  dq[1]
        M[4+seg*6][seg*6 + 4] = 1.0
        M[4+seg*6][seg*6 + 3] = deepcopy(dT0)*2.0
        M[4+seg*6][seg*6 + 2] = pow(deepcopy(dT0),2)*3.0
        M[4+seg*6][seg*6 + 1] = pow(deepcopy(dT0),3)*4.0
        M[4+seg*6][seg*6 + 0] = pow(deepcopy(dT0),4)*5.0
        eqns  +=1

        # Accelerations at end of segment ddq[1]
        M[5+seg*6][seg*6 + 3] = 2.0
        M[5+seg*6][seg*6 + 2] = deepcopy(dT0)*6.0
        M[5+seg*6][seg*6 + 1] = pow(deepcopy(dT0),2)*12.0
        M[5+seg*6][seg*6 + 0] = pow(deepcopy(dT0),3)*20.0
        eqns  +=1

        seg1 = seg+1
        # position at start of next segment
        M[6+seg*6][seg1*6 + 5] = 1.0
        eqns  +=1

        # velocity at start of next segment
        M[7+seg*6][seg1*6 + 4] = 1.0
        eqns  +=1

        # Accelerations at start of next segment
        M[8+seg*6][seg1*6 + 3] = 2.0
        eqns  +=1

    # Final knot point
    seg = num_segments -1
    dT0 = deepcopy(dT[seg])

    #print 'dT[',seg,']=',dT0

    # position at end of segment
    M[3+seg*6][seg*6 + 5] = 1.0
    M[3+seg*6][seg*6 + 4] = deepcopy(dT0)
    M[3+seg*6][seg*6 + 3] = pow(deepcopy(dT0),2)
    M[3+seg*6][seg*6 + 2] = pow(deepcopy(dT0),3)
    M[3+seg*6][seg*6 + 1] = pow(deepcopy(dT0),4)
    M[3+seg*6][seg*6 + 0] = pow(deepcopy(dT0),5)
    eqns  +=1

    # velocity at terminal knot point dq[1]
    M[4+seg*6][seg*6 + 4] = 1.0
    M[4+seg*6][seg*6 + 3] = deepcopy(dT0)*2.0
    M[4+seg*6][seg*6 + 2] = pow(deepcopy(dT0),2)*3.0
    M[4+seg*6][seg*6 + 1] = pow(deepcopy(dT0),3)*4.0
    M[4+seg*6][seg*6 + 0] = pow(deepcopy(dT0),4)*5.0
    eqns  +=1

    # acceleration at terminal knot point  ddq[1]
    M[5+seg*6][seg*6 + 3] = 2.0
    M[5+seg*6][seg*6 + 2] = deepcopy(dT0)*6.0
    M[5+seg*6][seg*6 + 1] = pow(deepcopy(dT0),2)*12.0
    M[5+seg*6][seg*6 + 0] = pow(deepcopy(dT0),3)*20.0
    eqns  +=1

    rank = np.linalg.matrix_rank(M)

    if (unknowns != eqns) or (rank != unknowns):
        condition_M = np.linalg.cond(M)
        np.set_printoptions(precision=3, linewidth=180)
        print "M="
        print M
        print 'Condition number = ', condition_M
        print "Invalid number of equations = ",eqns," and unknowns=",unknowns,"  rank(M)=",rank
        raise Exception("rank"+str(rank)+" condition="+str(condition_M))

    # Inverted matrix
    Mi = np.linalg.inv(M)
    #print "return the inverted matrix"
    return Mi

"""
    This function creates the x-vector used to solve M*p = x, where x contains the state at each knot points at
    beginning and end of each segment, where state is position, velocity, and acceleration
"""
def create_x_vector_state(num_segments, jnt, knot_points):
    x = np.zeros(6*num_segments);

    # Initial point
    x[0] = deepcopy(knot_points[0].positions[jnt])
    x[1] = deepcopy(knot_points[0].velocities[jnt])
    x[2] = deepcopy(knot_points[0].accelerations[jnt])

    # Interior knot points
    for seg in range(0,num_segments-1):
        # position, velocity, acceleration at end of segment
        x[3+seg*6] = deepcopy(knot_points[seg+1].positions[jnt])
        x[4+seg*6] = deepcopy(knot_points[seg+1].velocities[jnt])
        x[5+seg*6] = deepcopy(knot_points[seg+1].accelerations[jnt])

        x[6+seg*6] = deepcopy(knot_points[seg+1].positions[jnt])
        x[7+seg*6] = deepcopy(knot_points[seg+1].velocities[jnt])
        x[8+seg*6] = deepcopy(knot_points[seg+1].accelerations[jnt])

    # Terminal point
    seg = num_segments -1
    x[3+seg*6] = deepcopy(knot_points[seg+1].positions[jnt])
    x[4+seg*6] = deepcopy(knot_points[seg+1].velocities[jnt])
    x[5+seg*6] = deepcopy(knot_points[seg+1].accelerations[jnt])

    return x

def create_M_segment( dT0):
    unknowns = 6
    M = np.zeros((unknowns, unknowns));

    # Use numpy format for polynomial:  p[0] * x**n + p[1] * x**(n-1) + ... + p[n-1]*x + p[n]

    # Initial point q=p[0], dq[0]=ddq=0
    M[0][5] = 1 # q[0]
    M[1][4] = 1 # dq[0]
    M[2][3] = 2 # ddq[0]

    # Position at terminal q[1]
    M[3][ 5] = 1.0
    M[3][ 4] = deepcopy(dT0)
    M[3][ 3] = pow(deepcopy(dT0),2)
    M[3][ 2] = pow(deepcopy(dT0),3)
    M[3][ 1] = pow(deepcopy(dT0),4)
    M[3][ 0] = pow(deepcopy(dT0),5)

    # velocity at terminal dq[1]
    M[4][4] = 1.0
    M[4][3] = deepcopy(dT0)*2.0
    M[4][2] = pow(deepcopy(dT0),2)*3.0
    M[4][1] = pow(deepcopy(dT0),3)*4.0
    M[4][0] = pow(deepcopy(dT0),4)*5.0

    # acceleration at terminal knot point  ddq[1]
    M[5][3] = 2.0
    M[5][2] = deepcopy(dT0)*6.0
    M[5][1] = pow(deepcopy(dT0),2)*12.0
    M[5][0] = pow(deepcopy(dT0),3)*20.0

    # Inverted matrix
    Mi = np.linalg.inv(M)
    return Mi

def create_x_vector_segment(seg, jnt, knot_points):
    x = np.zeros(6);

    # Initial point
    x[0] = deepcopy(knot_points[seg].positions[jnt])
    x[1] = deepcopy(knot_points[seg].velocities[jnt])
    x[2] = deepcopy(knot_points[seg].accelerations[jnt])
    x[3] = deepcopy(knot_points[seg+1].positions[jnt])
    x[4] = deepcopy(knot_points[seg+1].velocities[jnt])
    x[5] = deepcopy(knot_points[seg+1].accelerations[jnt])

    return x

def create_M_cubic_segment( dT0):
    unknowns = 4
    M = np.zeros((unknowns, unknowns));

    # Use numpy format for polynomial:  p[0] * x**n + p[1] * x**(n-1) + ... + p[n-1]*x + p[n]

    # Initial point q=p[0], dq[0]=ddq=0
    M[0][3] = 1 # q[0]
    M[1][2] = 1 # dq[0]

    # Position at terminal q[1]
    M[2][ 3] = 1.0
    M[2][ 2] = deepcopy(dT0)
    M[2][ 1] = pow(deepcopy(dT0),2)
    M[2][ 0] = pow(deepcopy(dT0),3)

    # velocity at terminal dq[1]
    M[3][2] = 1.0
    M[3][1] = deepcopy(dT0)*2.0
    M[3][0] = pow(deepcopy(dT0),2)*3.0

    # Inverted matrix
    Mi = np.linalg.inv(M)
    return Mi

def create_x_vector_cubic_segment(seg, jnt, knot_points):
    x = np.zeros(4);

    # Initial point
    x[0] = deepcopy(knot_points[seg].positions[jnt])
    x[1] = deepcopy(knot_points[seg].velocities[jnt])
    x[2] = deepcopy(knot_points[seg+1].positions[jnt])
    x[3] = deepcopy(knot_points[seg+1].velocities[jnt])

    return x

def PrintException():
    exc_type, exc_obj, tb = sys.exc_info()
    f = tb.tb_frame
    lineno = tb.tb_lineno
    filename = f.f_code.co_filename
    linecache.checkcache(filename)
    line = linecache.getline(filename, lineno, f.f_globals)
    print 'EXCEPTION IN ({}, LINE {} "{}"): {}'.format(filename, lineno, line.strip(), exc_obj)



