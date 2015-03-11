import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from copy import deepcopy
import time
from   vigir_trajectory_utility import VigirTrajectoryLimits
import vigir_trajectory_utility  as traj_utility

"""
Generic smoothing code takes a JointTrajectory.msg as input, and recalculates the velocity and acceleration
to yeild a piecewise c3 smooth cubic spline trajectory.  The trajectory is assumed to start and end at rest.
"""
class VigirTrajectoryC3Smoother(object):
    def __init__(self, joint_limits,time_limit = 1.0, max_iterations=50):

        # Store the limits used to smooth each joint as a map from joint name to simple limits structure
        # For now we don't handle time factor, and dynamic reparameterization
        self.joint_limits   = joint_limits
        self.time_limit     = time_limit
        self.max_iterations = max_iterations

    def shutdown():
        print "Shutting down the VigirTrajectoryC3Smoother!"

    def reparameterize(self, old_trajectory):
      try:
        #print "Reparameterize using VigirTrajectoryCubicSmoother"
        count = 0
        elapsed = 0.0
        violation = True # Assume we violate a limit to start
        continue_flag = True

        start_time = time.clock()

        new_trajectory = deepcopy(old_trajectory)
        ##print "vvvvvvvvvvvvvvvvvvvv Original Data vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
        ##for jnt, joint_name in enumerate(new_trajectory.joint_names):
        ##    print "----------------------------------------------------------"
        ##    print "Joint ",joint_name," Trajectory "
        ##    prior_time = 0.0
        ##    for pnt, knot in enumerate(new_trajectory.points):
        ##      dt = (knot.time_from_start.to_sec() - prior_time)
        ##      prior_time = knot.time_from_start.to_sec()
        ##      print "     ",pnt," : ",dt," (",knot.positions[jnt],", ",knot.velocities[jnt],", ",knot.accelerations[jnt],")"
        ##
        ##print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"

        num_segments = len(old_trajectory.points) -1
        if (num_segments < 2):
            print "Invalid number of segments =",num_segments," cannot smooth!"
            return new_trajectory

        if (num_segments > 40):
            print "High number of segments =",num_segments," ( > 40) will not smooth!"
            return new_trajectory

        # Double check the time parameterization
        dT_new = self.time_refactor(new_trajectory, num_segments)

        new_trajectory.points = [deepcopy(old_trajectory.points[0])] + new_trajectory.points + [deepcopy(old_trajectory.points[-1])]
        new_trajectory.points[0].positions     = np.asarray(new_trajectory.points[0].positions    )
        new_trajectory.points[0].velocities    = np.asarray(new_trajectory.points[0].velocities   )
        new_trajectory.points[0].accelerations = np.asarray(new_trajectory.points[0].accelerations)
        new_trajectory.points[-1].positions     = np.asarray(new_trajectory.points[-1].positions    )
        new_trajectory.points[-1].velocities    = np.asarray(new_trajectory.points[-1].velocities   )
        new_trajectory.points[-1].accelerations = np.asarray(new_trajectory.points[-1].accelerations)

        dT_new = [deepcopy(dT_new[0])*0.25] + dT_new + [deepcopy(dT_new[-1])*0.25]
        dT_new[1]  *= 0.75
        dT_new[-2] *= 0.75
        num_segments +=2

        # Begin time re-parameterization loop
        Mi = []
        dT = []
        try:
          while violation and continue_flag:
            violation = False # reset the flag
            dT = deepcopy(dT_new)
            # Get spline parameters for for the given trajectory positions and current time intervals
            # Solve M*p = x ---> p = M^(-1)*x for each joint

            # M matrix depends only on the current time intervals
            #-----------------------------
            Mi = self.create_matrix(num_segments, dT)

            # Solve for each joint
            for jnt, joint_name in enumerate(new_trajectory.joint_names):
                x = self.create_x_vector(num_segments, jnt, new_trajectory.points)
                splines = np.dot(Mi,x)

                splines.resize(splines.size/4,4)

                # Check limits for this joint and update dT as needed
                jnt_violation, dT_new = self.check_limits(dT, splines, dT_new, joint_name)

                ##print "jnt_violation=",jnt_violation
                ##print "new dT=",dT_new
                ##print "------------------------------------------------------"

                # We will continue process all joints under the assumptation that defining and inverting M is the single most expensive operation
                violation = violation or jnt_violation

            if (violation):
                print "Limit violation - continue smoothing"
                violation = False # just do one iteration for now

            count += 1
            end_time = time.clock()
            elapsed = end_time - start_time
            if (count > self.max_iterations) or (elapsed > self.time_limit):
                print "Ran out of iterations or time : count=",count,"  elapsed time =",elapsed
                continue_flag = False
        except Exception as msg:
            print "Exception solving c3 matrix"
            print msg
            traj_utility.PrintException()
            raise(msg)

        if (violation):
            # Update the time and Mi matrix
            dT = deepcopy(dT_new)
            Mi = self.create_matrix(num_segments, dT)

        # Recalculate the spline parameters and assign the joint values
        for jnt, joint in enumerate(new_trajectory.joint_names):
            x = self.create_x_vector(num_segments, jnt, new_trajectory.points)
            splines = np.dot(Mi,x)
            splines.resize(splines.size/4,4)
            ##
            ##np.set_printoptions(precision=3, linewidth=180)
            ##
            ##if (jnt <1):
            ##    for ndx,xx in enumerate(x):
            ##        print "     x[",ndx,"]=",xx
            ##    print " spline[0]=",splines[0]
            ##

            for seg in range(0,num_segments):
                # Interior point data
                new_trajectory.points[seg].positions[jnt]     = np.asscalar(splines[seg][3])
                new_trajectory.points[seg].velocities[jnt]    = np.asscalar(splines[seg][2])
                new_trajectory.points[seg].accelerations[jnt] = 2.0*np.asscalar(splines[seg][1])

            # Terminal condition
            new_trajectory.points[num_segments].velocities[jnt]    = 0.0
            new_trajectory.points[num_segments].accelerations[jnt] = 0.0

        ##print "p0=",new_trajectory.points[0].positions[0],", ",new_trajectory.points[0].velocities[0]
        ##print "p1=",new_trajectory.points[-1].positions[0],", ",new_trajectory.points[-1].velocities[0]

        # Update the timing of the trajectory
        for seg in range(0,num_segments):
            new_trajectory.points[seg+1].time_from_start  = new_trajectory.points[seg].time_from_start + rospy.Duration(dT[seg])

        end_time = time.clock()
        elapsed = end_time - start_time
        print "Total elapsed time = ",elapsed, " to smooth ",num_segments," segments"
        ##for jnt, joint_name in enumerate(new_trajectory.joint_names):
        ##    print "----------------------------------------------------------"
        ##    print "Joint ",joint_name," Trajectory "
        ##    prior_time = 0.0
        ##    for pnt, knot in enumerate(new_trajectory.points):
        ##      dt = (knot.time_from_start.to_sec() - prior_time)
        ##      prior_time = knot.time_from_start.to_sec()
        ##      print "     ",pnt," : ",dt," (",knot.positions[jnt],", ",knot.velocities[jnt],", ",knot.accelerations[jnt],")"
        ##
        ##print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"

        return new_trajectory
      except Exception as msg:
        print "Exception during trajectory reparameterization"
        print msg

        print "Force piecewise linear commands ..."
        new_trajectory = deepcopy(old_trajectory)
        for point in new_trajectory.points:
            point.velocities    = ()
            point.accelerations = ()

        return new_trajectory

    """
        This function assumes piecewise linear segments, and determines the appropriate time intervals and segmentation plan to
        generate c3 smooth quintic splines.  If limits are violated, then the time interval is increase by a proportional factor
    """
    def time_refactor(self, new_trajectory, num_segments):
      try:

        ##print "Re-do time parameterization for c3 smoothing ..."
        dT_new = [ 0.0 for x in xrange(num_segments)]

        # Set the linear segment to the initial time duration
        for seg in range(0,num_segments):
            delta = new_trajectory.points[seg+1].time_from_start - new_trajectory.points[seg].time_from_start
            dT_new[seg] = deepcopy(delta.to_sec())

            # Convert to array for reassignment
            new_trajectory.points[seg].positions     = np.asarray(new_trajectory.points[seg].positions)
            new_trajectory.points[seg].velocities    = np.asarray(new_trajectory.points[seg].velocities)
            new_trajectory.points[seg].accelerations = np.asarray(new_trajectory.points[seg].accelerations)

        # Convert terminal point to array for reassignment
        new_trajectory.points[-1].positions     = np.asarray(new_trajectory.points[-1].positions)
        new_trajectory.points[-1].velocities    = np.asarray(new_trajectory.points[-1].velocities)
        new_trajectory.points[-1].accelerations = np.asarray(new_trajectory.points[-1].accelerations)

        #print "Original dT=",dT_new

        # "Check absolute minimums and average velocity ..."
        for jnt, joint_name in enumerate(new_trajectory.joint_names):
            limits = self.joint_limits[joint_name]

            for seg in range(0,num_segments):
                if (dT_new[seg] < limits.dt_blend_interval):
                    # Absolute minimum time we will consider with zero splits
                    print "Seg=",seg,"  update minimal interval=",limits.dt_blend_interval
                    dT_new[seg] = deepcopy(limits.dt_vel_min)

            # Per joint checks on average velocity
            for seg in range(0,num_segments):
                Dq  = new_trajectory.points[seg+1].positions[jnt] - new_trajectory.points[seg].positions[jnt]

                dt_min = np.fabs(Dq/limits.v_max)
                if (dt_min > dT_new[seg]):
                    print "velocity limit current[",seg,"] =",limits.v_max,"  Dq=",Dq," dt_min=",dt_min
                    dT_new[seg]    = deepcopy(dt_min)

        #print "Post velocity check dT=",dT_new

        # After all the joints have checked the average velocity, recalculate the point velocities and check average accelerations
        #print "Calculate average acceleration for each segment ..."
        time_changed = False
        for jnt, joint_name in enumerate(new_trajectory.joint_names):
            limits = self.joint_limits[joint_name]
            #print jnt," ",joint_name

            # Per joint checks on average acceleration after all timing limits applied
            Dq01  = new_trajectory.points[1].positions[jnt]   - new_trajectory.points[0].positions[jnt]
            dq01 = Dq01/dT_new[0]
            for seg in range(1,num_segments):
                #print "seg=",seg," jnt=",jnt
                #print "    dT_new*=",dT_new
                #print "    reseg=",resegment_plan

                # Recalculate the trailing velocity, and update the acceleration values
                Dq12  = new_trajectory.points[seg+1].positions[jnt] - new_trajectory.points[seg].positions[jnt]
                dq12 = Dq01/dT_new[seg]

                #dq   = (dq12+dq01)/2.0
                dq = (dq12*dT_new[seg-1] + dq01*dT_new[seg])/(dT_new[seg-1]+dT_new[seg]) # weight by swapped time duration
                                                                                         #   more segment time implies longer to accelerate to that speed
                                                                                         #   so give other velocity more weight

                new_trajectory.points[seg].velocities[jnt] = deepcopy(dq)

                # Acceleration
                Ddq = dq - new_trajectory.points[seg-1].velocities[jnt]

                dt_min = np.fabs(Ddq/limits.a_max)
                if (dt_min > dT_new[seg-1]):
                    print "Average acceleration limit found for ", jnt, " segment=",seg
                    time_changed = True
                    dT_new[seg-1] = deepcopy(dt_min)

                # For the next section
                Dq01 = deepcopy(Dq12)
                dq01 = deepcopy(dq12)
        return dT_new
      except Exception as msg:
        print "Exception during create_two_segment_transition reparameterization"

        print msg
        traj_utility.PrintException()
        raise (msg)


    def create_matrix(self, num_segments, dT):
      try:
        #start_time = time.clock()
        unknowns = 4*num_segments
        M = np.zeros((unknowns, unknowns));

        ##print "Unknowns = ",unknowns, "  from ",num_segments," segments"
        eqns = 0
        # Use numpy format for polynomial:  p[0] * x**n + p[1] * x**(n-1) + ... + p[n-1]*x + p[n]

        # Initial point q=p[0], dq=ddq=0
        M[0][3] = 1 # q[0]
        M[1][2] = 1 # dq[0]
        M[2][1] = 2 # ddq[0]
        eqns += 3

        # First added point for initial start
        seg = 0
        seg1 = seg+1
        dT0 = deepcopy(dT[seg])
        # continuous position at end of segment (unknown)
        M[3+seg*4][seg*4 + 3] = 1.0
        M[3+seg*4][seg*4 + 2] = deepcopy(dT0)
        M[3+seg*4][seg*4 + 1] = pow(deepcopy(dT0),2)
        M[3+seg*4][seg*4 + 0] = pow(deepcopy(dT0),3)
        M[3+seg*4][seg1*4 + 3] = -1.0
        eqns  +=1

        # Continuous velocity at interior knot point  dq[0] - dq[1] = 0
        M[4+seg*4][seg*4  + 2] = 1.0
        M[4+seg*4][seg*4  + 1] = deepcopy(dT0)*2.0
        M[4+seg*4][seg*4  + 0] = pow(deepcopy(dT0),2)*3.0
        M[4+seg*4][seg1*4 + 2] = -1.0
        eqns  +=1

        # Continuous acceleration at interior knot point  ddq[0] - ddq[1] = 0
        M[5+seg*4][seg*4  + 1] = 2.0
        M[5+seg*4][seg*4  + 0] = deepcopy(dT0)*6.0
        M[5+seg*4][seg1*4 + 1] = -2.0
        eqns  +=1

        for seg in range(1,num_segments-2):
            dT0 = deepcopy(dT[seg])

            #print "seg=",seg," dT0=",dT0

            # position at end of segment
            M[6+(seg-1)*4][seg*4 + 3] = 1.0
            M[6+(seg-1)*4][seg*4 + 2] = deepcopy(dT0)
            M[6+(seg-1)*4][seg*4 + 1] = pow(deepcopy(dT0),2)
            M[6+(seg-1)*4][seg*4 + 0] = pow(deepcopy(dT0),3)
            eqns  +=1

            seg1 = seg+1
            # position at start of next segment
            M[7+(seg-1)*4][seg1*4 + 3] = 1.0
            eqns  +=1

            # Continuous velocity at interior knot point  dq[0] - dq[1] = 0
            M[8+(seg-1)*4][seg*4  + 2] = 1.0
            M[8+(seg-1)*4][seg*4  + 1] = deepcopy(dT0)*2.0
            M[8+(seg-1)*4][seg*4  + 0] = pow(deepcopy(dT0),2)*3.0
            M[8+(seg-1)*4][seg1*4 + 2] = -1.0
            eqns  +=1

            # Continuous acceleration at interior knot point  ddq[0] - ddq[1] = 0
            M[9+(seg-1)*4][seg*4  + 1] = 2.0
            M[9+(seg-1)*4][seg*4  + 0] = deepcopy(dT0)*6.0
            M[9+(seg-1)*4][seg1*4 + 1] = -2.0
            eqns  +=1

        # Penultimate knot point was added
        # First added point for initial start
        seg = num_segments - 2
        seg1 = seg+1
        dT0 = deepcopy(dT[seg])

        # continuous position at end of segment (unknown)
        M[6+(seg-1)*4][seg*4 + 3] = 1.0
        M[6+(seg-1)*4][seg*4 + 2] = deepcopy(dT0)
        M[6+(seg-1)*4][seg*4 + 1] = pow(deepcopy(dT0),2)
        M[6+(seg-1)*4][seg*4 + 0] = pow(deepcopy(dT0),3)
        M[6+(seg-1)*4][seg1*4 + 3] = -1.0
        eqns  +=1

        # Continuous velocity at interior knot point  dq[0] - dq[1] = 0
        M[7+(seg-1)*4][seg*4  + 2] = 1.0
        M[7+(seg-1)*4][seg*4  + 1] = deepcopy(dT0)*2.0
        M[7+(seg-1)*4][seg*4  + 0] = pow(deepcopy(dT0),2)*3.0
        M[7+(seg-1)*4][seg1*4 + 2] = -1.0
        eqns  +=1

        # Continuous acceleration at interior knot point  ddq[0] - ddq[1] = 0
        M[8+(seg-1)*4][seg*4  + 1] = 2.0
        M[8+(seg-1)*4][seg*4  + 0] = deepcopy(dT0)*6.0
        M[8+(seg-1)*4][seg1*4 + 1] = -2.0
        eqns +=1

        # Final knot point
        dT0 = deepcopy(dT[seg1])
        #print 'dT[',seg,']=',dT0

        # position at end of segment
        M[9+(seg-1)*4][seg1*4 + 3] = 1.0
        M[9+(seg-1)*4][seg1*4 + 2] = deepcopy(dT0)
        M[9+(seg-1)*4][seg1*4 + 1] = pow(deepcopy(dT0),2)
        M[9+(seg-1)*4][seg1*4 + 0] = pow(deepcopy(dT0),3)
        eqns  +=1

        # velocity at terminal knot point dq[1] = 0
        M[10+(seg-1)*4][seg1*4 + 2] = 1.0
        M[10+(seg-1)*4][seg1*4 + 1] = deepcopy(dT0)*2.0
        M[10+(seg-1)*4][seg1*4 + 0] = pow(deepcopy(dT0),2)*3.0
        eqns  +=1

        # acceleration at terminal knot point  ddq[1] = 0
        M[11+(seg-1)*4][seg1*4 + 1] = 2.0
        M[11+(seg-1)*4][seg1*4 + 0] = deepcopy(dT0)*6.0
        eqns  +=1

        rank = np.linalg.matrix_rank(M)
        ##print "Rank=",rank

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

        return Mi
      except Exception as msg:
          print "Exception during create_matrix for c3 smoother"
          print "     ",msg
          traj_utility.PrintException()
          raise (msg)

    """
        This function creates the x-vector used to solve M*p = x, where x contains the state or changes in state at interior points
    """
    def create_x_vector(self, num_segments, jnt, knot_points):
        x = np.zeros(4*num_segments);

        # Initial point
        x[0] = deepcopy(knot_points[0].positions[jnt])
        x[1] = deepcopy(knot_points[0].velocities[jnt])
        x[2] = deepcopy(knot_points[0].accelerations[jnt])

        #x[3],x[4],x[5] have continuity at startup

        # Interior knot points
        for seg in range(1,num_segments-2):
            # position at end of segment
            x[6+(seg-1)*4] = deepcopy(knot_points[seg+1].positions[jnt])

            # position at start of next segment
            x[7+(seg-1)*4] = deepcopy(knot_points[seg+1].positions[jnt])

            # Continuous velocity, acceleration, jerk, and snap at interior points uses zero by construction of x vector

        # there is continuity at penultimate knot point

        # Terminal point
        seg = num_segments -2
        x[9+(seg-1)*4] = deepcopy(knot_points[num_segments].positions[jnt])
        # velocity and acceleration = 0 at terminal point

        return x

    """
        This code evaluates the splines at each segment to determine maximum velocity and acceleration within the segment,
        then checks against limits.  If limits are violated, then the time interval is increase by a proportional factor
    """
    def check_limits(self, dT, splines, dT_new, joint_name):
      try:
        vel  = np.zeros(4);   vel[0] = 3.0; vel[1] = 2.0; vel[2]=1.0
        acc  = np.zeros(4);   acc[0] = 6.0; acc[1] = 2.0;
        jerk = np.zeros(4);  jerk[0] = 6.0;

        jnt_violation = False
        for seg in range(0,len(dT)):

            dt = dT[seg]
            p_vel  = (splines[seg]*vel)[:-1]
            p_acc  = (splines[seg]*acc)[:-2]
            p_jerk = (splines[seg]*acc)[:-3]

            acc_roots = np.roots(p_acc)
            found_extrema = False
            for root in acc_roots:
                if np.fabs(np.imag(root)) < 1e-6: # only concerned with real roots
                    t = np.real(root)
                    if (t >= 0.0 and t <= dt):
                        # Extrema of velocity in range
                        found_extrema = True
                        v_ext = np.polyval(p_vel,t)
                        v_max = np.fabs(v_ext)
                        #print"found velocity extrema at t=",t," a_ext=",v_ext

                        if (v_max > self.joint_limits[joint_name].v_max):
                            jnt_violation = True
                            factor = 1.1*(v_max/self.joint_limits[joint_name].v_max)
                            print "seg=",seg
                            print "splines[seg]=",splines[seg]
                            print " vel=",vel
                            print "p_vel=",p_vel
                            print "v[0]=", np.polyval(p_vel,0.0)
                            print "v[t]=", np.polyval(p_vel,t)
                            print "v[dt]=", np.polyval(p_vel,dt)

                            print "found extrema at 0 < ",t," < ",dt," v=",v_ext, " v_max =",v_max, " time factor=",factor
                            dT_new[seg] = deepcopy(max([dT_new[seg],dT[seg]*factor]))

            if (found_extrema is False):

                 # If we didn't find extrema in real time range, then check end points
                 v_ext = p_vel[2]
                 v_max = np.fabs(v_ext)
                 #print"found velocity extrema at t=",0," a_ext=",v_ext
                 if (v_max > self.joint_limits[joint_name].v_max):
                     jnt_violation = True
                     factor = 1.1*(v_max/self.joint_limits[joint_name].v_max)
                     print "found extrema at t=0  v=",v_ext, " v_max =",v_max, " time factor=",factor
                     dT_new[seg] = deepcopy(max([dT_new[seg],dT[seg]*factor]))

                 v_ext = np.polyval(p_vel,dt)
                 v_max = np.fabs(v_ext)
                 #print"found velocity extrema at t=",dt," a_ext=",v_ext
                 if (v_max > self.joint_limits[joint_name].v_max):
                     jnt_violation = True
                     factor = 1.1*(v_max/self.joint_limits[joint_name].v_max)
                     print "found extrema at t=dt  v=",v_ext, " v_max =",v_max, " time factor=",factor
                     dT_new[seg] = deepcopy(max([dT_new[seg],dT[seg]*factor]))


            jerk_roots = np.roots(p_jerk)
            found_extrema = False
            for root in jerk_roots:
                 #print "jerk_root=",root
                 if np.fabs(np.imag(root)) < 1e-6: # only concerned with real roots
                     t = np.real(root)
                     if (t >= 0.0 and t <= dt):
                         # Extrema of velocity in range
                         found_extrema = True
                         a_ext = np.polyval(p_acc,t)
                         a_max = np.fabs(a_ext)
                         #print"found accelertion extrema at t=",t," a_ext=",a_ext
                         if (a_max > self.joint_limits[joint_name].v_max):
                             jnt_violation = True
                             factor = 1.1*(a_max/self.joint_limits[joint_name].a_max)
                             print "found extrema at 0 < ",t," < ",dt," a=",a_ext, " a_max =",a_max, " time factor=",factor
                             dT_new[seg] = deepcopy(max([dT_new[seg],dT[seg]*factor]))

            if (found_extrema is False):
                  # If we didn't find extrema in real time range, then check end points
                  a_ext = p_acc[1]
                  a_max = np.fabs(a_ext)
                  #print"found accelertion extrema at t=",0," a_ext=",a_ext
                  if (a_max > self.joint_limits[joint_name].a_max):
                      jnt_violation = True
                      factor = 1.1*(a_max/self.joint_limits[joint_name].a_max)
                      print "found acc extrema at t=0  v=",a_ext, " a_max =",a_max, " time factor=",factor
                      dT_new[seg] = deepcopy(max([dT_new[seg],dT[seg]*factor]))

                  a_ext = np.polyval(p_vel,dt)
                  a_max = np.fabs(a_ext)
                  #print"found accelertion extrema at t= dt a_ext=",a_ext
                  if (a_max > self.joint_limits[joint_name].a_max):
                      jnt_violation = True
                      factor = 1.1*(a_max/self.joint_limits[joint_name].a_max)
                      print "found acc extrema at t=dt  v=",a_ext, " a_max =",a_max, " time factor=",factor
                      dT_new[seg] = deepcopy(max([dT_new[seg],dT[seg]*factor]))

        return jnt_violation, dT_new
      except Exception as msg:
          print "Exception during check_limits for c3 smoother"
          print "     ",msg
          traj_utility.PrintException()
          raise (msg)

