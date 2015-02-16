import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from copy import deepcopy
import time
import sys
from   vigir_trajectory_utility import VigirTrajectoryLimits
import vigir_trajectory_utility  as traj_utility

"""
Generic smoothing code takes a JointTrajectory.msg as input assuming knot points are defined by parabolic smoothing,
, and recalculates acceleration to give smooth quintic spline interpolation.

  The resulting segments give c3 continuity (through acceleration) at the knot points defined by position and velocity, and
  uses linear jerk profiles through each segment to guarantee "kink less" position profiles within a segment.
.
"""

class VigirTrajectoryQuarticSmoother(object):
    def __init__(self, joint_limits,time_limit = 1.0, max_iterations=50):

        # Store the limits used to smooth each joint as a map from joint name to simple limits structure
        # For now we don't handle time factor, and dynamic reparameterization
        self.joint_limits   = joint_limits
        self.time_limit     = time_limit
        self.max_iterations = max_iterations

    def shutdown():
        print "Shutting down the VigirTrajectoryQuarticSmoother!"

    def reparameterize(self, old_trajectory):
      try:
        #print "Reparameterize using VigirTrajectoryQuarticSmoother"
        count = 0
        elapsed = 0.0

        start_time = time.clock()

        new_trajectory = deepcopy(old_trajectory)

        num_segments = len(old_trajectory.points) -1
        if (num_segments < 2):
            print "Invalid number of segments =",num_segments," cannot smooth!"
            return new_trajectory

        # Apply time limits and recalculate the accelerations
        #print "Refactor the time intervals ..."
        new_trajectory, dT_new  = self.time_refactor(new_trajectory, num_segments)

        end_time = time.clock()
        elapsed = end_time - start_time
        print "Total elapsed time = ",elapsed, " to smooth ",num_segments," segments"

        return new_trajectory

      except Exception as msg:
        print "Exception during trajectory reparameterization using piecewise smoother"
        print "     ",msg
        traj_utility.PrintException()

        print "     Force piecewise linear commands ..."
        new_trajectory = deepcopy(old_trajectory)
        for point in new_trajectory.points:
            point.velocities    = ()
            point.accelerations = ()
        print "done reparameterization with exception"
        return new_trajectory

    """
        This function assumes piecewise linear segments, and determines the appropriate time intervals and segmentation plan to
        generate c3 smooth quintic splines.  If limits are violated, then the time interval is increase by a proportional factor
    """
    def time_refactor(self, new_trajectory, num_segments):
      try:

        print "Re-do time parameterization for quartic smoothing ..."
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

        print "Original dT=",dT_new

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

        print "Post velocity check dT=",dT_new

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

                dq   = (dq12+dq01)/2.0
                #dq = (dq12*dT_new[seg-1] + dq01*dT_new[seg])/(dT_new[seg-1]+dT_new[seg]) # weight by swapped time duration
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

        # Now calculate the spline parameters assuming a quartic spline for each segment
        violation = True
        count = 0
        while violation:
            count += 1
            violation = False
            for seg in range(1, num_segments):

                segment_violation = self.update_prior_segment(seg, new_trajectory.points[seg-1], new_trajectory.points[seg], new_trajectory.points[seg+1], dT_new)
                violation = violation or segment_violation

            if (count > 10):
                break

        print "Final parameterization - violation = ",violation, " count = ", count
        return new_trajectory, dT_new

      except Exception as msg:
          print "Exception during create_two_segment_transition reparameterization"

          print msg
          traj_utility.PrintException()
          raise (msg)

    def update_prior_segment(self, seg, p0, p1, p2, dT_new):
      try:
        #print "Creating quintic splines for two segment transition..."

        # Recalculate the average velocties
        Dq01 = p1.positions - p0.positions
        Dq12 = p2.positions - p1.positions

        dq01 = Dq01/dT_new[seg-1]
        dq12 = Dq12/dT_new[seg]
        #dq   = (dq12*dT_new[seg-1]+dq01*dT_new[seg])/(dT_new[seg-1]+dT_new[seg-1]) # weight by the time duration (see earlier note)
        dq   = (dq12+dq01)/2.0

        # Update the velocity data
        p1.velocities = deepcopy(dq)

        # Update the time
        p1.time_from_start = p0.time_from_start + rospy.Duration(dT_new[seg-1])

        # Calculate quartic splines for the prior segment
        #print "seg=",seg,"  dT_new = ",dT_new

        Mi = self.create_M_quartic_segment(dT_new[seg-1]) # dT is same for all joints in segment
        spline_list = []

        # Calculate the update accelerations and check limits for each joint
        segment_violation = False
        for jnt in range(len(p0.positions)):
            # Define the spline per joint and append to spline_list
            x = self.create_x_vector_quartic_segment(p0.positions[jnt], p0.velocities[jnt], p0.accelerations[jnt], p1.positions[jnt], p1.velocities[jnt])
            jnt_spline = np.dot(Mi,x)

            # Update the acceleration value from quartic spline data
            acc  = np.zeros(5); acc[0] =12.0;  acc[1] = 6.0; acc[2] = 2.0;
            p_acc  = (jnt_spline*acc)[:-2]  # quadratic
            acc_seg = np.polyval(p_acc,dT_new[seg-1])
            p1.accelerations[jnt] = deepcopy(acc_seg)

            # Check limits for this spline and increase dT if necessary
            jnt_violation = False

            if (jnt_violation):
                dT_new[seg-1] *= 1.1;
                segment_violation = True

        return segment_violation
      except Exception as msg:
          print "Exception during update_prior_segment "

          print msg
          traj_utility.PrintException()
          raise (msg)


    """
    These 2 functions define M and x for M p = x for 2 jerk cycles to accelerate followed
    by linear (constant velocity segment)
    """
    def create_M_quartic_segment(self, dT): # dT is same for all joints in segment

        unknowns = 5
        M = np.zeros((unknowns, unknowns));

        eqns = 0
        # Use numpy format for polynomial:  p[0] * x**n + p[1] * x**(n-1) + ... + p[n-1]*x + p[n]

        # Initial point q=p[0], dq[0]=ddq=0
        M[0][4] = 1 # q[0]
        M[1][3] = 2 # dq[0]
        M[2][2] = 6 # ddq[0]
        eqns += 3

        # Known position and velocity at segment end point
        M[3][0] =   pow(dT,4); M[3][1] =   pow(dT,3); M[3][2] = pow(dT,2); M[3][3] = dT; M[3][4] = 1.0 # q[1]
        M[4][0] = 4*pow(dT,3); M[4][1] = 3*pow(dT,2); M[4][2] =    2*dT;   M[4][3] = 1.0;

        eqns += 2

        rank = np.linalg.matrix_rank(M)
        ##np.set_printoptions(precision=3, linewidth=180)
        ##print "M="
        ##print M
        ##print "rank=",rank

        if (unknowns != eqns) or (rank != unknowns):
            condition_M = np.linalg.cond(M)
            np.set_printoptions(precision=3, linewidth=180)
            print "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
            print "M="
            print M
            print 'Condition number = ', condition_M
            print "Invalid number of equations = ",eqns," and unknowns=",unknowns,"  rank(M)=",rank
            print "dT=",dT
            print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
            raise Exception("rank"+str(rank)+" condition="+str(condition_M))

        # Inverted matrix
        Mi = np.linalg.inv(M)
        ##print "return the inverted matrix"

        return Mi

    def create_x_vector_quartic_segment(self, q0, dq0, ddq0, q1, dq1):
        unknowns = 5

        x = np.zeros(unknowns);

        # Solution data (all else = 0)
        x[0] = deepcopy(q0)   # position at start
        x[1] = deepcopy(dq0)  # velocity at start
        x[2] = deepcopy(ddq0) # acceleration at start
        x[3] = deepcopy(q1)   # position at end
        x[4] = deepcopy(dq1)  # velocity at end
        #print "x=",x

        return x

