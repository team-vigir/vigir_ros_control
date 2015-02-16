import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from copy import deepcopy
import time
import sys
from   vigir_trajectory_utility import VigirTrajectoryLimits
import vigir_trajectory_utility  as traj_utility

"""
Generic smoothing code takes a JointTrajectory.msg as input assuming piecewise linear motion (constant velocity) between the knotpoints,
, and recalculates by adding segments around prior knot points, and assuming a
        jerk-constant accel-jerk-const velocity - jerk -  constant accel -- jerk pattern around the original segments.
  The jerk segements use a quadratic jerk profile starting and ending at 0.0 to give C3 continuity of the resulting curve.
.
"""

class VigirTrajectoryPiecewiseSmoother(object):
    def __init__(self, joint_limits,time_limit = 1.0, max_iterations=50):

        # Store the limits used to smooth each joint as a map from joint name to simple limits structure
        # For now we don't handle time factor, and dynamic reparameterization
        self.joint_limits   = joint_limits
        self.time_limit     = time_limit
        self.max_iterations = max_iterations

    def shutdown():
        print "Shutting down the VigirTrajectorySmoother!"

    def reparameterize(self, old_trajectory):
      try:
        #print "Reparameterize using VigirTrajectoryPiecewiseSmoother"
        count = 0
        elapsed = 0.0

        start_time = time.clock()

        new_trajectory = deepcopy(old_trajectory)

        num_segments = len(old_trajectory.points) -1
        if (num_segments < 2):
            print "Invalid number of segments =",num_segments," cannot smooth!"
            return new_trajectory

        # Apply time limits to piecewise linear segments
        #print "Refactor the time intervals ..."
        new_trajectory, dT_plan, resegment_plan = self.time_refactor(new_trajectory, num_segments)


        # Add new segements a the current knot points
        #print "Resegment the trajectory given new intervals ..."
        new_trajectory, dT_new, num_segments = self.resegment_trajectory(new_trajectory, dT_plan, resegment_plan)

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
        dT_new = [ [ [0.00000001, 0.0, 0.00000001], 0.0, [0.00000001, 0.0, 0.00000001] ] for x in xrange(num_segments)]
        resegment_plan = [ [0, 1, 0] for i in xrange(num_segments)]

        # Set the linear segment to the initial time duration
        for seg in range(0,num_segments):
            delta = new_trajectory.points[seg+1].time_from_start - new_trajectory.points[seg].time_from_start
            dT_new[seg][1] = delta.to_sec()

            # Convert to array for reassignment
            new_trajectory.points[seg].positions  = np.asarray(new_trajectory.points[seg].positions)
            new_trajectory.points[seg].velocities  = np.asarray(new_trajectory.points[seg].velocities)
            new_trajectory.points[seg].accelerations  = np.asarray(new_trajectory.points[seg].accelerations)

        # Convert terminal point to array for reassignment
        new_trajectory.points[-1].positions     = np.asarray(new_trajectory.points[-1].positions)
        new_trajectory.points[-1].velocities    = np.asarray(new_trajectory.points[-1].velocities)
        new_trajectory.points[-1].accelerations = np.asarray(new_trajectory.points[-1].accelerations)

        #print "Original dT=",dT_new
        #print "      reseg=",resegment_plan

        # "Check absolute minimums and average velocity ..."
        for jnt, joint_name in enumerate(new_trajectory.joint_names):
            limits = self.joint_limits[joint_name]

            for seg in range(0,num_segments):
                if (dT_new[seg][1] < limits.dt_blend_interval):
                    # Absolute minimum time we will consider with zero splits
                    #print "Seg=",seg,"  update minimal interval=",limits.dt_blend_interval
                    dT_new[seg][1] = deepcopy(limits.dt_vel_min)

            # Per joint checks on average velocity
            for seg in range(0,num_segments):
                Dq  = new_trajectory.points[seg+1].positions[jnt] - new_trajectory.points[seg].positions[jnt]

                dt_min = np.fabs(Dq/limits.v_max)
                if (dt_min > dT_new[seg][1]):
                    #print "velocity limit current[",seg,"] =",limits.v_max,"  Dq=",Dq," dt_min=",dt_min
                    dT_new[seg][0][0] = limits.dt_blend_hump
                    dT_new[seg][0][2] = limits.dt_blend_hump
                    dT_new[seg][1]    = dt_min
                    resegment_plan[seg][0] = 2

                elif (np.fabs(Dq) > 0.0001):
                    # Some motion, so define transition
                    #print "motion requires at least 2 segment transition current[",seg,"] =",limits.v_max,"  Dq=",Dq," dt_min=",dt_min
                    dT_new[seg][0][0] = limits.dt_blend_hump
                    dT_new[seg][0][2] = limits.dt_blend_hump
                    resegment_plan[seg][0] = 2

        #print "Post velocity check dT=",dT_new
        #print "                 reseg=",resegment_plan


        #print "Calculate average velocity for each segment ..."
        for jnt, joint_name in enumerate(new_trajectory.joint_names):
            limits = self.joint_limits[joint_name]
            #print jnt," ",joint_name

            # Per joint checks on average velocity after all timing limits applied
            for seg in range(0,num_segments):
                #print "seg=",seg," jnt=",jnt
                #print "    dT_new*=",dT_new
                #print "    reseg=",resegment_plan

                Dq  = new_trajectory.points[seg+1].positions[jnt] - new_trajectory.points[seg].positions[jnt]
                dq = Dq/dT_new[seg][1]
                if (np.fabs(dq) > limits.v_max):
                    print "wft? .... this should not happen!"
                    print "dq=",dq,"  dt=",dT_new[seg]," Dq=",Dq," v_max = ",limits.v_max

                new_trajectory.points[seg].velocities[jnt] = deepcopy(dq)

                # Now check for the average acceleration
                # No velocity change does not require a transition zone (e.g. constant zero)
                # average accel < Amax, then use 2 humps with 1/2 blend interval
                # average accel > Amax, then use hump - Amax - hump with hump = acc_min
                if (seg > 0):
                    v0 = new_trajectory.points[seg-1].velocities[jnt]
                    v1 = new_trajectory.points[ seg ].velocities[jnt]
                    Ddq =  v1 - v0 # change in average velocity

                    # Current transition time
                    sum_t_trans = dT_new[seg][0][0] + dT_new[seg][0][1] + dT_new[seg][0][2]

                    ddq = Ddq/min([sum_t_trans, 2*limits.dt_blend_hump])
                    dt_min = np.fabs(Ddq)/limits.a_max
                    a_avg_failed = (np.fabs(ddq) > limits.a_max)
                    if (dt_min > sum_t_trans) or a_avg_failed:
                        # We are acceleration limited
                        q0 = new_trajectory.points[seg-1].positions[jnt]
                        q1 = new_trajectory.points[ seg ].positions[jnt]
                        q2 = new_trajectory.points[seg+1].positions[jnt]
                        Dq = 0.5*q2 - 0.5*q0 # total change in position across both segments around the original knot
                        #print "      acceleration limit current[",seg,"] =",limits.a_max,"  Dq=",Dq,"  Ddq=",Ddq," dt_min=",dt_min, " sum_t=",sum_t_trans," a_avg_failed=",a_avg_failed

                        # At maximum acceleration, how long to cover distance Dq
                        #  Dq=d*t^2 + e*t + f  ===>  2*d = a_max  e=v0  f= 0, for quadratic equation let f=-Dq
                        #  t= (-e +/- sqrt(e^2 - 4df))/(2*d)
                        a_max = np.sign(Ddq)*limits.a_max
                        desc = v0*v0 + 2.0*(a_max)*Dq  # 4df with d=a_max/2
                        #print "         descriminant desc=",desc,"  a=",a_max," b=",v0," c=",-Dq, "  b^2 =",v0*v0, " 4ac=", 4.0*(a_max/2.0)*(-Dq)
                        if (desc >= 0.0):
                            desc = np.sqrt(desc)
                            dt_quad_1 = (-v0 + desc)/a_max  # a_max = 2*d
                            dt_quad_2 = (-v0 - desc)/a_max
                            dt_quad = max([dt_quad_1, dt_quad_2])
                            #print "              at a_max=",a_max," dt_quad=",dt_quad," (",dt_quad_1,", ",dt_quad_2,") dt_min_old=",dt_min
                            dt_min = min([dt_min, dt_quad])
                        else:
                            #print "         invalid descriminant desc=",desc,"  a_max=",a_max," v0=",v0," Dq=",Dq, "  v0*v0=",v0*v0, " -4ac=", 2.0*(a_max)*Dq
                            #print "         time to 0 = ",v0/a_max, "  time to Dq=",Dq/max(0.0001,v0)
                            #print "         increase dt_min slightly!"
                            dt_min += limits.dt_blend_hump
                            pass

                        a_avg_failed = a_avg_failed and (dt_min > limits.dt_blend_hump) # need minimal constant acceleration to use 3 segment transition

                        # Use a blending zone
                        if a_avg_failed or (resegment_plan[seg][0] > 2) :
                            #print "         3 seg transition required ...a_avg_failed=",a_avg_failed, " ",resegment_plan[seg][0]
                            resegment_plan[seg][0] = 3
                            dt_min -= limits.dt_blend_hump # only subtract one hump to account for rise time
                            if (dt_min > dT_new[seg][0][1]):
                                dT_new[seg][0][0] = deepcopy(limits.dt_blend_hump)
                                dT_new[seg][0][1] = deepcopy(dt_min)
                                dT_new[seg][0][2] = deepcopy(limits.dt_blend_hump)


                            # Calculate expected terminal velocity after max acceleration
                            dtt = (dT_new[seg][0][1] + dT_new[seg][0][2])
                            v_end = v0 + a_max*dtt
                            Dq_end = v0*dtt + 0.5*a_max*dtt * dtt
                            v_bar = v0 + 0.5*a_max*dtt # average of acceleration interval
                            if (np.fabs(v_bar) > 1e-6):
                                t_linear = max([0.0, (q2-q1-Dq_end)/v_bar]) #+ dT_new[seg][0][1]
                                t_linear0 = deepcopy(t_linear)
                                t_linear1 = deepcopy(t_linear)
                                if (np.fabs(v0) > 1e-6):
                                    t_linear0 = max([0.0, ((q2-q1-Dq_end)/v0 + 0.5*dT_new[seg][0][1]+dT_new[seg][0][2] ) ] )
                                if (np.fabs(v_end) > 1e-6):
                                    t_linear1 = max([ 0.0, ( (q2-q1-Dq_end)/v_end + 0.5*dT_new[seg][0][1]+dT_new[seg][0][2] ) ] )


                                #print "         Check linear segment time current ",dT_new[seg][1]," new ts=",t_linear,", ",t_linear0,", ",t_linear1, " v_end=",v_end, " v0=",v0, " Dq_end=",Dq_end, "  dtt=",dtt," dq_avg=",new_trajectory.points[ seg ].velocities[jnt]
                                t_linear = min([t_linear, t_linear0, t_linear1]) # minimum interval over linear section
                                if (t_linear > dT_new[seg][1]):
                                    dq_new = (q2-q1)/(t_linear)
                                    #print "         Increasing linear segment time from ",dT_new[seg][1]," to ",t_linear, " dq_new=",dq_new
                                    dT_new[seg][1] = t_linear

                            # Recalculate the average velocity
                            dq = (q2-q1)/dT_new[seg][1]
                            #print "         new dq=",dq,"   old=",new_trajectory.points[ seg ].velocities[jnt]

                            new_trajectory.points[ seg ].velocities[jnt] = dq

                            dt = np.fabs(dq-v0)/limits.a_max
                            if (dt > (dT_new[seg][0][1] + dT_new[seg][0][2])):
                                #print "        increase constant accel duration dt=",dt-dT_new[seg][0][2]," old=",dT_new[seg][0][1]
                                dT_new[seg][0][1] = (dt -dT_new[seg][0][2])

                        else:
                            #print "         2 seg transition required ...a_avg_failed=",a_avg_failed, " ",resegment_plan[seg][0]," dt_min=",dt_min

                            resegment_plan[seg][0] = 2
                            if (dt_min > 2*limits.dt_blend_hump):
                                #dt_min -= 2*limits.dt_blend_hump
                                t_update = 0.5*dt_min #limits.dt_blend_hump +
                                if (t_update > dT_new[seg][0][0]):
                                    #print "         Update segment bump time -t_update=",t_update
                                    dT_new[seg][0][0] = deepcopy(t_update)
                                    dT_new[seg][0][2] = deepcopy(t_update)
                                    # Recalculate the average velocity
                                    dq = Dq/(dT_new[seg][0][2]+dT_new[seg][1])
                                    new_trajectory.points[ seg ].velocities[jnt] = dq
                                else:
                                    #print "         Nothing to update segment time -t_update=",t_update
                                    pass
                            else:
                                #print "         < minimum time required = dt_min =",dt_min
                                if (limits.dt_blend_hump > dT_new[seg][0][0]):
                                    dT_new[seg][0][0] = deepcopy(limits.dt_blend_hump)
                                    dT_new[seg][0][2] = deepcopy(limits.dt_blend_hump)
                                    # Recalculate the average velocity
                                    dq = Dq/(dT_new[seg][0][2]+dT_new[seg][1])
                                    new_trajectory.points[ seg ].velocities[jnt] = dq

                    ##print "  # dT_new*=",dT_new
                    ##print "  # reseg=",resegment_plan

            ##print "dT_new&=",dT_new

            # Handle starting transition
            seg = 0
            Ddq = new_trajectory.points[seg].velocities[jnt]  # From rest
            sum_t_trans = dT_new[seg][0][0] + dT_new[seg][0][1] + dT_new[seg][0][2]

            ddq = Ddq/max([2*limits.dt_blend_hump, sum_t_trans])
            dt_min = np.fabs(Ddq)/limits.a_max
            a_avg_failed = (np.fabs(ddq) > limits.a_max)

            if (dt_min > sum_t_trans) or a_avg_failed:
                a_avg_failed = a_avg_failed and (dt_min > limits.dt_blend_hump) # need minimal constant acceleration to use 3 segment transition
                if a_avg_failed or (resegment_plan[seg][0] > 2) :
                    resegment_plan[seg][0] = 3
                    dt_min -= limits.dt_blend_hump # only subtract one hump to account for rise time
                    dT_new[seg][0][0] = deepcopy(limits.dt_blend_hump)
                    dT_new[seg][0][1] = deepcopy(dt_min)
                    dT_new[seg][0][2] = deepcopy(limits.dt_blend_hump)

                else:
                    resegment_plan[seg][0] = 2
                    if (dt_min > 2*limits.dt_blend_hump):
                        dt_min -= 2*limits.dt_blend_hump
                        t_update = limits.dt_blend_hump + 0.5*dt_min
                        dT_new[seg][0][0] = deepcopy(t_update)
                    else:
                        dT_new[seg][0][0] = deepcopy(limits.dt_blend_hump)
                    dT_new[seg][0][1] = 0.0
                    dT_new[seg][0][2] = deepcopy(dT_new[seg][0][0])
            ##print " start dT_new=",dT_new[seg]

            # Handle final transition
            #print "handle final for transition for jnt=",jnt
            Ddq = -new_trajectory.points[-1].velocities[jnt] # last point
            sum_t_trans = dT_new[-1][2][0] + dT_new[-1][2][1] + dT_new[-1][2][2]
            ddq = Ddq/min([sum_t_trans, 2*limits.dt_blend_hump])
            dt_min = np.fabs(Ddq)/limits.a_max
            a_avg_failed = (np.fabs(ddq) > limits.a_max)

            if (dt_min > sum_t_trans) or a_avg_failed:
                a_avg_failed = a_avg_failed and (dt_min > 2*limits.dt_blend_hump) # need minimal constant acceleration to use 3 segment transition
                if a_avg_failed or (resegment_plan[seg][0] > 2) :
                    resegment_plan[seg][0] = 3
                    dT_new[-1][2][0] = deepcopy(limits.dt_blend_hump)
                    dT_new[-1][2][1] = deepcopy(max([dT_new[-1][2][1], (dt_min-limits.dt_blend_hump)]))
                    dT_new[-1][2][2] = deepcopy(limits.dt_blend_hump)

                else:
                    if (dt_min > limits.dt_blend_hump):
                        dt_min -= limits.dt_blend_hump
                    else:
                        dt_min = 0.0
                    resegment_plan[seg][0] = 2
                    dT_new[-1][2][0] = deepcopy(max([dT_new[-1][2][0], limits.dt_blend_hump + 0.5*dt_min]))
                    dT_new[-1][2][1] = 0.0
                    dT_new[-1][2][2] = deepcopy(dT_new[-1][2][0])

        #print "-----------------------------------------------------------"
        # Now final process to adjust the segment intervals
        for seg in range(1,len(dT_new)):
            dT_new[seg-1][2]         = deepcopy(dT_new[seg][0])
            resegment_plan[seg-1][2] = deepcopy(resegment_plan[seg][0])

        dT_new[0][1] -= (0.5*dT_new[0][0][1] + dT_new[0][0][2] + dT_new[0][2][0] + 0.5*dT_new[0][2][1])
        if (dT_new[0][1] < 0.002):
            dT_new[0][1] = 0.002

        for seg in range(1,len(dT_new)):
            dT_new[seg][1] -= (0.5*dT_new[seg][0][1] + dT_new[seg][0][2] + dT_new[seg][2][0] + 0.5*dT_new[seg][2][1])
            if (dT_new[seg][1] < 0.002):
                dT_new[seg][1] = 0.002

        #print "New dT",dT_new
        #print "Resegment plan:", resegment_plan
        #print "-----------------------------------------------------------"
        #print "-----------------------------------------------------------"

        return new_trajectory, dT_new, resegment_plan
      except Exception as msg:
          print "Exception during create_two_segment_transition reparameterization"

          print msg
          traj_utility.PrintException()
          raise (msg)

    """
        This function creates new segments around each prior knot point.  It assumes piecewise linear interpolation along
        the old segments (constant velocity, zero accel), and assumes continuous position, velocity, acceleration along new segments.

        This assumes each segment as a reasonable minimal delta t
    """
    def resegment_trajectory(self, trajectory, dT_plan, resegment_plan):
        # Define the initial time intervals for each segment
        num_old_segments = len(trajectory.points) -1

        if (num_old_segments != len(dT_plan)):
            msg="Invalid segments="+str(num_old_segments)+"  len="+str(len(dT_plan))
            print msg
            raise Exception(msg)

        num_joints       = len(trajectory.joint_names)
        #print "Resegment old trajectory with ",num_old_segments," and ",num_joints," joints ..."
        #print "dT_plan = ",dT_plan
        #print "resegment =",resegment_plan

        if (num_joints != len(trajectory.points[0].positions)):
            msg = "Invalid number of joints = "+str(num_joints)+"  positions="+str(len(trajectory.points[0].positions))
            print msg
            raise Exception(msg)

        num_new_segments = 0
        for ndx, segment in enumerate(resegment_plan):
            num_new_segments += segment[0] # transition zone
            num_new_segments += segment[1] # linear zone

        num_new_segments += resegment_plan[-1][2] # final transition zone

        #print "original segments=",num_old_segments, " new segments=",num_new_segments, "dT_plan=",dT_plan



        # Initialize new trajectory data
        dT_new = [0 for x in xrange(num_new_segments)]
        zero_accelerations = np.asarray(tuple(0.0 for x in xrange(num_joints)))

        next_old_knot = trajectory.points[0]
        next_old_knot.positions     = deepcopy(trajectory.points[0].positions)
        next_old_knot.velocities    = deepcopy(zero_accelerations)  # Assume we start at rest
        next_old_knot.accelerations = deepcopy(zero_accelerations)

        new_trajectory = deepcopy(trajectory)
        new_trajectory.points = deepcopy([deepcopy(next_old_knot) for x in xrange(num_new_segments+1)])

        new_knot = 0
        old_knot = 0

        for seg in range(num_old_segments):

            prior_old_knot = deepcopy(next_old_knot)
            prior_knot     = deepcopy(old_knot)

            new_knot += 1
            old_knot += 1
            next_old_knot= deepcopy(trajectory.points[old_knot])
            next_old_knot.positions     = np.asarray(trajectory.points[old_knot].positions)
            next_old_knot.velocities    = np.asarray(trajectory.points[old_knot].velocities)
            next_old_knot.accelerations = np.asarray(trajectory.points[old_knot].accelerations)

            total_segment_time = 0.5*dT_plan[seg][0][1] + dT_plan[seg][0][2] + dT_plan[seg][1] + dT_plan[seg][2][0] + 0.5*dT_plan[seg][2][1]
            start_T1 = 0.5*dT_plan[seg][0][1] + dT_plan[seg][0][2]
            start_T2 = total_segment_time - (dT_plan[seg][2][0] + 0.5*dT_plan[seg][2][1])

            delta_q = next_old_knot.positions  - prior_old_knot.positions
            new_q01 = prior_old_knot.positions + delta_q*(start_T1/total_segment_time)
            new_q1  = prior_old_knot.positions + delta_q*(start_T2/total_segment_time)

            ##print "seg=",seg," new_knot=",new_knot, " plan=",resegment_plan[seg], "start_T2=",start_T2," total=",total_segment_time,"  fraction=",(start_T2/total_segment_time)
            ##print "old=",prior_old_knot.positions," q1=",new_q1, "Dq=",delta_q
            ##print "old vel=",prior_old_knot.velocities
            ##print "prior=",new_trajectory.points[new_knot-1].positions
            ##print "prior vel=",new_trajectory.points[new_knot-1].velocities

            if (resegment_plan[seg][0] == 2):
              try:
                # 2 transition segments
                ##print "seg=",seg," new_knot=",new_knot, " plan=",resegment_plan[seg], "start_T2=",start_T2," total=",total_segment_time,"  fraction=",(start_T2/total_segment_time)
                ##print "old=",prior_old_knot.positions," q1=",new_q1, "Dq=",delta_q
                ##print "old vel=",prior_old_knot.velocities
                ##print "prior=",new_trajectory.points[new_knot-1].positions
                ##print "prior vel=",new_trajectory.points[new_knot-1].velocities

                spline_list,dT_trans = self.create_two_segment_transition(new_trajectory.points[new_knot-1].positions, new_trajectory.points[new_knot-1].velocities, new_q01, new_q1, dT_plan[seg])

                ##print "dT_trans=",dT_trans

                # Convert spline list to knot points
                for jnt, joint in enumerate(new_trajectory.joint_names):
                    ##print spline_list[jnt]
                    # start of second jerk hump
                    new_trajectory.points[new_knot].positions[jnt]     = deepcopy(spline_list[jnt][11])
                    new_trajectory.points[new_knot].velocities[jnt]    = deepcopy(spline_list[jnt][10])
                    new_trajectory.points[new_knot].accelerations[jnt] = deepcopy(2.0*spline_list[jnt][9])

                    # start of linear segment
                    new_trajectory.points[new_knot+1].positions[jnt]     = deepcopy(spline_list[jnt][13])
                    new_trajectory.points[new_knot+1].velocities[jnt]    = deepcopy(spline_list[jnt][12])

                # end of linear section
                new_trajectory.points[new_knot+2].positions  = deepcopy(new_q1)
                new_trajectory.points[new_knot+2].velocities = deepcopy(new_trajectory.points[new_knot+1].velocities)

                ##print "dT_new^ =",dT_new
                ##print "dT_trans=",dT_trans," new_knot=",new_knot,"  length=",len(new_trajectory.points),"=",len(dT_new)
                dT_new[new_knot-1] = dT_trans[0]
                dT_new[new_knot  ] = dT_trans[1]
                dT_new[new_knot+1] = dT_trans[2]
                ##print "dT_new =",dT_new

                new_trajectory.points[new_knot+1].accelerations   = zero_accelerations
                new_trajectory.points[new_knot+2].accelerations   = zero_accelerations
                new_trajectory.points[new_knot  ].time_from_start = (new_trajectory.points[new_knot-1].time_from_start + rospy.Duration(dT_trans[0]))
                new_trajectory.points[new_knot+1].time_from_start = (new_trajectory.points[new_knot  ].time_from_start + rospy.Duration(dT_trans[1]))
                new_trajectory.points[new_knot+2].time_from_start = (new_trajectory.points[new_knot+1].time_from_start + rospy.Duration(dT_trans[2]))
                new_knot += 2

              except Exception as msg:
                    print "Exception during create_two_segment_transition reparameterization"

                    print msg
                    traj_utility.PrintException()

                    print "seg=",seg," new_knot=",new_knot, " plan=",resegment_plan[seg], "start_T2=",start_T2," total=",total_segment_time,"  fraction=",(start_T2/total_segment_time)
                    print "dT_plan=",dT_plan
                    print "resegment_plan=",resegment_plan
                    sys.exit(1)

            elif (resegment_plan[seg][0] == 3):
                try:
                  # 3 transition segments
                  spline_list,dT_trans = self.create_three_segment_transition(new_trajectory.points[new_knot-1].positions, new_trajectory.points[new_knot-1].velocities, new_q01, new_q1, dT_plan[seg])
                  ##Mtest = self.test_M_three_segment_transition(dT_trans) # dT is same for all joints in segment

                  # Convert spline list to knot points
                  for jnt, joint in enumerate(new_trajectory.joint_names):
                      ##print "--------------- ",jnt," ----------------------------"
                      ##np.set_printoptions(precision=3, linewidth=180)
                      ##print spline_list[jnt]

                      ##values = np.dot(Mtest,spline_list[jnt])
                      ##values.resize(values.size/4,4)
                      ##print values

                      # start of constant accerlation segment
                      new_trajectory.points[new_knot].positions[jnt]     = deepcopy(spline_list[jnt][8])
                      new_trajectory.points[new_knot].velocities[jnt]    = deepcopy(spline_list[jnt][7])
                      new_trajectory.points[new_knot].accelerations[jnt] = deepcopy(2.0*spline_list[jnt][6])

                      # start of second jerk hump
                      new_trajectory.points[new_knot+1].positions[jnt]     = deepcopy(spline_list[jnt][14])
                      new_trajectory.points[new_knot+1].velocities[jnt]    = deepcopy(spline_list[jnt][13])
                      new_trajectory.points[new_knot+1].accelerations[jnt] = deepcopy(2.0*spline_list[jnt][12])

                      # start of linear segment
                      new_trajectory.points[new_knot+2].positions[jnt]     = deepcopy(spline_list[jnt][16])
                      new_trajectory.points[new_knot+2].velocities[jnt]    = deepcopy(spline_list[jnt][15])

                  # end of linear section
                  new_trajectory.points[new_knot+3].positions  = deepcopy(new_q1)
                  new_trajectory.points[new_knot+3].velocities = deepcopy(new_trajectory.points[new_knot+2].velocities)

                  ##print "dT_trans=",dT_trans," new_knot=",new_knot,"  length=",len(new_trajectory.points),"=",len(dT_new)
                  dT_new[new_knot-1] = dT_trans[0]
                  dT_new[new_knot  ] = dT_trans[1]
                  dT_new[new_knot+1] = dT_trans[2]
                  dT_new[new_knot+2] = dT_trans[3]
                  ##print "dT_new =",dT_new

                  new_trajectory.points[new_knot+2].accelerations   = zero_accelerations
                  new_trajectory.points[new_knot+3].accelerations   = zero_accelerations
                  new_trajectory.points[new_knot  ].time_from_start = (new_trajectory.points[new_knot-1].time_from_start + rospy.Duration(dT_trans[0]))
                  new_trajectory.points[new_knot+1].time_from_start = (new_trajectory.points[new_knot  ].time_from_start + rospy.Duration(dT_trans[1]))
                  new_trajectory.points[new_knot+2].time_from_start = (new_trajectory.points[new_knot+1].time_from_start + rospy.Duration(dT_trans[2]))
                  new_trajectory.points[new_knot+3].time_from_start = (new_trajectory.points[new_knot+2].time_from_start + rospy.Duration(dT_trans[3]))
                  new_knot += 3

                except Exception as msg:
                      print "Exception during create_three_segment_transition reparameterization"
                      print msg
                      traj_utility.PrintException()
                      print "seg=",seg," new_knot=",new_knot, " plan=",resegment_plan[seg], "start_T2=",start_T2," total=",total_segment_time,"  fraction=",(start_T2/total_segment_time)
                      print "dT_plan=",dT_plan
                      print "resegment_plan=",resegment_plan
                      sys.exit(1)
            else:
                # Only linear segment
                dT_new[new_knot-1] = dT_plan[seg][1]
                new_dq = (new_q1 - prior_old_knot.positions)/dT_new[new_knot-1]
                ##print "linear seg=",new_q1," - ",prior_old_knot.positions," = ",new_dq

                new_trajectory.points[new_knot].positions     = tuple(new_q1)
                new_trajectory.points[new_knot].velocities    = tuple(new_dq)#[dq for dq in new_dq])
                new_trajectory.points[new_knot].accelerations = zero_accelerations

                dT_seg = dT_plan[seg][1]

                new_trajectory.points[new_knot].time_from_start = (new_trajectory.points[new_knot-1].time_from_start + rospy.Duration(dT_seg))

                ##print new_trajectory.points[new_knot].positions
                ##print new_trajectory.points[new_knot].velocities
                ##print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
                new_knot += 0

            ##print new_trajectory.points

        ##print "new_trajectory="
        ##print new_trajectory.points
        ##print "new_knot=",new_knot

        # Terminal
        new_knot += 1
        if (resegment_plan[seg][2] == 2):
            # 2 transition segments
            print "Two segment transition for final segment for this trajectory (len=",len(new_trajectory.points),") -- NEED TO FIX!"
            #@todo
            new_knot += 1
        elif (resegment_plan[seg][2] == 3):
            # 3 transition segments
            print "Three segment transition for final segment for this trajectory (len=",len(new_trajectory.points),") -- NEED TO FIX!"
            #@todo
            new_knot += 2
        elif (resegment_plan[seg][2] == 0):
            #print "No terminal transition segment for this trajectory - just add the terminal point (len=",len(new_trajectory.points),")"
            pass
        else:
            print "Invalid final transition data!"

        if (new_knot != len(new_trajectory.points)):
            msg = "Error processing the new trajectory! knots="+str(new_knot)+" =/= "+str(len(new_trajectory.points))
            print msg
            raise Exception(msg)

        ##print "Return new traj"
        ##print new_trajectory
        ##print dT_new
        ##
        ##print "Time Check:"
        ##print dT_plan
        ##print " seg : position ,  velocity ,    accel  ,   dT =?= dT"
        ##for seg in range(num_new_segments):
        ##    print "{0:3d} : {1: 10.6g}, {2: 10.6g}, {3: 10.6g} | {4:8.3f} =?= {5:8.3f}".format(seg, new_trajectory.points[seg].positions[0], new_trajectory.points[seg].velocities[0], new_trajectory.points[seg].accelerations[0],dT_new[seg], (new_trajectory.points[seg+1].time_from_start - new_trajectory.points[seg].time_from_start).to_sec())

        ##print "Finished resegmenting!"

        return new_trajectory, dT_new, num_new_segments


    def create_two_segment_transition(self, q0, dq0, q01, q1, dT_plan):
        #print "Creating quintic splines for two segment transition..."

        dT = [dT_plan[0][0], dT_plan[0][2], dT_plan[1]]

        #print "dT =",dT
        #print "q0 =",q0
        #print "dq0=",dq0
        #print "q1 =",q1

        Mi = self.create_M_two_segment_transition(dT) # dT is same for all joints in segment
        spline_list = []
        for jnt in range(len(q0)):
            # Define the spline per joint and append to spline_list
            x = self.create_x_vector_two_segment_transition(q0[jnt], dq0[jnt], q01[jnt], q1[jnt])
            jnt_splines = np.dot(Mi,x)
            spline_list.append(jnt_splines)

        ##print "two segment_spline list="
        ##print spline_list

        return spline_list, dT

    def create_three_segment_transition(self, q0, dq0, q01, q1, dT_plan):
        ##print "Creating quintic splines for each segment ..."

        dT = [dT_plan[0][0], dT_plan[0][1], dT_plan[0][2], dT_plan[1]]

        ##print "q0 =",q0
        ##print "dq0=",dq0
        ##print "q01=",q01
        ##print "q1 =",q1

        Mi = self.create_M_three_segment_transition(dT) # dT is same for all joints in segment
        spline_list = []
        for jnt in range(len(q0)):
            # Define the spline per joint and append to spline_list
            x = self.create_x_vector_three_segment_transition(q0[jnt], dq0[jnt], q01[jnt], q1[jnt])
            jnt_splines = np.dot(Mi,x)

            spline_list.append(jnt_splines)

        ##print "two segment_spline list="
        ##print spline_list

        return spline_list, dT

    """
    These 2 functions define M and x for M p = x for 2 jerk cycles to accelerate followed
    by linear (constant velocity segment)
    """
    def create_M_two_segment_transition(self, dT): # dT is same for all joints in segment

        unknowns = 6 + 6 + 2
        M = np.zeros((unknowns, unknowns));

        eqns = 0
        # Use numpy format for polynomial:  p[0] * x**n + p[1] * x**(n-1) + ... + p[n-1]*x + p[n]

        # Initial point q=p[0], dq[0]=ddq=0
        M[0][5] = 1 # q[0]
        M[1][4] = 1 # dq[0]
        M[2][3] = 2 # ddq[0]
        M[3][2] = 6 # dddq[0]
        eqns += 4

        # End of first jerk hump and start of next has continuous q,dq,ddq and dddq=0

        M[4][0] = pow(dT[0],5); M[4][1] = pow(dT[0],4); M[4][2] = pow(dT[0],3); M[4][3] = pow(dT[0],2); M[4][4] = dT[0]; M[4][5] = 1.0 # q[0]
        M[4][11] = -1.0 # Continuous position

        M[5][0] = 5*pow(dT[0],4); M[5][1] = 4*pow(dT[0],3); M[5][2] = 3*pow(dT[0],2); M[5][3] = 2*dT[0]; M[5][4] = 1.0;
        M[5][10] = -1.0 # Continuous velocity

        M[6][0] = 20*pow(dT[0],3); M[6][1] = 12*pow(dT[0],2); M[6][2] = 6*dT[0]; M[6][3] = 2.0;
        M[6][9] = -2.0 # Continuous acceleration

        M[7][0] = 60*pow(dT[0],2); M[7][1] = 24*dT[0]; M[7][2] = 6.0;
        M[8][8] = -6.0 # zero jerk
        eqns += 5

        #End of second jerk hump and start of linear section
        M[9][6] = pow(dT[1],5);M[9][7] = pow(dT[1],4); M[9][8] = pow(dT[1],3); M[9][9] = pow(dT[1],2); M[9][10] = dT[1]; M[9][11] = 1.0 # q[0]
        M[9][13] = -1.0 # Continuous position

        M[10][6] = 5*pow(dT[1],4); M[10][7] = 4*pow(dT[1],3); M[10][8] = 3*pow(dT[1],2); M[10][9] = 2*dT[1]; M[10][10] = 1.0;
        M[10][12] = -1.0 # Continuous velocity

        M[11][6] = 20*pow(dT[1],3); M[11][7] = 12*pow(dT[1],2); M[11][8] = 6*dT[1]; M[11][9] = 2.0; # zero accel
        eqns += 3

        # End of linear section
        M[12][13] = 1.0;                   # known intermediate position
        M[13][12] = dT[2]; M[13][13] = 1.0 # known terminal position
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

    def create_x_vector_two_segment_transition(self, q0, dq0, q01, q1):
        unknowns = 6 + 6 + 2

        x = np.zeros(unknowns);

        # Solution data (all else = 0)
        x[0]  = deepcopy(q0)
        x[1]  = deepcopy(dq0)
        x[12] = deepcopy(q01) # position at start of linear
        x[13] = deepcopy(q1)  # position at end of linear
        #print "x=",x

        return x

    """
    These 2 functions define M and x for M p = x for jerk - constant accel - jerk cycles to accelerate followed
    by linear (constant velocity segment)
    """
    def create_M_three_segment_transition(self, dT): # dT is same for all joints in segment

        unknowns = 6 + 3 + 6 + 2
        M = np.zeros((unknowns, unknowns));

        eqns = 0
        # Use numpy format for polynomial:  p[0] * x**n + p[1] * x**(n-1) + ... + p[n-1]*x + p[n]

        # Initial point q=p[0], dq[0]=ddq=0
        M[0][5] = 1 # q[0]
        M[1][4] = 1 # dq[0]
        M[2][3] = 2 # ddq[0]
        M[3][2] = 6 # dddq[0]
        eqns += 4

        # End of first jerk hump and start of constant acceleration has continuous q,dq,ddq and dddq=0
        M[4][0] = pow(dT[0],5); M[4][1] = pow(dT[0],4); M[4][2] = pow(dT[0],3); M[4][3] = pow(dT[0],2); M[4][4] = dT[0]; M[4][5] = 1.0 # q[0]
        M[4][8] = -1.0 # Continuous position

        M[5][0] = 5*pow(dT[0],4); M[5][1] = 4*pow(dT[0],3); M[5][2] = 3*pow(dT[0],2); M[5][3] = 2*dT[0]; M[5][4] = 1.0;
        M[5][7] = -1.0 # Continuous velocity

        M[6][0] = 20*pow(dT[0],3); M[6][1] = 12*pow(dT[0],2); M[6][2] = 6*dT[0]; M[6][3] = 2.0;
        M[6][6] = -2.0 # Continuous acceleration

        M[7][0] = 60*pow(dT[0],2); M[7][1] = 24*dT[0]; M[7][2] = 6.0; # zero jerk
        eqns += 4

        # end of constant acceleration followed by jerk to stop accelerating
        M[8][6]  =  pow(dT[1],2); M[8][7] = dT[1]; M[8][8] = 1.0 # q[0]
        M[8][14] = -1.0 # Continuous position

        M[9][6] =  2*dT[1]; M[9][7] = 1.0;
        M[9][13] = -1.0 # Continuous velocity

        M[10][6]  =  2.0;
        M[10][12] = -2.0 # Continuous acceleration

        M[11][11]  = -6.0 # zero jerk
        eqns += 4


        #End of second jerk hump and start of linear section
        M[12][9] = pow(dT[2],5);M[12][10] = pow(dT[2],4); M[12][11] = pow(dT[2],3); M[12][12] = pow(dT[2],2); M[12][13] = dT[2]; M[12][14] = 1.0 # q[0]
        M[12][16] = -1.0 # Continuous position

        M[13][9] = 5*pow(dT[2],4); M[13][10] = 4*pow(dT[2],3); M[13][11] = 3*pow(dT[2],2); M[13][12] = 2*dT[2]; M[13][13] = 1.0;
        M[13][15] = -1.0 # Continuous velocity

        M[14][9] = 20*pow(dT[2],3); M[14][10] = 12*pow(dT[2],2); M[14][11] = 6*dT[2]; M[14][12] = 2.0; # zero accel
        eqns += 3

        # End of linear section
        M[15][16] = 1.0;                   # known intermediate position
        M[16][15] = dT[3]; M[16][16] = 1.0 # known terminal position
        eqns += 2

        rank = np.linalg.matrix_rank(M)
        ##np.set_printoptions(precision=3, linewidth=180)
        ##print "M="
        ##print M
        ##print "rank=",rank

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
        ##print "return the inverted matrix"

        return Mi

    def create_x_vector_three_segment_transition(self, q0, dq0, q01, q1):
        unknowns = 6 + 3 + 6 + 2

        x = np.zeros(unknowns);

        # Solution data (all else = 0)
        x[0]  = deepcopy(q0)  # Starting position
        x[1]  = deepcopy(dq0) # Starting velocity (assuming 0 accel and jerk)
        x[15] = deepcopy(q01) # position at start of linear
        x[16] = deepcopy(q1)  # position at end of linear
        ##print x

        return x

    """
    This function define M for M p = x for jerk - constant accel - jerk cycles to accelerate followed
    by linear (constant velocity segment).  The M is used to test the parameter values
    """
    def test_M_three_segment_transition(self, dT): # dT is same for all joints in segment

        unknowns = 6 + 3 + 6 + 2
        M = np.zeros((32, unknowns));

        eqns = 0
        # Use numpy format for polynomial:  p[0] * x**n + p[1] * x**(n-1) + ... + p[n-1]*x + p[n]

        # Initial point q=p[0], dq[0]=ddq=0
        M[0][5] = 1 # q[0]
        M[1][4] = 1 # dq[0]
        M[2][3] = 2 # ddq[0]
        M[3][2] = 6 # dddq[0]

        # End of first jerk hump and start of constant acceleration has continuous q,dq,ddq and dddq=0
        M[4][0] = pow(dT[0],5); M[4][1] = pow(dT[0],4); M[4][2] = pow(dT[0],3); M[4][3] = pow(dT[0],2); M[4][4] = dT[0]; M[4][5] = 1.0 # q[0]

        M[5][0] = 5*pow(dT[0],4); M[5][1] = 4*pow(dT[0],3); M[5][2] = 3*pow(dT[0],2); M[5][3] = 2*dT[0]; M[5][4] = 1.0;

        M[6][0] = 20*pow(dT[0],3); M[6][1] = 12*pow(dT[0],2); M[6][2] = 6*dT[0]; M[6][3] = 2.0;

        M[7][0] = 60*pow(dT[0],2); M[7][1] = 24*dT[0]; M[7][2] = 6.0; # zero jerk

        # start of const accel
        M[8][8]  =  1.0 # q[0]
        M[9][7] =  1.0;
        M[10][6] =  2.0;
        M[11][6] =  0.0; # jerk

        # end of const accel
        M[12][6]  =  pow(dT[1],2); M[12][7] = dT[1]; M[12][8] = 1.0 # q[0]
        M[13][6] =  2*dT[1]; M[13][7] = 1.0;
        M[14][6] =  2.0;
        M[15][0] = 0.0


        M[16][14]  =  1.0 # q[0]
        M[17][13] =  1.0;
        M[18][12] =  2.0;
        M[19][11] =  6.0;

        #End of second jerk hump
        M[20][9] = pow(dT[2],5);M[20][10] = pow(dT[2],4); M[20][11] = pow(dT[2],3); M[20][12] = pow(dT[2],2); M[20][13] = dT[2]; M[20][14] = 1.0 # q[0]
        M[21][9] = 5*pow(dT[2],4); M[21][10] = 4*pow(dT[2],3); M[21][11] = 3*pow(dT[2],2); M[21][12] = 2*dT[2]; M[21][13] = 1.0;
        M[22][9] = 20*pow(dT[2],3); M[22][10] = 12*pow(dT[2],2); M[22][11] = 6*dT[2]; M[22][12] = 2.0; # zero accel
        M[23][0] = 60*pow(dT[2],2); M[23][10] = 24*dT[2]; M[23][11] = 6; # JERK

        # start of linear section
        M[24][16] =  1.0 # q[0]
        M[25][15] =  1.0;
        M[26][16] =  0.0 # q[0]
        M[27][15] =  0.0;

        # End of linear section
        M[28][15] = dT[3]; M[28][16] = 1.0;
        M[29][15] = 1.0
        M[30][15] = 0.0
        M[31][15] = 0.0

        return M

