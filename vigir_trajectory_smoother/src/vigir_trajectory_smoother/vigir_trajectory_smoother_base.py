import rospy
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from copy import deepcopy
import time

import vigir_trajectory_utility  as traj_utility
from  vigir_trajectory_piecewise_smoother import VigirTrajectoryPiecewiseSmoother
##from  vigir_trajectory_quartic_smoother import VigirTrajectoryQuarticSmoother
##from  vigir_trajectory_cubic_smoother import VigirTrajectoryCubicSmoother
from  vigir_trajectory_c3_smoother import VigirTrajectoryC3Smoother
from vigir_trajectory_plotting import VigirTrajectoryPlotter

"""
Generic smoothing code takes a JointTrajectory.msg as input, and recalculates the velocity and acceleration
to yeild a piecewise c3 smooth quintic spline trajectory.  The trajectory is assumed to end at rest.
"""
class VigirTrajectorySmoother(object):
    def __init__(self, joint_limits,time_limit = 1.0, max_iterations=50, test_trajectories=True, plot_trajectories=False):

        # Store the limits used to smooth each joint as a map from joint name to simple limits structure
        # For now we don't handle time factor, and dynamic reparameterization
        self.joint_limits      = joint_limits
        self.time_limit        = time_limit
        self.max_iterations    = max_iterations
        self.plot_trajectories = plot_trajectories
        self.test_trajectories = test_trajectories

        # Define which type of smoothing we will use
        self.piecewise_smoother   = VigirTrajectoryPiecewiseSmoother(joint_limits, time_limit, max_iterations)
        ##self.quartic_smoother     = VigirTrajectoryQuarticSmoother(joint_limits, time_limit, max_iterations)
        self.cubic_smoother       = VigirTrajectoryC3Smoother(joint_limits, time_limit, max_iterations)

        self.plotter = None
        if (self.plot_trajectories):
            self.plotter = VigirTrajectoryPlotter()

    def shutdown():
        print "Shutting down the VigirTrajectorySmoother!"

    def reparameterize(self, old_trajectory):
      try:
        count = 0
        elapsed = 0.0
        violation = True # Assume we violate a limit to start
        continue_flag = True

        start_time = time.clock()

        #if len(old_trajectory.points[0].accelerations) > 0:
        #    print  "Assume valid quintic spline - no reparameterization!"
        #    return old_trajectory

        #if len(old_trajectory.points[0].velocities) > 0:
        #    print  "Assume valid cubic spline - no reparameterization!"
        #    return old_trajectory

        new_trajectory = deepcopy(old_trajectory)

        if (len(new_trajectory.points[0].velocities) == 0):
            #print " Old trajectory does not have velocities - add for smoothing"
            for point in new_trajectory.points:
                point.velocities = [0.0 for x in xrange(len(point.positions)) ]

        if (len(new_trajectory.points[0].accelerations) == 0):
            #print " Old trajectory does not have accelerations - add for smoothing"
            for point in new_trajectory.points:
                point.accelerations = [0.0 for x in xrange(len(point.positions)) ]

        if (len(old_trajectory.points[0].velocities) == 0):
            # Call the specific reparameterization defined by smoother
            new_trajectory = self.piecewise_smoother.reparameterize(new_trajectory)
        else:
            # If we have velocities or accelerations, then assume this is a MoveIt! parabolicly smoothed trajectory or native cubic
            new_trajectory = self.cubic_smoother.reparameterize(new_trajectory)

        ############################################################################################################################
        try:
          if (self.test_trajectories or self.plot_trajectories):
            print "vvvvvvvvvvvvvvvvvvvv Test vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
            spline_list, dT_new, num_segments2, num_joints = traj_utility.define_trajectory_splines(new_trajectory)

            if (self.test_trajectories):
                print "Test trajectory splines ..."
                dT = deepcopy(dT_new)
                violation = False
                for jnt, joint_name in enumerate(new_trajectory.joint_names):
                    jnt_violation, dT_new = self.check_limits(dT, spline_list[jnt], dT_new, joint_name)
                    #if (jnt_violation):
                    #    print "Limit violation for ",joint_name

                    violation = violation or jnt_violation

                if (violation):
                    print "     At least one joint violation in final trajectory!"

            if (self.plot_trajectories):
                self.plotter.plot_splines_list(spline_list, dT, new_trajectory.joint_names,show_plot=False)

            print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
        except Exception as msg:
            print "Exception during trajectory testing after smoother"
            ##print "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
            ##print new_trajectory
            ##print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
            print "     ",msg
            traj_utility.PrintException()
            sys.exit(-1)
        ############################################################################################################################


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
        This code evaluates the splines at each segment to determine maximum velocity and acceleration within the segment,
        then checks against limits.  If limits are violated, then the time interval is increase by a proportional factor
    """
    def check_limits(self, dT, jnt_splines, dT_new, joint_name):

        #print "  check ",joint_name
        jnt_violation = False
        for seg in range(0,len(dT)):

            seg_violation, dt_new = self.joint_limits[joint_name].check_segment_limits(jnt_splines[seg], dT[seg], joint_name, seg )
            #print "     seg ",seg," : ",seg_violation
            jnt_violation = jnt_violation or seg_violation
            if (seg_violation):
                dT_new[seg] = dt_new
        #print "    violation=",jnt_violation
        return jnt_violation, dT_new

