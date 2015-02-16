import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from copy import deepcopy
import time
import vigir_trajectory_utility  as traj_utility
import matplotlib.pyplot         as plt

"""
Generic code takes a JointTrajectory.msg as input, and plots the quintic spline interpolation to the screen.
"""
class VigirTrajectoryPlotter(object):
    def __init__(self):

        self.colors=['r','g','b','m','c']
        self.style=['+','x','o']
        self.time_step = 0.0002

    def shutdown():
        print "Shutting down the VigirTrajectorySmoother!"

    def plot_trajectories(self, trajectories, joint_names,show_plot=True):
      try:

        print " Get spline list for each trajectory"
        master_spline_list = []
        for trajectory in trajectories:
            # Generate spline data
            spline_list, dT, num_segments, num_joints = traj_utility.define_trajectory_splines(trajectory)
            master_spline_list.append( (spline_list, dT) )

        print " Plot per joint ..."
        for jnt, joint_name in enumerate(joint_names):
            # define figure and axes
            print "Plot trajectories for joint ", joint_name
            fig = plt.figure()
            ax1 = fig.add_subplot(141)
            ax2 = fig.add_subplot(142)
            ax3 = fig.add_subplot(143)
            ax4 = fig.add_subplot(144)
            axes = (ax1, ax2, ax3, ax4)
            for ax in axes:
                ax.hold(True)

            for traj in range(len(trajectories)):
                #print "    plot joint splines to figure ..."

                splines = master_spline_list[traj][0][jnt]
                dT      = master_spline_list[traj][1]
                self.plot_splines(axes, splines, dT, self.colors[traj%len(self.colors)])
                self.plot_knots(axes, splines, dT, self.colors[traj%len(self.colors)], self.style[traj%len(self.style)])
            fig.suptitle(joint_name)
        if (show_plot):
            plt.show()
        print "Done plotting!"

      except Exception as msg:
            print "Exception in VigirTrajectoryPlotter.plot_trajectories"
            print msg

    def plot_splines_list(self, splines_list, dT, joint_names,show_plot=True):
      try:

        print " Plot per joint ..."
        for jnt, joint_name in enumerate(joint_names):
            # define figure and axes
            print "Plot trajectories for joint ", joint_name
            fig = plt.figure()
            ax1 = fig.add_subplot(141)
            ax2 = fig.add_subplot(142)
            ax3 = fig.add_subplot(143)
            ax4 = fig.add_subplot(144)
            axes = (ax1, ax2, ax3,ax4)
            for ax in axes:
                ax.hold(True)

            splines = splines_list[jnt]
            self.plot_splines(axes, splines, dT, self.colors[0])
            self.plot_knots(axes, splines, dT, self.colors[0], self.style[0])
            fig.suptitle(joint_name)
        if (show_plot):
            plt.show()
        print "Done plotting!"

      except Exception as msg:
            print "Exception in VigirTrajectoryPlotter.plot_splines_list"
            print msg

    def plot_splines(self, axes, splines, dT, clr):
      try:

        #print "Initialize vectors for plot"
        time=None
        q=None
        dq=None
        ddq=None

        # Derivatives
        vel  = np.zeros(6); vel[0]  =  5.0;  vel[1] = 4.0;  vel[2] = 3.0; vel[3] = 2.0; vel[4]=1.0
        acc  = np.zeros(6); acc[0]  = 20.0;  acc[1] =12.0;  acc[2] = 6.0; acc[3] = 2.0;
        jerk = np.zeros(6); jerk[0] = 60.0; jerk[1] =24.0; jerk[2] = 6.0;

        start_time = 0.0
        for seg,dt in enumerate(dT):
            #print " define polynomials ..."
            p_pos  =  splines[seg]            # quintic
            p_vel  = (splines[seg]*vel)[:-1]  # quartic
            p_acc  = (splines[seg]*acc)[:-2]  # cubic
            p_jerk = (splines[seg]*jerk)[:-3] # quadratic

            #print "create time interval..."
            t_seg   = list(np.linspace(0,dt,int(np.ceil(dt/self.time_step))))

            q_seg    = np.polyval(p_pos,t_seg)
            dq_seg   = np.polyval(p_vel,t_seg)
            ddq_seg  = np.polyval(p_acc,t_seg)
            dddq_seg = np.polyval(p_jerk,t_seg)

            t_seg = np.add(t_seg, start_time)

            start_time += dt

            if (time is None):
                # Initialize the vectors
                time = t_seg
                q    = q_seg
                dq   = dq_seg
                ddq  = ddq_seg
                dddq = dddq_seg
            else:
                # Concatenate the vectors
                time=np.append(time, t_seg)
                q   =np.append(q   , q_seg)
                dq  =np.append(dq  , dq_seg)
                ddq =np.append(ddq , ddq_seg)
                dddq =np.append(dddq, dddq_seg)

        #print "    plotting pos, vel, acc with color=",clr
        p1 = axes[0].plot(time,   q, clr, label="pos")
        p2 = axes[1].plot(time,  dq, clr, label="vel")
        p3 = axes[2].plot(time, ddq, clr, label="acc")
        p4 = axes[3].plot(time,dddq, clr, label="jerk")

      except Exception as msg:
        print "Exception during test_plot!"
        print msg

    def plot_knots(self, axes, splines, dT, clr, style):
      try:
          time=None
          q=None
          dq=None
          ddq=None

          # Derivatives
          vel  = np.zeros(6); vel[0]  =  5.0;  vel[1] = 4.0;  vel[2] = 3.0; vel[3] = 2.0; vel[4]=1.0
          acc  = np.zeros(6); acc[0]  = 20.0;  acc[1] =12.0;  acc[2] = 6.0; acc[3] = 2.0;
          jerk = np.zeros(6); jerk[0] = 60.0; jerk[1] =24.0; jerk[2] = 6.0;

          start_time = 0.0
          for seg,dt in enumerate(dT):
              #print " define polynomials ..."
              p_pos  = splines[seg]             # quintic
              p_vel  = (splines[seg]*vel)[:-1]  # quartic
              p_acc  = (splines[seg]*acc)[:-2]  # cubic
              p_jerk = (splines[seg]*jerk)[:-3] # quadratic

              #print "create time interval..."
              t_seg   = 0.0

              q_seg    = np.polyval(p_pos,t_seg)
              dq_seg   = np.polyval(p_vel,t_seg)
              ddq_seg  = np.polyval(p_acc,t_seg)
              dddq_seg = np.polyval(p_jerk,t_seg)

              t_seg = start_time

              start_time += dt

              if (time is None):
                  # Initialize the vectors
                  time = t_seg
                  q    = q_seg
                  dq   = dq_seg
                  ddq  = ddq_seg
                  dddq = dddq_seg
              else:
                  # Concatenate the vectors
                  time=np.append(time, t_seg)
                  q   =np.append(q   , q_seg)
                  dq  =np.append(dq  , dq_seg)
                  ddq =np.append(ddq , ddq_seg)
                  dddq =np.append(dddq, dddq_seg)

          print "    plotting pos, vel, acc with color=",clr
          p1 = axes[0].plot(time,   q, clr+style, label="pos")
          p2 = axes[1].plot(time,  dq, clr+style, label="vel")
          p3 = axes[2].plot(time, ddq, clr+style, label="acc")
          p4 = axes[3].plot(time,dddq, clr+style, label="acc")
      except Exception as msg:
          print "plot_splines failed :"+msg
          return

