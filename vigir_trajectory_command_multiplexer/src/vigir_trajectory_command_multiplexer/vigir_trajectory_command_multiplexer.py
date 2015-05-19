#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# Copyright (c) 2015, TORC Robotics.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Stuart Glaser
"""
Example:

from move_base.msg import *
rospy.init_node('foo')


from move_base.msg import *
from geometry_msgs.msg import *
g1 = MoveBaseGoal(PoseStamped(Header(frame_id = 'base_link'),
                              Pose(Point(2, 0, 0),
                                   Quaternion(0, 0, 0, 1))))
g2 = MoveBaseGoal(PoseStamped(Header(frame_id = 'base_link'),
                              Pose(Point(5, 0, 0),
                                   Quaternion(0, 0, 0, 1))))

client = ActionClient('move_base', MoveBaseAction)

h1 = client.send_goal(g1)
h2 = client.send_goal(g2)
client.cancel_all_goals()
"""

import rospy
import rostopic
import rosservice

from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import *
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers


import threading
import weakref
import time
import rospy
from rospy import Header
from actionlib_msgs.msg import *
from actionlib.exceptions import *

g_goal_id = 1


def get_name_of_constant(C, n):
    for k,v in C.__dict__.iteritems():
        if type(v) is int and v == n:
            return k
    return "NO_SUCH_STATE_%d" % n


class CommState(object):
    WAITING_FOR_GOAL_ACK = 0
    PENDING = 1
    ACTIVE = 2
    WAITING_FOR_RESULT = 3
    WAITING_FOR_CANCEL_ACK = 4
    RECALLING = 5
    PREEMPTING = 6
    DONE = 7
    LOST = 8


class TerminalState(object):
    RECALLED = GoalStatus.RECALLED
    REJECTED = GoalStatus.REJECTED
    PREEMPTED = GoalStatus.PREEMPTED
    ABORTED = GoalStatus.ABORTED
    SUCCEEDED = GoalStatus.SUCCEEDED
    LOST = GoalStatus.LOST


GoalStatus.to_string = classmethod(get_name_of_constant)
CommState.to_string = classmethod(get_name_of_constant)
TerminalState.to_string = classmethod(get_name_of_constant)


def _find_status_by_goal_id(status_array, id):
    for s in status_array.status_list:
        if s.goal_id.id == id:
            return s
    return None

## @brief Client side handle to monitor goal progress.
##
## A ClientGoalHandle is a reference counted object that is used to
## manipulate and monitor the progress of an already dispatched
## goal. Once all the goal handles go out of scope (or are reset), an
## ActionClient stops maintaining state for that goal.
class ClientGoalHandle:
    ## @brief Internal use only
    ##
    ## ClientGoalHandle objects should be created by the action
    ## client.  You should never need to construct one yourself.
    def __init__(self, comm_state_machine):
        self.comm_state_machine = comm_state_machine

        #print "GH created.  id = %.3f" % self.comm_state_machine.action_goal.goal_id.stamp.to_sec()

    ## @brief True iff the two ClientGoalHandle's are tracking the same goal
    def __eq__(self, o):
        if not o:
            return False
        return self.comm_state_machine == o.comm_state_machine

    ## @brief True iff the two ClientGoalHandle's are tracking different goals
    def __ne__(self, o):
        if not o:
            return True
        return not (self.comm_state_machine == o.comm_state_machine)


    ## @brief Sends a cancel message for this specific goal to the ActionServer.
    ##
    ## Also transitions the client state to WAITING_FOR_CANCEL_ACK
    def cancel(self):
        with self.comm_state_machine.mutex:
            cancel_msg = GoalID(stamp = rospy.Time(0),
                                id = self.comm_state_machine.action_goal.goal_id.id)
            self.comm_state_machine.send_cancel_fn(cancel_msg)
            self.comm_state_machine.transition_to(CommState.WAITING_FOR_CANCEL_ACK)

    ## @brief Get the state of this goal's communication state machine from interaction with the server
    ##
    ## Possible States are: WAITING_FOR_GOAL_ACK, PENDING, ACTIVE, WAITING_FOR_RESULT,
    ##                      WAITING_FOR_CANCEL_ACK, RECALLING, PREEMPTING, DONE
    ##
    ## @return The current goal's communication state with the server
    def get_comm_state(self):
        if not self.comm_state_machine:
            rospy.logerr("TCMX:  Trying to get_comm_state on an inactive ClientGoalHandle.")
            return CommState.LOST
        return self.comm_state_machine.state

    ## @brief Returns the current status of the goal.
    ##
    ## Possible states are listed in the enumeration in the
    ## actionlib_msgs/GoalStatus message.
    ##
    ## @return The current status of the goal.
    def get_goal_status(self):
        if not self.comm_state_machine:
            rospy.logerr("TCMX:  Trying to get_goal_status on an inactive ClientGoalHandle.")
            return GoalStatus.PENDING
        return self.comm_state_machine.latest_goal_status.status

    ## @brief Returns the current status text of the goal.
    ##
    ## The text is sent by the action server.
    ##
    ## @return The current status text of the goal.
    def get_goal_status_text(self):
        if not self.comm_state_machine:
            rospy.logerr("TCMX: Trying to get_goal_status_text on an inactive ClientGoalHandle.")
            return "ERROR: Trying to get_goal_status_text on an inactive ClientGoalHandle."
        return self.comm_state_machine.latest_goal_status.text

    ## @brief Gets the result produced by the action server for this goal.
    ##
    ## @return None if no result was receieved.  Otherwise the goal's result as a *Result message.
    def get_result(self):
        if not self.comm_state_machine:
            rospy.logerr("TCMX: Trying to get_result on an inactive ClientGoalHandle.")
            return None
        if not self.comm_state_machine.latest_result:
            #rospy.logerr("TCMX: Trying to get_result on a ClientGoalHandle when no result has been received.")
            return None
        return self.comm_state_machine.latest_result.result

    ## @brief Gets the terminal state information for this goal.
    ##
    ## Possible States Are: RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    ## This call only makes sense if CommState==DONE. This will send ROS_WARNs if we're not in DONE
    ##
    ## @return The terminal state as an integer from the GoalStatus message.
    def get_terminal_state(self):
        if not self.comm_state_machine:
            rospy.logerr("TCMX:  Trying to get_terminal_state on an inactive ClientGoalHandle.")
            return GoalStatus.LOST

        with self.comm_state_machine.mutex:
            if self.comm_state_machine.state != CommState.DONE:
                rospy.logwarn("TCMX:  Asking for the terminal state when we're in [%s]",
                             CommState.to_string(self.comm_state_machine.state))

            goal_status = self.comm_state_machine.latest_goal_status.status
            if goal_status in [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED,
                               GoalStatus.ABORTED, GoalStatus.REJECTED,
                               GoalStatus.RECALLED, GoalStatus.LOST]:
                return goal_status

            rospy.logerr("TCMX:  Asking for a terminal state, but the goal status is %d", goal_status)
            return GoalStatus.LOST


NO_TRANSITION = -1
INVALID_TRANSITION = -2
_transitions = {
    CommState.WAITING_FOR_GOAL_ACK: {
        GoalStatus.PENDING:    CommState.PENDING,
        GoalStatus.ACTIVE:     CommState.ACTIVE,
        GoalStatus.REJECTED:   (CommState.PENDING, CommState.WAITING_FOR_RESULT),
        GoalStatus.RECALLING:  (CommState.PENDING, CommState.RECALLING),
        GoalStatus.RECALLED:   (CommState.PENDING, CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTED:  (CommState.ACTIVE, CommState.PREEMPTING, CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  (CommState.ACTIVE, CommState.WAITING_FOR_RESULT),
        GoalStatus.ABORTED:    (CommState.ACTIVE, CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTING: (CommState.ACTIVE, CommState.PREEMPTING) },
    CommState.PENDING: {
        GoalStatus.PENDING:    NO_TRANSITION,
        GoalStatus.ACTIVE:     CommState.ACTIVE,
        GoalStatus.REJECTED:   CommState.WAITING_FOR_RESULT,
        GoalStatus.RECALLING:  CommState.RECALLING,
        GoalStatus.RECALLED:   (CommState.RECALLING, CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTED:  (CommState.ACTIVE, CommState.PREEMPTING, CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  (CommState.ACTIVE, CommState.WAITING_FOR_RESULT),
        GoalStatus.ABORTED:    (CommState.ACTIVE, CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTING: (CommState.ACTIVE, CommState.PREEMPTING) },
    CommState.ACTIVE: {
        GoalStatus.PENDING:    INVALID_TRANSITION,
        GoalStatus.ACTIVE:     NO_TRANSITION,
        GoalStatus.REJECTED:   INVALID_TRANSITION,
        GoalStatus.RECALLING:  INVALID_TRANSITION,
        GoalStatus.RECALLED:   INVALID_TRANSITION,
        GoalStatus.PREEMPTED:  (CommState.PREEMPTING, CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  CommState.WAITING_FOR_RESULT,
        GoalStatus.ABORTED:    CommState.WAITING_FOR_RESULT,
        GoalStatus.PREEMPTING: CommState.PREEMPTING },
    CommState.WAITING_FOR_RESULT: {
        GoalStatus.PENDING:    INVALID_TRANSITION,
        GoalStatus.ACTIVE:     NO_TRANSITION,
        GoalStatus.REJECTED:   NO_TRANSITION,
        GoalStatus.RECALLING:  INVALID_TRANSITION,
        GoalStatus.RECALLED:   NO_TRANSITION,
        GoalStatus.PREEMPTED:  NO_TRANSITION,
        GoalStatus.SUCCEEDED:  NO_TRANSITION,
        GoalStatus.ABORTED:    NO_TRANSITION,
        GoalStatus.PREEMPTING: INVALID_TRANSITION },
    CommState.WAITING_FOR_CANCEL_ACK: {
        GoalStatus.PENDING:    NO_TRANSITION,
        GoalStatus.ACTIVE:     NO_TRANSITION,
        GoalStatus.REJECTED:   CommState.WAITING_FOR_RESULT,
        GoalStatus.RECALLING:  CommState.RECALLING,
        GoalStatus.RECALLED:   (CommState.RECALLING, CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTED:  (CommState.PREEMPTING, CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  (CommState.PREEMPTING, CommState.WAITING_FOR_RESULT),
        GoalStatus.ABORTED:    (CommState.PREEMPTING, CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTING: CommState.PREEMPTING },
    CommState.RECALLING: {
        GoalStatus.PENDING:    INVALID_TRANSITION,
        GoalStatus.ACTIVE:     INVALID_TRANSITION,
        GoalStatus.REJECTED:   CommState.WAITING_FOR_RESULT,
        GoalStatus.RECALLING:  NO_TRANSITION,
        GoalStatus.RECALLED:   CommState.WAITING_FOR_RESULT,
        GoalStatus.PREEMPTED:  (CommState.PREEMPTING, CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  (CommState.PREEMPTING, CommState.WAITING_FOR_RESULT),
        GoalStatus.ABORTED:    (CommState.PREEMPTING, CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTING: CommState.PREEMPTING },
    CommState.PREEMPTING: {
        GoalStatus.PENDING:    INVALID_TRANSITION,
        GoalStatus.ACTIVE:     INVALID_TRANSITION,
        GoalStatus.REJECTED:   INVALID_TRANSITION,
        GoalStatus.RECALLING:  INVALID_TRANSITION,
        GoalStatus.RECALLED:   INVALID_TRANSITION,
        GoalStatus.PREEMPTED:  CommState.WAITING_FOR_RESULT,
        GoalStatus.SUCCEEDED:  CommState.WAITING_FOR_RESULT,
        GoalStatus.ABORTED:    CommState.WAITING_FOR_RESULT,
        GoalStatus.PREEMPTING: NO_TRANSITION },
    CommState.DONE: {
        GoalStatus.PENDING:    INVALID_TRANSITION,
        GoalStatus.ACTIVE:     INVALID_TRANSITION,
        GoalStatus.REJECTED:   NO_TRANSITION,
        GoalStatus.RECALLING:  INVALID_TRANSITION,
        GoalStatus.RECALLED:   NO_TRANSITION,
        GoalStatus.PREEMPTED:  NO_TRANSITION,
        GoalStatus.SUCCEEDED:  NO_TRANSITION,
        GoalStatus.ABORTED:    NO_TRANSITION,
        GoalStatus.PREEMPTING: INVALID_TRANSITION } }


class CommStateMachine:
    def __init__(self, action_goal, transition_cb, feedback_cb, send_goal_fn, send_cancel_fn):
        self.action_goal = action_goal
        self.transition_cb = transition_cb
        self.feedback_cb = feedback_cb
        self.send_goal_fn = send_goal_fn
        self.send_cancel_fn = send_cancel_fn

        self.state = CommState.WAITING_FOR_GOAL_ACK
        self.mutex = threading.RLock()
        self.latest_goal_status = GoalStatus(status = GoalStatus.PENDING)
        self.latest_result = None

    def __eq__(self, o):
        return self.action_goal.goal_id.id == o.action_goal.goal_id.id

    def set_state(self, state):
        rospy.logdebug("Transitioning CommState from %s to %s",
                       CommState.to_string(self.state), CommState.to_string(state))
        self.state = state

    ##
    ## @param gh ClientGoalHandle
    ## @param status_array actionlib_msgs/GoalStatusArray
    def update_status(self, status_array):
        with self.mutex:
            if self.state == CommState.DONE:
                return

            status = _find_status_by_goal_id(status_array, self.action_goal.goal_id.id)

            # You mean you haven't heard of me?
            if not status:
                if self.state not in [CommState.WAITING_FOR_GOAL_ACK,
                                      CommState.WAITING_FOR_RESULT,
                                      CommState.DONE]:
                    self._mark_as_lost()
                return

            self.latest_goal_status = status

            # Determines the next state from the lookup table
            if self.state not in _transitions:
                rospy.logerr("TCMX:  CommStateMachine is in a funny state: %i" % self.state)
                return
            if status.status not in _transitions[self.state]:
                rospy.logerr("TCMX:  Got an unknown status from the ActionServer: %i" % status.status)
                return
            next_state = _transitions[self.state][status.status]

            # Knowing the next state, what should we do?
            if next_state == NO_TRANSITION:
                pass
            elif next_state == INVALID_TRANSITION:
                rospy.logerr("TCMX:  Invalid goal status transition from %s to %s" %
                             (CommState.to_string(self.state), GoalStatus.to_string(status.status)))
            else:
                if hasattr(next_state, '__getitem__'):
                    for s in next_state:
                        self.transition_to(s)
                else:
                    self.transition_to(next_state)

    def transition_to(self, state):
        rospy.logdebug("Transitioning to %s (from %s, goal: %s)",
                       CommState.to_string(state), CommState.to_string(self.state),
                       self.action_goal.goal_id.id)
        self.state = state
        if self.transition_cb:
            self.transition_cb(ClientGoalHandle(self))

    def _mark_as_lost(self):
        self.latest_goal_status.status = GoalStatus.LOST
        self.transition_to(CommState.DONE)

    def update_result(self, action_result):
        # Might not be for us
        if self.action_goal.goal_id.id != action_result.status.goal_id.id:
            return

        with self.mutex:
            self.latest_goal_status = action_result.status
            self.latest_result = action_result

            if self.state in [CommState.WAITING_FOR_GOAL_ACK,
                              CommState.WAITING_FOR_CANCEL_ACK,
                              CommState.PENDING,
                              CommState.ACTIVE,
                              CommState.WAITING_FOR_RESULT,
                              CommState.RECALLING,
                              CommState.PREEMPTING]:
                # Stuffs the goal status in the result into a GoalStatusArray
                status_array = GoalStatusArray()
                status_array.status_list.append(action_result.status)
                self.update_status(status_array)

                self.transition_to(CommState.DONE)
            elif self.state == CommState.DONE:
                rospy.logerr("TCMX:  Got a result when we were already in the DONE state")
            else:
                rospy.logerr("TCMX:  In a funny state: %i" % self.state)

    def update_feedback(self, action_feedback):
        # Might not be for us
        if self.action_goal.goal_id.id != action_feedback.status.goal_id.id:
            return

        #with self.mutex:
        if self.feedback_cb and self.state != CommState.DONE:
            self.feedback_cb(ClientGoalHandle(self), action_feedback.feedback)


class GoalManager:

    # statuses - a list of weak references to CommStateMachine objects

    def __init__(self, ActionSpec):
        self.list_mutex = threading.RLock()
        self.statuses = []
        self.send_goal_fn = None

        try:
            a = ActionSpec()

            self.ActionSpec = ActionSpec
            self.ActionGoal = type(a.action_goal)
            self.ActionResult = type(a.action_result)
            self.ActionFeedback = type(a.action_feedback)
        except AttributeError:
            raise ActionException("Type is not an action spec: %s" % str(ActionSpec))

    def _generate_id(self):
        global g_goal_id
        id, g_goal_id = g_goal_id, g_goal_id + 1
        now = rospy.Time.now()
        return GoalID(id = "%s-%i-%.3f" % \
                          (rospy.get_caller_id(), id, now.to_sec()), stamp = now)

    def register_send_goal_fn(self, fn):
        self.send_goal_fn = fn
    def register_cancel_fn(self, fn):
        self.cancel_fn = fn

    ## Sends off a goal and starts tracking its status.
    ##
    ## @return ClientGoalHandle for the sent goal.
    def init_goal(self, goal, transition_cb = None, feedback_cb = None):
        action_goal = self.ActionGoal(header = Header(),
                                      goal_id = self._generate_id(),
                                      goal = goal)
        action_goal.header.stamp = rospy.get_rostime()

        csm = CommStateMachine(action_goal, transition_cb, feedback_cb,
                               self.send_goal_fn, self.cancel_fn)

        with self.list_mutex:
            self.statuses.append(weakref.ref(csm))

        self.send_goal_fn(action_goal)

        return ClientGoalHandle(csm)


    # Pulls out the statuses that are still live (creating strong
    # references to them)
    def _get_live_statuses(self):
        with self.list_mutex:
            live_statuses = [r() for r in self.statuses]
            live_statuses = filter(lambda x: x, live_statuses)
            return live_statuses


    ## Updates the statuses of all goals from the information in status_array.
    ##
    ## @param status_array (\c actionlib_msgs/GoalStatusArray)
    def update_statuses(self, status_array):
        live_statuses = []

        with self.list_mutex:
            # Garbage collects dead status objects
            self.statuses = [r for r in self.statuses if r()]

        for status in self._get_live_statuses():
            status.update_status(status_array)


    def update_results(self, action_result):
        for status in self._get_live_statuses():
            status.update_result(action_result)

    def update_feedbacks(self, action_feedback):
        for status in self._get_live_statuses():
            status.update_feedback(action_feedback)

class FollowJointTrajectoryActionClient:
    ## @brief Constructs an FollowTrajectoryActionClient and opens connections to an FollowTrajectoryActionServer.
    ##
    ## @param ns The namespace in which to access the action.  For
    ## example, the "goal" topic should occur under ns/goal
    ##
    ## will grab the other message types from this type.
    def __init__(self, ns, action_server, state_publish_cb):
        self.ns = ns
        self.last_status_msg = None
        self.action_server   = action_server
        self.is_running      = False

        try:
            a = FollowJointTrajectoryAction()

            self.ActionSpec      = FollowJointTrajectoryAction
            self.ActionGoal      = type(a.action_goal)
            self.ActionResult    = type(a.action_result)
            self.ActionFeedback  = type(a.action_feedback)
            self.JointTrajectory = JointTrajectory

        except AttributeError:
            raise ActionException("Type is not an action spec: %s" % str(self.ActionSpec))

        print "Defining topics for ",ns, "  --> ",rospy.remap_name(ns)

        self.pub_command   = rospy.Publisher(rospy.remap_name(ns) + '/command', self.JointTrajectory, queue_size=1)

        action_name = rospy.remap_name(ns)+"/follow_joint_trajectory";
        self.pub_goal   = rospy.Publisher(action_name + '/goal', self.ActionGoal, queue_size=1)
        self.pub_cancel = rospy.Publisher(action_name + '/cancel', GoalID, queue_size=1)

        self.manager = GoalManager(self.ActionSpec)
        self.manager.register_send_goal_fn(self.pub_goal.publish)
        self.manager.register_cancel_fn(self.pub_cancel.publish)

        self.status_sub   = rospy.Subscriber(action_name + '/status', GoalStatusArray, self._status_cb)
        self.result_sub   = rospy.Subscriber(action_name + '/result', self.ActionResult, self._result_cb)
        self.feedback_sub = rospy.Subscriber(action_name + '/feedback', self.ActionFeedback, self._feedback_cb)
        self.state_sub    = rospy.Subscriber(rospy.remap_name(ns) + '/state', JointTrajectoryControllerState, state_publish_cb)

    ## @brief Sends a goal to the action server
    ##
    ## @param goal An instance of the *Goal message.
    ##
    ## @param transition_cb Callback that gets called on every client
    ## state transition for the sent goal.  It should take in a
    ## ClientGoalHandle as an argument.
    ##
    ## @param feedback_cb Callback that gets called every time
    ## feedback is received for the sent goal.  It takes two
    ## parameters: a ClientGoalHandle and an instance of the *Feedback
    ## message.
    ##
    ## @return ClientGoalHandle for the sent goal.
    def send_goal(self, goal):#, transition_cb = None, feedback_cb = None):
        print " send_goal from ",self.ns
        return self.manager.init_goal(goal, self.server_transition_cb, self.server_feedback_cb)

    ## @brief Cancels all goals currently running on the action server.
    ##
    ## Preempts all goals running on the action server at the point
    ## that the cancel message is serviced by the action server.
    def cancel_all_goals(self):
        cancel_msg = GoalID(stamp = rospy.Time.from_sec(0.0),
                            id = "")
        self.pub_cancel.publish(cancel_msg)

    ## @brief Cancels all goals prior to a given timestamp
    ##
    ## This preempts all goals running on the action server for which the
    ## time stamp is earlier than the specified time stamp
    ## this message is serviced by the ActionServer.

    def cancel_goals_at_and_before_time(self, time):
        cancel_msg = GoalID(stamp = time, id = "")
        self.pub_cancel.publish(cancel_msg)


    ## @brief [Deprecated] Use wait_for_server
    def wait_for_action_server_to_start(self, timeout = rospy.Duration(0.0)):
        return self.wait_for_server(timeout)

    ## @brief Waits for the ActionServer to connect to this client
    ##
    ## Often, it can take a second for the action server & client to negotiate
    ## a connection, thus, risking the first few goals to be dropped. This call lets
    ## the user wait until the network connection to the server is negotiated
    def wait_for_server(self, timeout = rospy.Duration(0.0)):
        started = False
        timeout_time = rospy.get_rostime() + timeout
        while not rospy.is_shutdown():
            if self.last_status_msg:
                server_id = self.last_status_msg._connection_header['callerid']

                if self.pub_goal.impl.has_connection(server_id) and \
                        self.pub_cancel.impl.has_connection(server_id):
                    #We'll also check that all of the subscribers have at least
                    #one publisher, this isn't a perfect check, but without
                    #publisher callbacks... it'll have to do
                    status_num_pubs = 0
                    for stat in self.status_sub.impl.get_stats()[1]:
                        if stat[4]:
                            status_num_pubs += 1

                    result_num_pubs = 0
                    for stat in self.result_sub.impl.get_stats()[1]:
                        if stat[4]:
                            result_num_pubs += 1

                    feedback_num_pubs = 0
                    for stat in self.feedback_sub.impl.get_stats()[1]:
                        if stat[4]:
                            feedback_num_pubs += 1

                    if status_num_pubs > 0 and result_num_pubs > 0 and feedback_num_pubs > 0:
                        started = True
                        break

            if timeout != rospy.Duration(0.0) and rospy.get_rostime() >= timeout_time:
                break

            time.sleep(0.01)

        return started

    def _status_cb(self, msg):
        self.last_status_msg = msg
        self.manager.update_statuses(msg)
        #print "Status CB: ",self.ns," :\n ",msg,"\n\n"

    def _result_cb(self, msg):
        self.manager.update_results(msg)
        if self.action_server.is_active():
            if GoalStatus.RECALLED == msg.status.status:
                rospy.loginfo("TCMX:   Recalled goal by %s" % self.ns)
                self.action_server.set_aborted(msg.result)
            elif GoalStatus.REJECTED == msg.status.status:
                rospy.loginfo("TCMX:   REJECTED goal  by %s" % self.ns)
                self.action_server.set_aborted(msg.result)
            elif GoalStatus.PREEMPTED == msg.status.status:
                rospy.loginfo("TCMX:   PREEMPTED goal by %s" % self.ns)
                self.action_server.set_aborted(msg.result)
            elif GoalStatus.ABORTED == msg.status.status:
                rospy.loginfo("TCMX:   ABORTED goal by %s" % self.ns)
                self.action_server.set_aborted(msg.result)
            elif GoalStatus.SUCCEEDED == msg.status.status:
                rospy.loginfo("TCMX:   SUCCEEDED goal by %s" % self.ns)
                self.action_server.set_succeeded(msg.result)
            elif GoalStatus.LOST == msg.status.status:
                rospy.loginfo("TCMX:   LOST goal by %s" % self.ns)
                self.action_server.set_aborted(msg.result)
            else:
                rospy.loginfo("TCMX:   Unknown status=%d by %s" % (msg.status.status, self.ns))
                self.action_server.set_aborted(msg.result)
        else:
            rospy.loginfo("TCMX:  Action server for %s is NOT active!" % self.ns)

    def _feedback_cb(self, msg):
        self.manager.update_feedbacks(msg)
        self.action_server.publish_feedback(msg.feedback);

    def server_transition_cb(self, gh):
        rospy.loginfo("TCMX:    controller %s  transition %s" % (self.ns, gh))

    def server_feedback_cb(self, gh, msg):
        rospy.loginfo("TCMX:    server_feedback_cb - controller %s  feedback %s" % (self.ns, msg))
        self.action_server.publish_feedback(msg.feedback)


class VigirJointTrajectoryControllerInterface(object):

    def __init__(self, controller, topic_name, action_server, state_publish_cb):

        rospy.loginfo("TCMX:  Add VigirJointTrajectoryControllerInterface for  %s  (%s) -> %s" % (controller.name, controller.state,topic_name))

        self._client = FollowJointTrajectoryActionClient(topic_name, action_server, state_publish_cb )
        # subscribe to feedback, state, result, and status topics

        self._name = controller.name
        self._current_state = controller.state
        self._running = controller.state == 'running'

    def is_running(self):
        return self._running

    def set_controller_state(self, state):
        if self._current_state != state:
            self._running = state == 'running'
            self._current_state = state
            rospy.loginfo("TCMX:    Controller %s is now in state (%s) - running=%d" % (self._name, self._current_state, self._running))

    def set_goal(self, goal):
        if self._running:
            rospy.loginfo("TCMX:    Send goal to %s  " % self._name)
            self._client.send_goal(goal)
            return True
        else:
            rospy.logerr("TCMX:    Controller not running - abort goal to %s  " % self._name)
            return False

    def preempt_goal(self):
        rospy.loginfo("TCMX:    Preempt goal from %s by calling cancel_all_goals ... " % self._name)
        self._client.cancel_all_goals()
        return True


class VigirTrajectoryCommandInterface(object):
    """
    This allows single action to interface to multiple controllers based on which one is running at current time.
    """
    def __init__(self, action_name, target_namespace, appendage_name,action_server):

        # Store the limits used to smooth each joint as a map from joint name to simple limits structure
        # For now we don't handle time factor, and dynamic reparameterization
        self._action_name = action_name
        self._target_namespace = target_namespace
        self._appendage_name = appendage_name
        self._as = action_server

        self.pub_state = rospy.Publisher(action_name + '/state', JointTrajectoryControllerState, queue_size=1)

        self.get_controller_list_service = None
        self.controllers = {}
        self.active_controller = None

        rospy.loginfo("TCMX:   Get the initial list of active controllers ...")
        self.update_running_controllers(None)

        rospy.loginfo("TCMX:   Initialized the trajectory command interface!")

    def publish_state_cb(self, msg):
        self.pub_state.publish(msg)

    def source_goal_callback(self):
        rospy.loginfo("TCMX:  vvvvv   source_goal_callback for %s vvvvvvvvv" % self._action_name)
        goal = self._as.accept_new_goal()
        self.update_running_controllers(None)
        if self.active_controller is not None:
            self.active_controller.set_goal(goal)
        else:
            rospy.logwarn("TCMX:  source_goal_callback - No active controllers for %s" % self._action_name)
            result = FollowJointTrajectoryResult()
            result.error_string = "No active controllers"
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self._as.set_aborted(result)

        rospy.loginfo("TCMX:  ^^^^^^^  done source_goal_callback for %s ! ^^^^^^^^^" % self._action_name)

    def source_preempt_callback(self):
        print "-vvvvvvvvvvv---source_preempt_callback---vvvvvvvvvvvvvvvvv---"
        if self.active_controller is not None:
            self.active_controller.preempt_goal()
        else:
            rospy.logwarn("TCMX:  source_preempt_callback - No active controllers for %s" % self._action_name)
        print "---^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^----------------"

    def source_command_callback(self, msg):
        if self.active_controller is not None:
            self.active_controller._client.pub_command.publish(msg)
        else:
            rospy.logwarn("TCMX:  source_command_callback: No active controllers for %s" % self._action_name)

        return

    def shutdown(self):
        print "Shutting down the VigirTrajectoryCommandInterface for ", self._action_name, "!"

    def update_running_controllers(self, event):
        # update the list of currently initialized controllers
        # we will assume that once a controller is initialized it remains in the list until shutdown.

        #rospy.loginfo("TCMX:   Update the controllers list ")

        controllers = self.get_active_traj_controllers(self._target_namespace)

        self.active_controller = None  # Reset the active controller list

        if controllers is None:
            rospy.logwarn("TCMX:  No controllers returned - abort!")
            return

        for controller in controllers:
            if not (controller.name in self.controllers):
                if self._appendage_name in controller.name:
                    print "  Adding ", controller.name, " to controller list ..."
                    self.controllers[controller.name] = \
                        VigirJointTrajectoryControllerInterface(controller, self._target_namespace + '/' + controller.name, self._as, self.publish_state_cb)
                    if controller.state == 'running':
                        if self.active_controller is None:
                            self.active_controller = self.controllers[controller.name]
                        else:
                            rospy.logwarn("TCMX:  Controller %s is already listed as active - ignore %s" % (self.active_controller.name, controller.name))
                # else:
                #    print "Controller ",controller.name, " is not for this appendage (",self._appendage_name,")"
            else:
                #print "  ",controller.name," already exists in dictionary!"
                self.controllers[controller.name].set_controller_state(controller.state)
                if controller.state == 'running':
                    if self.active_controller is None:
                        self.active_controller = self.controllers[controller.name]
                    else:
                        rospy.logwarn("TCMX:   Controller %s is already listed as running - ignore %s" % (self.active_controller.name, controller.name))

    # @staticmethod
    # def get_traj_controllers(namespace, controller_list):
    #    time_list = []
    #    for controller in controller_list:
    #        start = time.clock()
    #        topic_type = rostopic.get_topic_type(namespace + '/' + controller.name + '/command')[0]
    #        end = time.clock()
    #        time_list.append(end-start)
    #        if topic_type == 'trajectory_msgs/JointTrajectory':
    #            yield controller
    #    print "Mean to check topic type:", (sum(time_list)/len(time_list)), "Times:", len(time_list)

    def get_active_traj_controllers(self, namespace):
        if self.get_controller_list_service is None:
            try:
                # rospy.loginfo("TCMX:    Waiting for %s to get active trajectory controllers ..."%(list_service))
                # rospy.wait_for_service(list_service,)
                list_service = namespace + "/controller_manager/list_controllers"
                self.get_controller_list_service = rospy.ServiceProxy(list_service, ListControllers)
            except rospy.ServiceException as exc:
                rospy.logerr("TCMX:  Failed to connect to the controller action service %s - failed: %s" % (list_service, exc))
                return None

        try:
            controller_list = self.get_controller_list_service().controller
        except rospy.ServiceException as exc:
            rospy.logerr("TCMX:  Retrieving controller list on namespace %s : failed %s" % (namespace, exc))
            self.get_controller_list_service = None
            return None

        # active_traj_controllers = list(VigirTrajectoryCommandInterface.get_traj_controllers(namespace, controller_list))
        # controller_names = [controller.name for controller in active_traj_controllers]
        # print 'Active traj controllers:', controller_names
        return controller_list


