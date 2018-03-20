#! /usr/bin/env python

import rospy

from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionServer, SimpleActionClient
from ras_msgs.msg import GotoAction, GotoGoal
from ras_msgs.msg import DoErrorAction, DoErrorFeedback, DoErrorResult
from ras_msgs.msg import TabletGotoAction, TabletGotoFeedback, TabletGotoResult
from tablet_interface.srv import Tablet

class SchedulerServer:

    def __init__(self):
        self.is_goto_active = False
        self.success = False
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.shutdown)

        self.do_error = SimpleActionServer(
            'do_error', DoErrorAction,
            execute_cb=self.do_error_execute,
            auto_start=False)

        # Called from the tablet when we want to go to a particular object
        # This then forwards it to our Go To node via the self.goto_client
        self.tablet_goto = SimpleActionServer(
            'tablet_response', TabletGotoAction,
            execute_cb=self.tablet_goto_execute,
            auto_start=False)
        self.tablet_goto.start()

        # Forward commands through our Go To node
        self.goto_client = SimpleActionClient(
            'goto', GotoAction)
        self.do_error.start()
        self.goto_client.wait_for_server()

        self.tablet_goto.start()
        self.do_error_start()

        # Tablet data
        self.task_number = 0
        self.error_step = 0
        self.audio_url = ""
        self.face_url = ""
        self.object = ""
        self.video_step_url = ""
        self.video_full_url = ""

    def shutdown(self):
        if self.is_goto_active:
            self.goto_client.cancel_goal()

    def error_data(self, task_number, error_step):
        """
        From the task number and error step, get the corresponding object name
        and video urls
        """
        object_to_find = '' # No object for this error / step
        video_step_url = ''
        video_full_url = ''

        basename = 'http://casas.wsu.edu/smarthomestats/video/'

        # Water plants
        if task_number == 0:
           video_full_url = basename + 'waterplants.all.mp4'
           if error_step in [0, 1, 4, 5]:
               video_step_url = basename + 'waterplants.error1.mp4'
               object_to_find = 'watercan'
           elif error_step == 2:
               video_step_url = basename + 'waterplants.error2.mp4'
               object_to_find = 'plantcoffee'
           elif error_step == 3:
               video_step_url = basename + 'waterplants.error3.mp4'
               object_to_find = "plantside"
        # Take Meds
        elif task_number == 1:
            video_full_url = basename + 'takemedication.all.mp4'
            if error_step in [0, 5, 11]:
                video_step_url = basename + 'takemedication.error1.mp4'
                object_to_find = 'food'
            elif error_step in [1, 2, 7, 10]:
                video_step_url = basename + 'takemedication.error2.mp4'
                object_to_find = 'glass'
            elif error_step in [3, 6, 9]:
                video_step_url = basename + 'takemedication.error3.mp4'
                object_to_find = 'pillbottle'
        # Walk the dog
        elif task_number == 2:
            video_full_url = basename + 'walkdog.all.mp4'
            if error_step == 0:
                video_step_url = basename + 'walkdog.error1.mp4'
                object_to_find = 'umbrella'
            elif error_step == 1:
                video_step_url = basename + 'walkdog.error2.mp4'
                object_to_find = 'leash'
            elif error_step == 2:
                video_step_url = basename + 'walkdog.error3.mp4'
                object_to_find = 'keys'
            elif error_step == 3:
                video_step_url = basename + 'walkdog.error4.mp4'
                object_to_find = 'dog'

        return object_to_find, video_step_url, video_full_url

    def tablet_command(self, screen, object_name, face_url, video_step_url,
            video_full_url, audio_url):
        """
        Command the tablet to switch to a particular screen
        """
        rospy.wait_for_service("tablet")

        rospy.loginfo("Commanding tablet: s: %s o: %s f: %s vs: %s vf: %s a: %s",
                screen, object_name, face_url, video_step_url,
                video_full_url, audio_url)

        try:
            query = rospy.ServiceProxy("tablet", Tablet)
            results = query(screen, object_name, face_url, video_step_url,
            video_full_url, audio_url)
            return results.success
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        return False

    def tablet_goto_execute(self, goal):
        """
        Handle the response from what the user clicked on the tablet
        """
        success = True
        response = goal.response
        rospy.loginfo("Got message from tablet: "+response)

        if response == "audioplay":
            pass
        elif response == "videoplay":
            pass
        elif response == "yes":
            pass
        elif response == "no":
            pass
        elif response == "videodone":
            # Return to the options screen
            self.tablet_command("options", self.object, self.face_url,
                    self.video_step_url, self.video_full_url, self.audio_url)
        elif response == "audiodone":
            pass
        elif response == "watchfull":
            pass
        elif response == "watchstep":
            pass
        elif response == "goto":
            # Show options but without the go to object button
            self.object = ""
            self.tablet_command("options", self.object, self.face_url,
                    self.video_step_url, self.video_full_url, self.audio_url)

            # Find the object
            if self.goto(self.object, self.task_number, self.error_step):
                while not self.is_goto_active:
                    self.rate.sleep()

                tablet_goto_feedback = TabletGotoFeedback()
                tablet_goto_feedback.status = 1
                tablet_goto_feedback.text = "SENT GOTO GOAL TO TURTLEBOT"
                self.tablet_goto.publish_feedback(tablet_goto_feedback)
                self.rate.sleep()

                while self.is_goto_active:
                    tablet_goto_feedback.status = 2
                    tablet_goto_feedback.text = "TURTLEBOT NAVIGATING"
                    self.tablet_goto.publish_feedback(tablet_goto_feedback)
                    self.rate.sleep()

                tablet_goto_feedback.status = 3
                tablet_goto_feedback.text = "TURTLEBOT COMPLETED TASK"
                self.tablet_goto.publish_feedback(tablet_goto_feedback)

                if self.success:
                    rospy.loginfo("Found object")
                else:
                    rospy.loginfo("Did not find object")
            else:
                # It's "done", but it failed
                success = False
        else:
            pass

        tablet_goto_result = TabletGotoResult()
        tablet_goto_result.success = success
        self.tablet_goto.set_succeeded(tablet_goto_result)

    def do_error_execute(self, goal):
        """
        What to do when error detection calls this action server, when an error
        is detected
        """
        rospy.loginfo("Executing Do Error")

        # For use on tablet
        self.audio_url = "" # TODO nobody recorded the audio files yet?
        self.face_url = "happy-cartoon-face-hi.png"
        self.object, self.video_step_url, self.video_full_url = \
                self.error_data(goal.task_number, goal.error_step)
        self.task_number = goal.task_number
        self.error_step = goal.error_step

        # Command tablet
        self.tablet_command("choice", self.object, self.face_url,
                self.video_step_url, self.video_full_url, self.audio_url)

        # Find the human (type 1)
        if self.goto("human", goal.task_number, goal.error_step):
            while not self.is_goto_active:
                self.rate.sleep()

            do_error_feedback = DoErrorFeedback()
            do_error_feedback.status = 1
            do_error_feedback.text = "SENT GOTO GOAL TO TURTLEBOT"
            self.do_error.publish_feedback(do_error_feedback)
            self.rate.sleep()

            while self.is_goto_active:
                do_error_feedback.status = 2
                do_error_feedback.text = "TURTLEBOT NAVIGATING"
                self.do_error.publish_feedback(do_error_feedback)
                self.rate.sleep()

            do_error_feedback.status = 3
            do_error_feedback.text = "TURTLEBOT COMPLETED TASK"
            self.do_error.publish_feedback(do_error_feedback)

            do_error_result = DoErrorResult()
            do_error_result.status = 3
            do_error_result.is_complete = True
            self.do_error.set_succeeded(do_error_result)

            if self.success:
                rospy.loginfo("Found human")
            else:
                rospy.loginfo("Did not find human")
        else:
            # It's "done", but it failed
            do_error_result = DoErrorResult()
            do_error_result.status = 3
            do_error_result.is_complete = True
            self.do_error.set_succeeded(do_error_result)

    def goto(self, object_name, task_number, error_step):
        """
        From the specified server, try going to the specified object
        """
        if object_name == "base1":
            goto_type = 0
        elif object_name == "base2":
            goto_type = 0
        elif object_name == "human":
            goto_type = 1
        else:
            goto_type = 2

        if self.is_goto_active:
            rospy.logerr("Cannot navigate to two places at once!")
            return False

        goto_goal = GotoGoal()
        goto_goal.type = goto_type
        goto_goal.task_number = task_number
        goto_goal.error_step = error_step
        goto_goal.error_object = error_step
        self.goto_client.send_goal(
            goto_goal,
            done_cb=self.goto_done_cb,
            active_cb=self.goto_active_cb,
            feedback_cb=self.goto_feedback_cb)

        return True

    def goto_done_cb(self, terminal_state, result):
        self.is_goto_active = False
        if terminal_state == GoalStatus.RECALLED:
            self.success = False
            status = "RECALLED"
        elif terminal_state == GoalStatus.REJECTED:
            self.success = False
            status = "REJECTED"
        elif terminal_state == GoalStatus.PREEMPTED:
            self.success = False
            status = "PREMEPTED"
        elif terminal_state == GoalStatus.ABORTED:
            self.success = False
            status = "ABORTED"
        elif terminal_state == GoalStatus.SUCCEEDED:
            self.success = True
            status = "SUCCEEDED"
        elif terminal_state == GoalStatus.LOST:
            self.success = False
            status = "LOST"
        else:
            self.success = False
            status = "UNKNOWN"
        rospy.loginfo("terminal state: {}  result: ({}, {})".format(
            status, result.status, result.is_complete))

    def goto_active_cb(self):
        self.is_goto_active = True

    def goto_feedback_cb(self, feedback):
        rospy.loginfo("goto feedback: x={}, y={}, z={}, status={}".format(
            feedback.x, feedback.y, feedback.z, feedback.text))


if __name__ == '__main__':
    try:
        rospy.init_node('scheduler_server')
        server = SchedulerServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
