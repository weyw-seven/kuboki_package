#!/usr/bin/python3

from os.path import dirname, abspath
import sys
sys.path.append(dirname(dirname(abspath(__file__))))
import subprocess, time

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
import actionlib_msgs.msg

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
from ui.robot_tools_mainWindow import Ui_mainWindow


class MyMainWindow(QMainWindow, Ui_mainWindow):
    def __init__(self, parent=None):
        super(MyMainWindow, self).__init__(parent) # qt
        self.setupUi(self)
        
        rospy.init_node('robot_tools', anonymous=True, disable_signals=True)
        self.centralwidget.destroyed.connect(lambda:rospy.signal_shutdown("GUI closed")) # tell ros to close
        self.load_parameters()

    
    def load_parameters(self):
        """load parameters from ros param server
        Variables created here:
            self.batteryVoltage   # int, track the battery info
            self.timer_stayHere   # QTimer, periodocally publish twist 0
            self.map_topic        # str, topic to get the map
            self.map_folder       # str, folder to save the map
            self.rtab_reset       # func, call rtabmap reset service
            self.call_rtab_pause  # func, call rtabmap pause service
            self.call_rtab_resume # fucn, call rtabmap resume service
            self.call_rtab_mode_local   # func, call ratbmap set mode localization
            self.call_rtab_mode_mapping # fucn, call ratbmap set mode mapping
            self.pub_cancelGoal   # ros publisher, cancel move_base goal
            self.pub_cmdVel       # ros publisher, publish vel command
        """
        self.timer_stayHere = QTimer(self)
        self.timer_stayHere.timeout.connect(lambda: self.pub_cmdVel.publish(Twist()))
        

        # get ros param
        voltage_min  = rospy.get_param('~voltage_min', 132)
        voltage_max  = rospy.get_param('~voltage_max', 163)
        sensor_topic = rospy.get_param('~sensor_topic', '/mobile_base/sensors/core')
        cancel_goal_topic = rospy.get_param('~cancel_goal_topic', '/move_base/cancel')
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel_mux/input/teleop')
        rtab_reset_service  = rospy.get_param('~rtab_reset_service',  '/rtabmap/reset')
        rtab_pause_service  = rospy.get_param('~rtab_pause_service',  '/rtabmap/pause')
        rtab_resume_service = rospy.get_param('~rtab_resume_service', '/rtabmap/resume')
        rtab_set_mode_local   = rospy.get_param('~rtab_set_mode_localization_service', '/rtabmap/set_mode_localization')
        rtab_set_mode_mapping = rospy.get_param('~rtab_set_mode_mapping_service',      '/rtabmap/set_mode_mapping')
        self.map_topic  = rospy.get_param('~map_topic', '/map')
        self.map_folder = rospy.get_param('~map_folder', '~')

        # UI setting
        self.batteryVoltage = voltage_max
        self.pb_voltage.setMinimum(voltage_min)
        self.pb_voltage.setMaximum(voltage_max)
        self.pb_voltage.setValue(voltage_max)

        # ros publish
        self.pub_cancelGoal = rospy.Publisher(cancel_goal_topic, actionlib_msgs.msg.GoalID, queue_size = 1)
        self.pub_cmdVel = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 1)

        # ros subscribe
        rospy.Subscriber(sensor_topic, SensorState, self.sensorStateCallback)

        # ros service proxy
        self.call_rtab_reset = rospy.ServiceProxy(rtab_reset_service, Empty)
        self.call_rtab_pause = rospy.ServiceProxy(rtab_pause_service, Empty)
        self.call_rtab_resume = rospy.ServiceProxy(rtab_resume_service, Empty)
        self.call_rtab_mode_local = rospy.ServiceProxy(rtab_set_mode_local, Empty)
        self.call_rtab_mode_mapping = rospy.ServiceProxy(rtab_set_mode_mapping, Empty)
        
    
    def sensorStateCallback(self, data):
        """related to battery sensor callback"""
        # only change GUI if battery voltage changed
        if self.batteryVoltage > data.battery:
            self.batteryVoltage = data.battery  # in 0.1V
            self.pb_voltage.setValue(data.battery)
            self.lb_voltage.setText('{:.1f}'.format(data.battery / 10))

    def btn_resetMapCallback(self):
        """reset Rtabmap's already mapped data"""
        try:
            self.call_rtab_reset()
        except rospy.ServiceException as e:
            rospy.logerr('reset map failed: %s' % e)

    def btn_saveMapCallback(self):
        """save the map to your home folder"""
        CMD = "rosrun map_server map_saver -f {} map:={}".format(
            self.map_folder + '/' + time.strftime('%Y-%m-%d_%H-%M-%S'),
            self.map_topic)
        rospy.loginfo("command: " + CMD)
        out = subprocess.getstatusoutput(CMD)
        # check result
        if out[0]:
            rospy.logerr('Cannot save map. The command executed is %s' % CMD)
        else:
            rospy.loginfo('map is saved successfully.')
        
    
    def btn_rtabPauseCallback(self, pause):
        """pause/resume Rtabmap"""
        try:
            if pause:
                self.btn_rtabPause.setText("resume")
                self.call_rtab_pause()
            else:
                self.btn_rtabPause.setText("pause")
                self.call_rtab_resume()
        except rospy.ServiceException as e:
            rospy.logerr('pause/resume failed: %s' % e)

    def btn_cancelNavGoalCallback(self):
        """cancel the current goal for move_base"""
        self.pub_cancelGoal.publish(actionlib_msgs.msg.GoalID())
   
    def cb_rtabModeCallback(self, mode):
        """set rtabmap mode to mapping / localization"""
        try:
            if mode:
                self.call_rtab_mode_local()
            else:
                self.call_rtab_mode_mapping()
        except rospy.ServiceException as e:
            rospy.logerr('set rtabmap mode failed: %s' % e)

    def ckb_stayHereCallback(self, stay):
        """force the robot to stay where it is"""
        if stay:
            self.timer_stayHere.start(100)
        else:
            self.timer_stayHere.stop()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MyMainWindow()
    myWin.show()
    sys.exit(app.exec_())
