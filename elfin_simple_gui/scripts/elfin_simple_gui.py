#!/usr/bin/env python
import wx
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class MyFrame(wx.Frame):
    def __init__(self, parent, title):
        super(MyFrame, self).__init__(parent, style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER, title=title, size=(350, 200))

        panel = wx.Panel(self)
        
        # Set Font
        font=panel.GetFont()
        font.SetPixelSize((8, 16))
        panel.SetFont(font)

        # Some variables
        btn_x = 20
        label_x = 250

        fault_y = 20
        servo_y = 80

        # Create the three buttons
        self.fault_button = wx.Button(panel, label=" Clear Fault ", 
                                        name='Clear Fault', pos=(btn_x, fault_y))
        self.servo_on_button = wx.Button(panel, label=" Servo On ", 
                                        name="Servo On", pos=(btn_x, servo_y))
        self.servo_off_button = wx.Button(panel, label=" Servo Off ",
                                        name="Servo Off", pos=(125, servo_y))

        # Create the static texts above each label
        wx.StaticText(panel, label="Fault state:", pos=(label_x, fault_y-10))
        wx.StaticText(panel, label="Servo state:", pos=(label_x, servo_y-10))

        # Create the two displays
        self.fault_state_show = wx.TextCtrl(panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', pos=(label_x, fault_y+10))
        self.servo_on_state_show = wx.TextCtrl(panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', pos=(label_x, servo_y+10))

        # Bind the buttons to event handlers
        self.fault_button.Bind(wx.EVT_BUTTON, self.on_fault_button)
        self.servo_on_button.Bind(wx.EVT_BUTTON, self.on_servo_on_button)
        self.servo_off_button.Bind(wx.EVT_BUTTON, self.on_servo_off_button)

    def on_fault_button(self, event):
        # Call service to clear faults
        self.call_clear_fault.call(self.call_clear_fault_req)

    def on_servo_on_button(self, event):
        # Call service to turn on servos
        self.call_servo_on.call(self.call_servo_on_req)

    def on_servo_off_button(self, event):
        # Call service to turn on servos
        self.call_servo_off.call(self.call_servo_off_req)

    def update_fault_state(self, msg):
        if msg.data:
            self.fault_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.fault_state_show.SetValue('Warning')
        else:
            self.fault_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.fault_state_show.SetValue('No Fault')

    def update_servo_state(self, msg):
        if msg.data:
            self.servo_on_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.servo_on_state_show.SetValue('Enabled')
        else:
            self.servo_on_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.servo_on_state_show.SetValue('Disabled')

    def fault_state_cb(self, msg):
        wx.CallAfter(self.update_fault_state, msg)

    def servo_state_cb(self, msg):
        wx.CallAfter(self.update_servo_state, msg)

    def ros_infrastructure(self):
        # Subscribe to fault and servo states
        topic = "elfin_ros_control/elfin/fault_state"
        rospy.Subscriber(topic, Bool, self.fault_state_cb)
        topic = "elfin_ros_control/elfin/enable_state"
        rospy.Subscriber(topic, Bool, self.servo_state_cb)

        # Create service clients
        service = "elfin_ros_control/elfin/enable_robot"
        self.call_servo_on = rospy.ServiceProxy(service, SetBool)
        self.call_servo_on_req = SetBoolRequest()
        self.call_servo_on_req.data = True
        rospy.loginfo("Waiting for service '%s'...", service)
        rospy.wait_for_service(service)
        rospy.loginfo("Service '%s' is now available!", service)

        service = "elfin_ros_control/elfin/disable_robot"
        self.call_servo_off = rospy.ServiceProxy(service, SetBool)
        self.call_servo_off_req = SetBoolRequest()
        self.call_servo_off_req.data = True
        rospy.loginfo("Waiting for service '%s'...", service)
        rospy.wait_for_service(service)
        rospy.loginfo("Service '%s' is now available!", service)

        service = "elfin_ros_control/elfin/clear_fault"
        self.call_clear_fault = rospy.ServiceProxy(service, SetBool)
        self.call_clear_fault_req = SetBoolRequest()
        self.call_clear_fault_req.data = True
        rospy.loginfo("Waiting for service '%s'...", service)
        rospy.wait_for_service(service)
        rospy.loginfo("Service '%s' is now available!", service)


if __name__ == "__main__":
    rospy.init_node('elfin_simple_gui')
    app = wx.App()
    frame = MyFrame(None, title="Elfin Simple GUI")
    frame.Show()

    frame.ros_infrastructure()

    app.MainLoop()