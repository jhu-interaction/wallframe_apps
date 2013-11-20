#!/usr/bin/env python
import roslib; roslib.load_manifest('wallframe_app_key_control')
import rospy, sys, subprocess

# Wallframe Core Functionality
import wallframe_core
from wallframe_core import WallframeAppWidget

# PySide Imports
import PySide
from PySide.QtGui import QApplication

# Send key events to a window based on user gestures.

# TODO Some reasonable way to switch on application to use different
# state machines to generate key events

# For the moment make this a QWidget so we can inherit all the wallframe
# setup

class KeyControlAppWidget(WallframeAppWidget):

  def __init__(self,name,app):
    super(KeyControlAppWidget,self).__init__(name,app)
    self.hide()

    # TODO lookup param
    self.subapp = 'supertuxkart'

    # Current state of key presses.
    # key -> True (down), False (up)
    self.keypress = {}

  def user_state_cb(self, msg):
    for user in msg.users:
      if user.focused:
        self.update_subapp(user)

  # TODO: Cache joint indexes. Should be constant along with joint names.

  def joint(self,user,name):
    return user.translations_body_mm[user.frame_names.index(name)]

  def update_subapp(self, user):
    # rospy.logwarn(user)  
    
    # Todo right and left hands seem reversed...
    right_hand = self.joint(user, 'left_hand')
    left_hand =  self.joint(user, 'right_hand')
    right_hip =  self.joint(user, 'right_hip')
    left_hip =   self.joint(user, 'left_hip')
    torso =      self.joint(user, 'torso')
    head =      self.joint(user, 'head')
    
    midpoint = (torso.y + right_hip.y) / 2

    right_hand_up = right_hand.y > midpoint
    left_hand_up = left_hand.y > midpoint

    #rospy.logwarn('Right hand up: ' + str(right_hand_up))
    #rospy.logwarn('Left hand up: ' + str(left_hand_up))

    if user.hands_together:
      self.send_keydown('Down')
      self.send_keyup('Up')    
      self.send_keyup('Left')
      self.send_keyup('Right')     
    elif right_hand_up and left_hand_up:
      self.send_keydown('Up')
      self.send_keyup('Down')

      # Try to allow raising hand above head to turn with both hands up
      
      head_top = head.y + (torso.y - right_hip.y)

      if (right_hand.y > head_top and right_hand.y > left_hand.y):
        self.send_keyup('Left')
        self.send_keydown('Right')
      elif (left_hand.y > head_top and left_hand.y > right_hand.y):
        self.send_keyup('Right')
        self.send_keydown('Left')
      else:
        self.send_keyup('Left')
        self.send_keyup('Right')

    elif right_hand_up:
      self.send_keydown('Right')
      self.send_keyup('Left')
    elif left_hand_up:
      self.send_keydown('Left')
      self.send_keyup('Right')      
    else:
      self.send_keyup('Up')
      self.send_keyup('Down')
      self.send_keyup('Left')
      self.send_keyup('Right')      

  def send_keydown(self, key):
    if key not in self.keypress or not self.keypress[key]:
      self.send_event('keydown', key)
      self.keypress[key] = True

  def send_keyup(self, key):
    if key not in self.keypress or self.keypress[key]:
      self.send_event('keyup', key)
      self.keypress[key] = False

  def send_event(self, event, value):
    rospy.logwarn('Sending ' + event + ' ' + value)

    subprocess.call(["xdotool", "search", "--name", self.subapp, event, value])

class KeyControlApp():
  def __init__(self):
    rospy.init_node('key_control_app',anonymous=True)
    app = QApplication(sys.argv)
    app_widget = KeyControlAppWidget("KeyControlApp",app)

    rospy.logwarn("KeyControlApp: Started")  
    app.exec_()
    rospy.logwarn('KeyControlApp: Finished')

if __name__ == '__main__':
  app = KeyControlApp()
