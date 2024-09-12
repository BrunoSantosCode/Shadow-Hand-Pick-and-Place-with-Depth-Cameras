#!/usr/bin/env python3

import tkinter as tk
from tkinter import scrolledtext
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

class GUI:
    def __init__(self, master):
        self.master = master
        master.title("Shadow Hand Info")

        self.text_area = scrolledtext.ScrolledText(master, width=50, height=40)
        self.text_area.pack()

def main():
    # Init ROS
    rospy.init_node('get_shadow_joints')

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Shadow Hand Joint Names
    shadow_ff_joints = ('rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4')
    shadow_lf_joints = ('rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5') 
    shadow_mf_joints = ('rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4') 
    shadow_rf_joints = ('rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4') 
    shadow_th_joints = ('rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5') 
    shadow_wr_joints = ('rh_WRJ1', 'rh_WRJ2')

    # GUI setup
    root = tk.Tk()
    gui = GUI(root)

    while not rospy.is_shutdown():
        # Get Tactile Sensors info
        tactile_sensors = hand_commander.get_tactile_state()

        # Get Joints Effort info
        joints_effort = hand_commander.get_joints_effort()

        # Update GUI
        gui.text_area.delete(1.0, tk.END)
        gui.text_area.insert(tk.END, "Tactile sensors:\n")
        gui.text_area.insert(tk.END, str(tactile_sensors) + "\n\n")

        gui.text_area.insert(tk.END, "Joint Effort sensors:\n")
        for joints_group in [shadow_ff_joints, shadow_lf_joints, shadow_mf_joints, shadow_rf_joints, shadow_th_joints, shadow_wr_joints]:
            line = ""
            for joint in joints_group:
                line += "{}: {:.3f}\n".format(joint, joints_effort[joint])
            gui.text_area.insert(tk.END, line + "\n")
        
        gui.text_area.update_idletasks()
        root.update()

if __name__ == "__main__":
    main()
