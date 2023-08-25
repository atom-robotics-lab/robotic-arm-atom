#!/usr/bin/env python3

import rospy
import tkinter as tk
from std_msgs.msg import String
import threading
from queue import Queue
from attach import attach_links

considerLink = "medium_box_1"

def collision_callback(msg):
    string = msg.data
    word_list = string.split("-")
    wordList = []

    for wordIndex in range(len(word_list)):
        word = word_list[wordIndex]
        index = word.find("::")
        needWord = word[:index] if index != -1 else word
        wordList.append(needWord)

    print(wordList)
    
    if considerLink in wordList:
        update_button_state(wordList)

def update_button_state(wordList):
    global button_enabled
    if considerLink in wordList:
        if not button_enabled:
            button_state_queue.put(True)
            button_enabled = True
    else:
        if button_enabled:
            button_state_queue.put(False)
            button_enabled = False

def ros_spin():
    rospy.spin()

def button_click():
    attach_links()  # Call the function from attach.py
    print("Button clicked!")

def enable_button():
    button.config(state=tk.NORMAL)

def disable_button():
    button.config(state=tk.DISABLED)

def process_button_state_queue():
    while True:
        state = button_state_queue.get()
        if state:
            enable_button()
        else:
            disable_button()
        button_state_queue.task_done()

if __name__ == '__main__':
    try:
        rospy.init_node('collision_subscriber', anonymous=True)
        rospy.Subscriber("/gazebo/collision/info", String, collision_callback)

        thread_ros_spin = threading.Thread(target=ros_spin)
        thread_ros_spin.daemon = True
        thread_ros_spin.start()

        root = tk.Tk()
        root.title("Collision Detection")

        button_state_queue = Queue()
        button_enabled = False

        button = tk.Button(root, text="Clickable Button", state=tk.DISABLED, command=button_click)
        button.pack(padx=20, pady=20)

        button_state_thread = threading.Thread(target=process_button_state_queue)
        button_state_thread.daemon = True
        button_state_thread.start()

        root.mainloop()
    except rospy.ROSInterruptException:
        pass
