#!/usr/bin/env python3.7

import tkinter as Tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
import numpy as np
import time
import datetime
from PIL import Image, ImageTk
from numpy import row_stack
from sympy import E, EX
import rospy
from diagnostic_msgs.msg import KeyValue
from pub_classes import diag_class, move_class, act_class
from multimodal_tactile_custom_msgs.msg import user_prediction, capability, diagnostics, current_action, screw_count
from std_msgs.msg import String
from postgresql.database_funcs import database
import os
import pandas as pd
from tkinter import ttk
from global_data import ACTIONS, TASKS, DEFAULT_TASK, GESTURES, USER_PARAMETERS
import argparse
import rosnode
import threading
import signal

os.chdir(os.path.dirname(__file__))

# Argument parsing
parser = argparse.ArgumentParser(
    description='Node for GUI')

parser.add_argument('--task_type', '-T',
                    help='Task for users to perform, options: assemble_box (default), assemble_complex_box',
                    choices=TASKS,
                    default=DEFAULT_TASK)
args = parser.parse_known_args()[0]

plt.ion()

QUIT = False


def send_shutdown_signal(diag_obj):
    print("shutdown time!")
    shutdown_window = shutting_down_window()

    last_t = time.time()
    while time.time() - last_t < 5:
        shutdown_window.animate_window()
        time.sleep(0.01)

    diag_obj.publish(1, "SHUTDOWN")
    time.sleep(0.5)

    shutdown_window.shutdown()
    rospy.signal_shutdown('Quit Button')


class user_frame:
    def __init__(self, no, id, name, root, cmd_publisher):
        self.no = no
        self.id = id
        self.name = name
        self.root = root
        self.act_pred = np.ones(4)
        self.ges_pred = np.ones(len(GESTURES))
        self.task_name = None
        self.task_data = None
        self.status = "unknown"
        self.destroy = False
        self.shimmer = [None, None, None]
        self.shimmer_info = []

        self.cmd_publisher = cmd_publisher
        self.cmd_publisher.publish(f'User:{self.name}')
        self.cmd_publisher.publish(f'Task:{self.task_name}')

        self.act_fig = Figure()
        self.act_ax = self.act_fig.subplots(1, 1)

        self.ges_fig = Figure()
        self.ges_ax = self.ges_fig.subplots(1, 1)

        self.db = database()

        self.create_user_frame()
        self.create_actions_plots()
        self.update_action_plot()
        self.update_gesture_plot()

    def create_actions_plots(self):
        # Shimmer status indicators
        self.shimmer_frame = Tk.Frame(master=self.root, height=5)#, width=2, bg="red")
        self.shimmer_frame.grid(row=0, column=2, sticky="nsew")
        for i in range(0, 3):
            self.shimmer[i] = Tk.Text(master=self.shimmer_frame, height=5/3)#, width=2, font=('', 10))
            self.shimmer[i].tag_configure("center", justify='center')
            self.shimmer[i].grid(row=i, column=0, sticky="nsew")
            text = "Unknown shimmer\n" \
                   "Unknown\n"
            self.shimmer_info.append([text, 'Unknown'])

        self.update_shimmer_text()

        self.shimmer_frame.grid_columnconfigure(0, weight=1)
        self.shimmer_frame.grid_rowconfigure(0, weight=1)
        self.shimmer_frame.grid_rowconfigure(1, weight=1)
        self.shimmer_frame.grid_rowconfigure(2, weight=1)

        # Graph area for current action predictions
        # A tk.DrawingArea.
        self.act_canvas = FigureCanvasTkAgg(self.act_fig, master=self.root)
        self.act_canvas.draw()
        self.act_canvas.get_tk_widget().grid(row=1, column=2, sticky="nsew")

        # Graph area for current gesture predictions
        # A tk.DrawingArea.
        self.ges_canvas = FigureCanvasTkAgg(self.ges_fig, master=self.root)
        self.ges_canvas.draw()
        self.ges_canvas.get_tk_widget().grid(row=2, column=2, sticky="nsew")

        # Next action button
        self.next_action_button = Tk.Button(
            master=self.root, text="Next Action", command=self.next_action, bg="blue", padx=50, pady=20, width=1, height=1)
        self.next_action_button.grid(
            row=3, column=1, sticky='nsew')

        # Start Task button
        self.start_task_button = Tk.Button(
            master=self.root, text="Start Task", command=self.start_task, bg="green", padx=50, pady=20, width=1, height=1)
        self.start_task_button.grid(
            row=3, column=2, sticky='nsew')

    def create_user_frame(self):
        self.user_frame = Tk.Frame(master=self.root, bg="red")
        self.user_frame.grid(row=0, rowspan=4, column=1, sticky="nsew")

        # User Details
        self.user_deets = Tk.Text(master=self.user_frame, height=5)#, width=2, font=('', 10))
        #self.user_deets.tag_configure("center", justify='center')
        self.user_deets.grid(row=0, column=0, sticky="nsew")
        self.update_user_deets()

        # Tasks List
        self.col_names = ["action_no","action_id","action_name","default_time","user_type","prev_dependent","started","done","t_left"]

        self.tasks = ttk.Treeview(self.user_frame, show=[
                                "headings"], height=1, displaycolumns="#all")
    
        self.tasks.grid(row=1, column=0, sticky='nsew')
        self.tasks["columns"] = self.col_names

        for i in self.col_names:
            self.tasks.column(i, anchor="center", stretch=True, width=20)
            self.tasks.heading(i, text=i, anchor='center')
            
        if self.task_name is not None:
            self.load_task_data()

        # Adjust spacing of objects
        self.user_frame.grid_columnconfigure(0, weight=1)

        self.user_frame.grid_rowconfigure(0, weight=0)
        self.user_frame.grid_rowconfigure(1, weight=1)

    def next_action(self):
        self.cmd_publisher.publish('next_action')

    def start_task(self):
        self.cmd_publisher.publish('start')

    def update_shimmer_text(self):
        for i in range(len(self.shimmer_info)):
            IMU_MSGS = ['ERROR', 'Ready', 'Unknown', 'Shutdown', 'Starting', 'Connecting', 'Initialising']
            colour = "grey"
            if self.shimmer_info[i][1] == IMU_MSGS[1]:
                colour = "green"
            elif (self.shimmer_info[i][1] == IMU_MSGS[4]) or (self.shimmer_info[i][1] == IMU_MSGS[5]) or (self.shimmer_info[i][1] == IMU_MSGS[6]):
                colour = "yellow"
            elif self.shimmer_info[i][1] == IMU_MSGS[0]:
                colour = "red"
            elif self.shimmer_info[i][1] == IMU_MSGS[2]:
                colour = "blue"
            else:
                colour = "grey"
            self.shimmer[i].config(bg=colour)

            self.shimmer[i].delete("1.0", Tk.END)
            self.shimmer[i].insert(Tk.INSERT, self.shimmer_info[i][0])
            self.shimmer[i].tag_add("center", "1.0", "end")

    def update_user_deets(self):
        self.details_text = f"  Name: {self.name} \n" \
                            f"    Id: {self.id} \n" \
                            f"  Task: {self.task_name} \n" \
                            f"Status: {self.status} \n"

    def load_task_data(self):
        self.col_names, actions_list = self.db.query_table(self.task_name, 'all')
        self.task_data = pd.DataFrame(actions_list, columns=self.col_names)
        for row in self.task_data.itertuples():
            self.task_data.at[row.Index, 'default_time'] = round(row.default_time.total_seconds(), 2)
        self.task_data["started"] = 0
        self.task_data["done"] = 0
        self.task_data["t_left"] = 0
        self.col_names.extend(("started", "done", "t_left"))

    def update_action_plot(self):
        self.act_ax.cla()
        pos = np.arange(len(ACTIONS))
        _ = self.act_ax.bar(pos, self.act_pred, align='center', alpha=0.5)

        try:
            self.act_ax.set_xticks(pos)
            self.act_ax.set_xticklabels(ACTIONS)
        except Exception as e:
            print(e)
        self.act_ax.set_ylabel('Confidence')
        self.act_ax.set_ylim([0, 1])
        self.act_ax.set_title('Current Action Prediction')

        plt.pause(0.00001)
        if not QUIT:
            self.act_canvas.draw_idle()

    def update_gesture_plot(self):
        self.ges_ax.cla()
        pos = np.arange(len(GESTURES))
        _ = self.ges_ax.bar(pos, self.ges_pred, align='center', alpha=0.5)

        try:
            self.ges_ax.set_xticks(pos)
            self.ges_ax.set_xticklabels(GESTURES)
        except Exception as e:
            print(e)
        self.ges_ax.set_ylabel('Confidence')
        self.ges_ax.set_ylim([0, 1])
        self.ges_ax.set_title('Current Gesture Prediction')

        plt.pause(0.00001)
        if not QUIT:
            self.ges_canvas.draw_idle()

    def update_user_information(self, name):
        self.name = name
        self.task_name = USER_PARAMETERS[name]
        self.update_user_deets()
        self.load_task_data()


class node_indicator:
    def __init__(self, node_name, master, i):
        self.name = node_name
        self.status = None
        self.indicator = Tk.Label(master=master, bg="grey", text=node_name, width=10,
                                   padx=10, pady=3, borderwidth=2, relief="ridge")
        self.indicator.grid(row=int(i/2), column=i % 2, sticky="nsew")
        self.update_time = None


class new_user_dialogue(object):
    def __init__(self, parent):
        self.fcancel = False
        self.toplevel = Tk.Toplevel(parent)
        self.toplevel.geometry('350x150')
        self.toplevel.resizable(False, False)
        self.var = Tk.StringVar()

        db = database()
        col_names, users = db.query_table('users', 'all')
        users = pd.DataFrame(users, columns=col_names)
        self.default = "Select User"
        options = [self.default]
        for _, row in users.iterrows():
            options.append((row['user_id'], row['user_name']))

        options = tuple(options)
        label = Tk.Label(self.toplevel, text="Choose New User:", height=1, padx=50, pady=2, anchor='center')
        om = ttk.OptionMenu(self.toplevel, self.var, options[0], *options)
        ok_button = Tk.Button(self.toplevel, text="OK", command=self.toplevel.destroy, width=10, height=1, padx=30, pady=10, anchor='center')
        cancel_button = Tk.Button(self.toplevel, text="Cancel", command=self.cancel, width=10, height=1, padx=30, pady=10, anchor='center')

        label.grid(row=0, column=0, columnspan=2)#, sticky="nsew")
        om.grid(row=1, column=0, columnspan=2)#, sticky="nsew")
        ok_button.grid(row=2, column=0)#, sticky="nsew")
        cancel_button.grid(row=2, column=1)#, sticky="nsew")

        # Adjust spacing of objects
        self.toplevel.grid_columnconfigure(0, weight=1)
        self.toplevel.grid_columnconfigure(1, weight=1)
        self.toplevel.grid_rowconfigure(0, weight=1)
        self.toplevel.grid_rowconfigure(1, weight=1)
        self.toplevel.grid_rowconfigure(2, weight=1)

    def cancel(self):
        self.fcancel = True
        self.toplevel.destroy()

    def show(self):
        self.toplevel.deiconify()
        self.toplevel.wait_window()
        value = self.var.get()
        if self.fcancel or (value == self.default):
            return None
        else:
            return value


class shutting_down_window():
    def __init__(self):
        self.Window_Width=800
        self.Window_Height=600
        self.im_size = 100
        self.xinc = self.yinc = 4

        # Create GUI
        self.root = Tk.Tk()
        self.root.wm_title("Shutting Down...")
        self.root.geometry(f'{self.Window_Width}x{self.Window_Height}')

        l = Tk.Label(self.root, text = "Goodbye!")
        l.config(font =("Courier", 20))
        l.pack(fill=Tk.BOTH)

        self.canvas = Tk.Canvas(self.root)
        self.canvas.configure(bg="Blue")
        self.canvas.pack(fill="both", expand=True)
        self.animate_ball()

    def shutdown(self):
        self.root.quit()
        self.root.destroy()

    def animate_ball(self):
        image = Image.open("goodbye.png")
        image = image.resize((self.im_size, self.im_size), Image.ANTIALIAS)
        self.the_image = ImageTk.PhotoImage(image)
        self.image = self.canvas.create_image(400, 300, anchor=Tk.NW, image=self.the_image)
        self.animate_window()

    def animate_window(self):
        self.canvas.move(self.image, self.xinc, self.yinc)
        self.root.update()
        ball_pos = self.canvas.coords(self.image)
        # unpack array to variables
        x, y = ball_pos
        if x < abs(self.xinc) or (x+self.im_size) > self.Window_Width-abs(self.xinc):
            self.xinc = -self.xinc
        if y < abs(self.yinc) or (y+self.im_size) > self.Window_Height-abs(self.yinc):
            self.yinc = -self.yinc


class GUI:
    def __init__(self, cmd_publisher):
        self.db = database()
        # Create GUI
        self.root = Tk.Tk()
        # self.root = Toplevel
        self.root.wm_title("HRC Interaction System")
        self.root.resizable(True, True)
        self.cmd_publisher = cmd_publisher

        self.create_system_frame()

        self.users = []
        self.users.append(user_frame(len(self.users)+1, 1, "unknown", self.root, self.cmd_publisher))


        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_columnconfigure(2, weight=1)
        self.root.grid_rowconfigure(0, weight=0)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_rowconfigure(3, weight=0)

    def create_system_frame(self):
        self.sys_frame = Tk.Frame(master=self.root, bg="dodger blue")
        self.sys_frame.grid(row=0, rowspan=4, column=0, sticky="nsew")

        # Uni logo
        load = Image.open("logo.jpg")
        imsize = 100
        resized = load.resize((imsize, imsize), Image.ANTIALIAS)
        render = ImageTk.PhotoImage(image=resized)
        self.img = Tk.Label(master=self.sys_frame, image=render)
        self.img.image = render
        self.img.grid(row=0, column=0)

        # Nodes Stats Indicators
        self.node_stats = Tk.Frame(master=self.sys_frame)
        self.node_stats.grid(row=1, column=0, sticky="nsew")
        self.nodes_list = [['Database_node', None],
                           ['users_node', None],
                           ['skeleton_viewer', None],
                           ['robot_control_node', None],
                           ['multimodal_tactile_robot_control', None],
                           ['rq_gripper_2F140', None]]
        i = 0
        for node in self.nodes_list:
            node[1] = node_indicator(node[0], self.node_stats, i)
            i += 1

        # Adjust spacing of objects
        self.node_stats.grid_columnconfigure((0, 1), weight=1)
        self.node_stats.grid_rowconfigure((0, 1, 2), weight=1)

        # Robot Status Text
        self.robot_stat_text = "Robot status: unknown"
        self.robot_stats = Tk.Text(master=self.sys_frame, height=2)
        self.robot_stats.grid(row=2, column=0, sticky="nsew")
        self.robot_stats.insert(Tk.INSERT, self.robot_stat_text)

        # Robot Move Command Text
        self.robot_move_text = f"Robot Move Cmd: unknown"
        self.robot_move = Tk.Text(master=self.sys_frame, height=2)
        self.robot_move.grid(row=3, column=0, sticky="nsew")
        self.robot_move.insert(Tk.INSERT, self.robot_move_text)

        # Tasks List
        self.load_robot_actions_data()
        self.tasks = ttk.Treeview(self.sys_frame, show=[
                                  "headings"], displaycolumns="#all")
        self.tasks.grid(row=4, column=0, sticky="nsew")
        self.tasks["columns"] = self.col_names

        for i in self.col_names:
            if i == "last_completed_action_no":
                txt = "prev action no"
            elif i == "user_id":
                txt = "id"
            elif i == "user_name":
                txt = "name"
            else:
                txt = i
            self.tasks.column(i, anchor="center", stretch=True, width=8*len(txt))
            self.tasks.heading(i, text=txt, anchor='center')

        for index, row in self.robot_tasks_data.iterrows():
            self.tasks.insert("", index=index, values=list(
                row), tags=(row['user_id'],))

        # User Feedback Text
        self.usr_feedback_text = f"Please wait, system starting"
        self.usr_feedback = Tk.Text(master=self.sys_frame, height=2, font=("Courier", 14), wrap='word', width=20)
        self.usr_feedback.tag_configure("feedback_tag_center", justify='center')
        self.usr_feedback.grid(row=5, column=0, sticky="nsew")
        self.usr_feedback.insert(Tk.INSERT, self.usr_feedback_text)

        # Quit button
        self.quit_button = Tk.Button(master=self.sys_frame, text="Quit", command=self._quit, bg="red", padx=50, pady=20)
        self.quit_button.grid(row=6, column=0, sticky="nsew")

        # Adjust spacing of objects
        self.sys_frame.grid_columnconfigure(0, weight=1)

        self.sys_frame.grid_rowconfigure(0, weight=0)
        self.sys_frame.grid_rowconfigure(1, weight=0)
        self.sys_frame.grid_rowconfigure(2, weight=0)
        self.sys_frame.grid_rowconfigure(3, weight=0)
        self.sys_frame.grid_rowconfigure(4, weight=1)
        self.sys_frame.grid_rowconfigure(5, weight=1)
        self.sys_frame.grid_rowconfigure(6, weight=0)

    def _quit(self, sig=None, frame=None):
        self.cmd_publisher.publish('stop')
        global QUIT
        QUIT = True
        self.root.quit()     # stops mainloop
        self.root.destroy()  # this is necessary on Windows to prevent

    def load_robot_actions_data(self):
        self.col_names, actions_list = self.db.query_table('robot_action_timings', 'all')
        self.robot_tasks_data = pd.DataFrame(actions_list, columns=self.col_names)
        for row in self.robot_tasks_data.itertuples():
            self.robot_tasks_data.at[row.Index, 'robot_start_t'] = max(0.0, round(row.robot_start_t.total_seconds(), 2))

    def update_gui(self, diag_obj):
        try:
            # Update node status indicators
            i = 0
            for node in self.nodes_list:
                # Check node has had initial reading
                if node[1].update_time is not None:
                    # Time out on node status
                    if (time.time() - node[1].update_time) > 10:
                        node[1].status = 3
                    colour = "grey"
                    if node[1].status == 0:
                        colour = "green"
                    elif node[1].status == 1:
                        colour = "yellow"
                    elif node[1].status == 2:
                        colour = "red"
                    elif node[1].status == 3:
                        colour = "blue"
                    else:
                        colour = "grey"
                    node[1].indicator.config(bg=colour)
                i += 1

            # If postgresql node running
            if [node[1].status for node in self.nodes_list if node[1].name == 'Database_node'][0] == 0:
                # Update robot actions
                self.load_robot_actions_data()
                self.tasks.delete(*self.tasks.get_children())
                for index, row in self.robot_tasks_data.iterrows():
                    self.tasks.insert("", index=index, values=list(row), tags=(row['user_id'],))

                # Update future prediction user data tables
                col_names, predictions_list = self.db.query_table('future_action_predictions', 'all')
                predictions_data = pd.DataFrame(predictions_list, columns=col_names)
                for row in predictions_data.itertuples():
                    try:
                        user_id = row.user_id
                        user_i = [idx for idx, user in enumerate(self.users) if user.id == user_id]
                        if len(user_i)!=0:
                            user_i = user_i[0]
                            self.users[user_i].task_data.loc[self.users[user_i].task_data['action_no'] == row.action_no, 'started'] = round(row.started, 2)
                            self.users[user_i].task_data.loc[self.users[user_i].task_data['action_no'] == row.action_no, 'done'] = round(row.done, 2)
                            self.users[user_i].task_data.loc[self.users[user_i].task_data['action_no'] == row.action_no, 't_left'] = round(row.time_left, 2)
                    except Exception as e:
                        print(e)
                        print(user_i)

                for u, user in enumerate(self.users):
                    user.tasks.delete(*user.tasks.get_children())
                    if user.task_data is not None:
                        for index, row in user.task_data.iterrows():
                            user.tasks.insert("", index=index, values=list(row), tags=(row['action_no'],))
                        try:
                            act_no = self.robot_tasks_data[self.robot_tasks_data['user_id'] == user.id]['next_r_action_no'].values[0]
                            user.tasks.tag_configure(act_no, background='yellow')
                            act_no = self.robot_tasks_data[self.robot_tasks_data['user_id'] == user.id]['last_completed_action_no'].values[0]
                            user.tasks.tag_configure(act_no, background='green')
                        except Exception as e:
                            pass
                            # print(e)
                            # print(self.robot_tasks_data)

            # Update user sinfo and status
            for user in self.users:
                user.update_shimmer_text()
                user.user_deets.delete("1.0", Tk.END)
                user.user_deets.insert(Tk.INSERT, user.details_text)
                # try:
                #     user.act_canvas.draw_idle()
                #     user.ges_canvas.draw_idle()
                # except Exception as e:
                #     print(e)
                #     pass
                #user.user_frame.update_idletasks()
                #user.user_frame.update()

            # Update robot status text
            self.robot_stats.delete("1.0", Tk.END)
            self.robot_stats.insert(Tk.INSERT, self.robot_stat_text)

            # Update robot move cmd text
            self.robot_move.delete("1.0", Tk.END)
            self.robot_move.insert(Tk.INSERT, self.robot_move_text)

            # Update user feedback text
            self.usr_feedback.delete("1.0", Tk.END)
            self.usr_feedback.insert(Tk.INSERT, self.usr_feedback_text)
            self.usr_feedback.tag_add("feedback_tag_center", "1.0", "end")

            # Configure layout and update plots
            # self.root.grid_columnconfigure(0, weight=1)
            # self.root.grid_columnconfigure(1, weight=1)
            # self.root.grid_columnconfigure(2, weight=1)
            # self.root.grid_rowconfigure(0, weight=1)
            # self.root.grid_rowconfigure(1, weight=1)
            # self.root.grid_rowconfigure(2, weight=0)

            # Update gui
            # self.root.update_idletasks()
            # self.root.update()
            diag_obj.publish(0, "Running")
        except Exception as e:
            print(e)
            diag_obj.publish(2, f"Error: {e}")

        # update every 0.5 s
        if not QUIT:
            self.root.after(500, lambda: self.update_gui(diag_obj))

    def update_actions(self, data):
        for user in self.users:
            if data.UserId == user.id:
                # if user.name != data.UserName:
                #     print(f"ERROR: users list name does not match current_action msg name {data.UserName}")
                #     user.name = data.UserName
                # else:
                if data.Header.frame_id[-8:] == '_actions':
                    user.act_pred = data.ActionProbs
                    user.update_action_plot()
                elif data.Header.frame_id[-9:] == '_gestures':
                    user.ges_pred = data.ActionProbs
                    user.update_gesture_plot()

    def update_sys_stat(self, data):
        try:
            i = [idx for idx, sublist in enumerate(self.nodes_list) if data.Header.frame_id in sublist[0]][0]
            self.nodes_list[i][1].status = data.DiagnosticStatus.level
            self.nodes_list[i][1].update_time = time.time()
        except IndexError:
            pass

        for user in self.users:
            if data.Header.frame_id == f'shimmerBase {user.name} {user.id} node':
                i = 0
                for keyval in data.DiagnosticStatus.values[:-1]:
                    text = f"{keyval.key}\n" \
                            f"{keyval.value}\n"
                    user.shimmer_info[i] = [text, keyval.value]
                    i += 1

    def update_robot_stat(self, data):
        self.robot_stat_text = f"Robot status: {data.data}"

    def update_robot_move(self, data):
        self.robot_move_text = f"Robot Move Cmd: {data.data}"
    
    def update_usr_feedback(self, data):
        if data.data != self.usr_feedback_text:
            self.usr_feedback_text = data.data
            if data.data[0:5] == "Hello":
                self.users[0].update_user_information(data.data[6:-1])



def run_gui():
    # Run ROS node
    frame_id = 'gui_node'
    rospy.init_node(frame_id, anonymous=True, disable_signals=False)

    diag_obj = diag_class(frame_id=frame_id, user_id=0, user_name="N/A", queue=1)
    cmd_publisher= rospy.Publisher('ProcessCommands', String, queue_size=10)
    diag_obj.publish(1, "Starting")

    gui = GUI(cmd_publisher)
    signal.signal(signal.SIGINT, gui._quit)
    rospy.Subscriber('CurrentAction', current_action, gui.update_actions)
    rospy.Subscriber('SystemStatus', diagnostics, gui.update_sys_stat)
    rospy.Subscriber('RobotStatus', String, gui.update_robot_stat)
    rospy.Subscriber('RobotMove', String, gui.update_robot_move)
    rospy.Subscriber('UserFeedback', String, gui.update_usr_feedback)

    gui.update_gui(diag_obj)
    Tk.mainloop()

    if QUIT:
        send_shutdown_signal(diag_obj)

if __name__ == '__main__':
    # Run GUI
    try:
        run_gui()
    except rospy.ROSInterruptException:
        pass
