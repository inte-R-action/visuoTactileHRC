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
from global_data import ACTIONS, TASKS, DEFAULT_TASK, inclAdjParam
import argparse
import rosnode
import threading

# os.chdir(os.path.expanduser(
#     "~/catkin_ws/src/multimodal_human_robot_collaboration/sam_nodes/scripts"))
os.chdir(os.path.dirname(__file__))

# Argument parsing
parser = argparse.ArgumentParser(
    description='Node for GUI')

parser.add_argument('--task_type', '-T',
                    help='Task for users to perform, options: assemble_box (default), assemble_complex_box',
                    choices=TASKS,
                    default=DEFAULT_TASK)
# parser.add_argument('--inclAdjParam',
#                     help='include user/task adjustment parameters for lstm model',
#                     choices=[True, False],
#                     default=False)
args = parser.parse_known_args()[0]

print(f"GUI settings: {args.task_type}")
pos = np.arange(len(ACTIONS))

plt.ion()

QUIT = False


def send_shutdown_signal(diag_obj):
    print("shutdown time!")
    shutdown_window = shutting_down_window()
    nodes = rosnode.get_node_names()

    x = threading.Thread(target=enter_dreaming_phase)
    x.start()

    while nodes or x.is_alive():
        shutdown_window.animate_window()

        if not x.is_alive():
            diag_obj.publish(1, "SHUTDOWN")
            time.sleep(5)
            nodes = False
        
        # nodes = rosnode.get_node_names()
        # try:
        #     nodes.remove('/rosout')
        # except Exception:
        #     pass
        # try:
        #     nodes.remove('/gui_node')
        # except Exception:
        #     pass
        time.sleep(0.01)

    time.sleep(1)
    shutdown_window.shutdown()
    rospy.signal_shutdown('Quit Button')


class user_frame:
    def __init__(self, no, id, name, root, cmd_publisher):
        self.no = no
        self.id = id
        self.name = name
        self.root = root
        self.imu_pred = np.ones(4)
        self.task_name = args.task_type
        self.task_data = None
        self.status = "unknown"
        self.destroy = False
        self.shimmer = [None, None, None]
        self.shimmer_info = []

        self.cmd_publisher = cmd_publisher
        self.cmd_publisher.publish(f'User:{self.name}')
        self.cmd_publisher.publish(f'Task:{self.task_name}')

        self.fig = Figure()
        self.ax = self.fig.subplots(1, 1)

        self.db = database()

        self.create_user_frame()
        self.update_action_plot()

    def create_user_frame(self):
        self.user_frame = Tk.Frame(master=self.root, bg="red")
        self.user_frame.grid(row=0, column=self.no, sticky=Tk.N + Tk.S)

        # User Details
        self.user_deets = Tk.Text(master=self.user_frame, height=5, width=2, font=('', 10))
        #self.user_deets.tag_configure("center", justify='center')
        self.user_deets.grid(row=0, column=0, sticky="nsew")
        self.update_user_deets()

        # Shimmer status indicators
        self.shimmer_frame = Tk.Frame(master=self.user_frame, height=5, width=2, bg="red")
        self.shimmer_frame.grid(row=0, column=1, sticky="nsew")
        for i in range(0, 3):
            self.shimmer[i] = Tk.Text(master=self.shimmer_frame, height=5/3, width=2, font=('', 10))
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

        # LSTM network parameters
        self.lstm_params_txt = Tk.Text(master=self.user_frame, height=5, width=2, font=('', 10))
        self.lstm_params_txt.tag_configure("right", justify='right')
        self.lstm_params_txt.grid(row=0, column=2, sticky="nsew")
        self.update_lstm_params_txt()

        # Graph area for current predictions
        # A tk.DrawingArea.
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.user_frame)
        self.canvas.draw()
        try:
            self.canvas.get_tk_widget().grid(row=1, column=0, columnspan=3,
                                             sticky="nsew")
        except AttributeError:
            pass

        # Tasks List
        self.load_task_data()
        self.tasks = ttk.Treeview(self.user_frame, show=[
                                  "headings"], height=len(self.task_data.index), displaycolumns="#all")
        self.tasks.grid(row=2, column=0, columnspan=3,
                        sticky=Tk.W + Tk.E + Tk.N + Tk.S)
        self.tasks["columns"] = self.col_names

        for i in self.col_names:
            self.tasks.column(i, anchor="center", stretch=True, width=20)
            self.tasks.heading(i, text=i, anchor='center')

        for index, row in self.task_data.iterrows():
            self.tasks.insert("", index=index, values=list(
                row), tags=(row['action_no'],))

        # Remove User button
        self.remove_user_button = Tk.Button(
            master=self.user_frame, text="Remove User", command=self.remove_user, bg="red", padx=50, pady=20, width=1, height=1)
        self.remove_user_button.grid(
            row=3, column=0, sticky='nsew')

        # Next action button
        self.next_action_button = Tk.Button(
            master=self.user_frame, text="Next Action", command=self.next_action, bg="blue", padx=50, pady=20, width=1, height=1)
        self.next_action_button.grid(
            row=3, column=1, sticky='nsew')

        # Start Task button
        self.start_task_button = Tk.Button(
            master=self.user_frame, text="Start Task", command=self.start_task, bg="green", padx=50, pady=20, width=1, height=1)
        self.start_task_button.grid(
            row=3, column=2, sticky='nsew')

        # Adjust spacing of objects
        self.user_frame.grid_columnconfigure(0, weight=1)
        self.user_frame.grid_columnconfigure(1, weight=1)
        self.user_frame.grid_columnconfigure(2, weight=1)

        self.user_frame.grid_rowconfigure(0, weight=0)
        self.user_frame.grid_rowconfigure(1, weight=1)
        self.user_frame.grid_rowconfigure(2, weight=0)
        self.user_frame.grid_rowconfigure(3, weight=0)

    def remove_user(self):
        self.user_frame.quit()     # stops mainloop
        self.user_frame.destroy()  # this is necessary on Windows to prevent
        self.destroy = True

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

    def update_lstm_params_txt(self):
        if inclAdjParam:
            col_names, data = self.db.query_table('users', 'all')
            users_data = pd.DataFrame(data, columns=col_names)
            users_data = users_data.loc[users_data['user_name']==self.name]
            col_names, data = self.db.query_table('tasks', 'all')
            tasks_data = pd.DataFrame(data, columns=col_names)
            tasks_data = tasks_data.loc[tasks_data['task_name']==self.task_name]

            user_params = users_data[ACTIONS].values[0].round(1)  # time adjust for user
            task_params = tasks_data[ACTIONS].values[0].round(1)  # time adjustment for task
        else:
            user_params = ["N/A"]*len(ACTIONS)
            task_params = ["N/A"]*len(ACTIONS)

        text = "LSTM Params       \n" \
               "User|Task \n"
        text = text+''.join([f"{action}:  {user_params[a]} |  {task_params[a]} \n" for a, action in enumerate(ACTIONS)])

        self.lstm_params_txt.delete("1.0", Tk.END)
        self.lstm_params_txt.insert(Tk.INSERT, text)
        self.lstm_params_txt.tag_add("right", "1.0", "end")

    def update_user_deets(self):
        text = f" Name: {self.name} \n" \
               f"       Id: {self.id} \n" \
               f"  Task: {self.task_name} \n" \
               f"Status: {self.status} \n"

        self.user_deets.delete("1.0", Tk.END)
        self.user_deets.insert(Tk.INSERT, text)

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
        self.ax.cla()
        _ = self.ax.bar(pos, self.imu_pred, align='center', alpha=0.5)

        try:
            self.ax.set_xticks(pos)
            self.ax.set_xticklabels(ACTIONS)
        except Exception as e:
            print(e)
        self.ax.set_ylabel('Confidence')
        self.ax.set_ylim([0, 1])
        self.ax.set_title('Current Action Prediction')

        plt.pause(0.00001)


class node_indicator:
    def __init__(self, node_name, master, i):
        self.name = node_name
        self.status = None
        self.indicator = Tk.Label(master=master, bg="grey", text=node_name,
                                  width=1, padx=10, pady=3, borderwidth=2, relief="ridge")
        self.indicator.grid(row=i % 2, column=int(i/2), sticky="nsew")
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

        l = Tk.Label(self.root, text = "Dreaming State")
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
        image = Image.open("dreaming.png")
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
        self.fig = Figure()
        self.axs = np.reshape(self.fig.subplots(1, 1), (-1, 1))

        self.create_system_frame()

        self.users = []

        self.root.grid_columnconfigure(0, weight=1)
        for i in range(len(self.users)):
            self.root.grid_columnconfigure(i+1, weight=1)

        self.root.grid_rowconfigure(0, weight=1)

    def create_system_frame(self):
        self.sys_frame = Tk.Frame(master=self.root, bg="dodger blue")
        self.sys_frame.grid(row=0, column=0, sticky="nsew")

        # Uni logo
        load = Image.open("logo.jpg")
        imsize = 100
        resized = load.resize((imsize, imsize), Image.ANTIALIAS)
        render = ImageTk.PhotoImage(image=resized)
        self.img = Tk.Label(master=self.sys_frame, image=render)
        self.img.image = render
        self.img.grid(row=0, column=0, columnspan=2)

        # Nodes Stats Indicators
        self.node_stats = Tk.Frame(master=self.sys_frame)
        self.node_stats.grid(row=1, column=0, columnspan=2, sticky="nsew")
        self.nodes_list = [['Database_node', None],
                           ['users_node', None],
                           ['skeleton_viewer', None],
                           ['robot_control_node', None],
                           ['hri_static_demo', None],
                           ['rq_gripper_2F140', None]]
        i = 0
        for node in self.nodes_list:
            node[1] = node_indicator(node[0], self.node_stats, i)
            i += 1

        # Adjust spacing of objects
        self.node_stats.grid_columnconfigure((0, 1, 2), weight=1)
        self.node_stats.grid_rowconfigure((0, 1), weight=1)

        # Robot Status Text
        self.robot_stat_text = "Robot status: unknown"
        self.robot_stats = Tk.Text(master=self.sys_frame, height=2)
        self.robot_stats.grid(row=2, column=0, columnspan=2, sticky="nsew")
        self.robot_stats.insert(Tk.INSERT, self.robot_stat_text)

        # Robot Move Command Text
        self.robot_move_text = f"Robot Move Cmd: unknown"
        self.robot_move = Tk.Text(master=self.sys_frame, height=2)
        self.robot_move.grid(row=3, column=0, columnspan=2, sticky="nsew")
        self.robot_move.insert(Tk.INSERT, self.robot_move_text)

        # Tasks List
        self.load_robot_actions_data()
        self.tasks = ttk.Treeview(self.sys_frame, show=[
                                  "headings"], height=18, displaycolumns="#all")
        self.tasks.grid(row=4, column=0, columnspan=2, sticky="nsew")
        self.tasks["columns"] = self.col_names

        for i in self.col_names:
            self.tasks.column(i, anchor="center", stretch=True, width=20)
            self.tasks.heading(i, text=i, anchor='center')

        for index, row in self.robot_tasks_data.iterrows():
            self.tasks.insert("", index=index, values=list(
                row), tags=(row['user_id'],))

        # Timing Predictions Graph
        # A tk.DrawingArea.
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.sys_frame)
        self.canvas.draw()
        try:
            self.canvas.get_tk_widget().grid(row=5, column=0, columnspan=2,
                                            sticky="nsew")
        except AttributeError:
            pass

        # New User button
        self.new_user_button = Tk.Button(master=self.sys_frame, text="New User", command=self._new_user, bg="green", padx=50, pady=20)
        self.new_user_button.grid(row=6, column=0, sticky="nsew")

        # Quit button
        self.quit_button = Tk.Button(master=self.sys_frame, text="Quit", command=self._quit, bg="red", padx=50, pady=20)
        self.quit_button.grid(row=6, column=1, sticky="nsew")

        # Adjust spacing of objects
        self.sys_frame.grid_columnconfigure(0, weight=1)
        self.sys_frame.grid_columnconfigure(1, weight=1)

        self.sys_frame.grid_rowconfigure(0, weight=0)
        self.sys_frame.grid_rowconfigure(1, weight=0)
        self.sys_frame.grid_rowconfigure(2, weight=0)
        self.sys_frame.grid_rowconfigure(3, weight=0)
        self.sys_frame.grid_rowconfigure(4, weight=1)
        self.sys_frame.grid_rowconfigure(5, weight=1)
        self.sys_frame.grid_rowconfigure(6, weight=0)

    def _quit(self):
        self.cmd_publisher.publish('stop')
        global QUIT
        QUIT = True
        self.root.quit()     # stops mainloop
        self.root.destroy()  # this is necessary on Windows to prevent

    def _new_user(self):
        user = new_user_dialogue(self.root).show()
        if user is not None:
            user = eval(user)
            self.users.append(user_frame(len(self.users)+1, user[0], user[1], self.root, self.cmd_publisher))
            self.fig.clf()
            self.axs = np.reshape(self.fig.subplots(len(self.users)+1, 1, sharex='col'), (-1,1))  # subplot for each user plus robot

    def load_robot_actions_data(self):
        self.col_names, actions_list = self.db.query_table('robot_action_timings', 'all')
        self.robot_tasks_data = pd.DataFrame(actions_list, columns=self.col_names)
        for row in self.robot_tasks_data.itertuples():
            self.robot_tasks_data.at[row.Index, 'robot_start_t'] = max(0.0, round(row.robot_start_t.total_seconds(), 2))

    def update_gui(self):
        # destroy any removed users
        for i in range(len(self.users)):
            try:
                if self.users[i].destroy == True:
                    del self.users[i]
                    self.fig.clf()
                    self.axs = np.reshape(self.fig.subplots(len(self.users)+1, 1, sharex='col'), (-1,1))  # subplot for each user plus robot
            except IndexError:
                pass

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

            for u, user in enumerate(self.users):
                user.tasks.delete(*user.tasks.get_children())
                for index, row in user.task_data.iterrows():
                    user.tasks.insert("", index=index, values=list(row), tags=(row['action_no'],))

                try:
                    act_no = self.robot_tasks_data[self.robot_tasks_data['user_id'] == user.id]['next_r_action_no'].values[0]
                    user.tasks.tag_configure(act_no, background='yellow')
                    act_no = self.robot_tasks_data[self.robot_tasks_data['user_id'] == user.id]['last_completed_action_no'].values[0]
                    user.tasks.tag_configure(act_no, background='green')
                except Exception:
                    pass

            # Update future timings plot
            self.update_timings_plot(predictions_data)

        # Update user screw counts
        for user in self.users:
            user.update_shimmer_text()
            try:
                user.canvas.draw_idle()
            except:
                pass
            # user.user_frame.update_idletasks()
            # user.user_frame.update()

        # Update robot status text
        self.robot_stats.delete("1.0", Tk.END)
        self.robot_stats.insert(Tk.INSERT, self.robot_stat_text)

        # Update robot move cmd text
        self.robot_move.delete("1.0", Tk.END)
        self.robot_move.insert(Tk.INSERT, self.robot_move_text)

        # Configure layout and update plots
        self.root.grid_columnconfigure(0, weight=1)
        for i in range(len(self.users)):
            self.root.grid_columnconfigure(i+1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        try:
            self.canvas.draw_idle()
        except:
            pass

        # Update gui
        #self.root.update_idletasks()
        self.root.update()

    def update_actions(self, data):
        for user in self.users:
            if data.UserId == user.id:
                if user.name != data.UserName:
                    print(f"ERROR: users list name {self.users[data.UserId].name} does not match current_action msg name {data.UserName}")
                else:
                    user.imu_pred = data.ActionProbs
                    user.update_action_plot()

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

    def update_timings_plot(self, predictions_data):
        active_users = self.users
        [ax[0].cla() for ax in self.axs]

        for u in range(self.axs.shape[0]-1):
            time_predictions = predictions_data.loc[predictions_data["user_id"]==active_users[u].id]["time_left"]
            for t in time_predictions:
                self.axs[u, 0].axvline(x=t)
            try:
                self.axs[u, 0].get_yaxis().set_ticks([])
            except Exception:
                pass
            self.axs[u, 0].set_ylabel(f"User: {u}")

        # Plot robot solo action times
        try:
            time_predictions = self.robot_tasks_data.loc[self.robot_tasks_data["user_name"]=="robot"]["robot_start_t"].values[0]
            self.axs[-1, 0].axvline(x=time_predictions)
        except (KeyError, IndexError) as e:
            # print("robot solo action time error")
            pass
        try:
            self.axs[-1, 0].get_yaxis().set_ticks([])
        except Exception:
            pass
        self.axs[-1, 0].set_ylabel("Robot Solo")

        self.fig.text(0.5, 0.02, 'Time into future, s', ha='center')
        self.fig.suptitle('Future Timing Predictions')

        plt.pause(0.00001)


def run_gui():
    # Run ROS node
    frame_id = 'gui_node'
    rospy.init_node(frame_id, anonymous=True)

    diag_obj = diag_class(frame_id=frame_id, user_id=0, user_name="N/A", queue=1)
    cmd_publisher= rospy.Publisher('ProcessCommands', String, queue_size=10)
    diag_obj.publish(1, "Starting")

    gui = GUI(cmd_publisher)
    rospy.Subscriber('CurrentAction', current_action, gui.update_actions)
    rospy.Subscriber('SystemStatus', diagnostics, gui.update_sys_stat)
    rospy.Subscriber('RobotStatus', String, gui.update_robot_stat)
    rospy.Subscriber('RobotMove', String, gui.update_robot_move)

    while not rospy.is_shutdown():
        if QUIT:
            send_shutdown_signal(diag_obj)
        else:
            try:
                gui.update_gui()
                diag_obj.publish(0, "Running")
            except Exception as e:
                diag_obj.publish(2, f"Error: {e}")


if __name__ == '__main__':
    # Run GUI
    try:
        run_gui()
    except rospy.ROSInterruptException:
        pass
