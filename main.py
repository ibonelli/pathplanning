"""

Mobile robot motion planning for different agents

"""

import math
import numpy as np
	# https://docs.scipy.org/doc/numpy/user/basics.creation.html
import matplotlib.pyplot as plt
	# https://matplotlib.org/users/pyplot_tutorial.html
import subprocess

# Modules
from agent_apf import apfAgent

# Globals
show_animation = True
file_number = 1
goal_agent = "APF"

def main():
    global file_number
    print(__file__ + " start!!")

    # initial state [x, y, yaw(rad), v(m/s), omega(rad/s)]
    if (goal_agent == "DWA"):
        x = np.array([5, 5, math.pi / 8.0, 0.0, 0.0])
    if (goal_agent == "tangentBug"):
        x = np.array([5, 5])

    # goal position [x, y]
    goal = np.array([50, 50])
    # Saving trajectory
    traj = np.array(x)
    u = np.array([0.0, 0.0])

    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt("world05.csv")

    r1 = rAgent()
    r2 = rAgent()
    if (goal_agent == "DWA"):
        dwa = dwaAgent()
    if (goal_agent == "tangentBug"):
        atan1 = atan1Agent()

    for i in range(100):
        # Random1 - Calculating trajectory
        if (r1_ticks == 0):
            r1_ang, r1_vel, r1_ticks = r1.random_control()
        else:
            r1_ticks-=1
        # Random1 - Calculating movement
        if (goal_agent == "tangentBug"):
            ob2add = np.array([[x[0], x[1], atan1.robot_radius], [r2_x[0], r2_x[1], r2.robot_radius]])
        if (goal_agent == "DWA"):
            ob2add = np.array([[x[0], x[1], dwa.robot_radius], [r2_x[0], r2_x[1], r2.robot_radius]])
        ob4r1 = np.vstack((ob,ob2add))
        r1_x, r1_col = r1.motion(r1_x, ob4r1, r1_ang, r1_vel)
        if (r1_col):
            r1_ticks = 0
        r1_traj = np.vstack((r1_traj, r1_x))  # store state history

        # Random2 - Calculating trajectory
        if (r2_ticks == 0):
            r2_ang, r2_vel, r2_ticks = r2.random_control()
        else:
            r2_ticks-=1
        # Random2 - Calculating movement
        if (goal_agent == "tangentBug"):
            ob2add = np.array([[x[0], x[1], atan1.robot_radius], [r1_x[0], r1_x[1], r1.robot_radius]])
        if (goal_agent == "DWA"):
            ob2add = np.array([[x[0], x[1], dwa.robot_radius], [r1_x[0], r1_x[1], r1.robot_radius]])
        ob4r2 = np.vstack((ob,ob2add))
        r2_x, r2_col = r2.motion(r2_x, ob4r2, r2_ang, r2_vel)
        if (r2_col):
            r2_ticks = 0
        r2_traj = np.vstack((r2_traj, r2_x))  # store state history

        # Goal agent
        if (goal_agent == "DWA"):
            ob2add = np.array([[r1_x[0], r1_x[1], r1.robot_radius], [r2_x[0], r2_x[1], r2.robot_radius]])
            ob4dwa = np.vstack((ob,ob2add))
            u, ltraj = dwa.dwa_control(x, u, goal, ob4dwa)
            x = dwa.motion(x, u)
            print "Pos " + str(i) + " - x: " + str(x)
        if (goal_agent == "tangentBug"):
            ob2add = np.array([[r1_x[0], r1_x[1], r1.robot_radius], [r2_x[0], r2_x[1], r2.robot_radius]])
            ob4tb = np.vstack((ob,ob2add))
            limit = atan1.lidar(x, ob4tb)
            #graph_lidar(x, goal, limit, ob, i)
            ang, vel, ticks = atan1.tangentbug_control(x, ob4tb, goal, limit)
            x,col = atan1.motion(x, ob, ang, vel)
            print "======================================================================="
            print "Step " + str(i) + " | pos: " + str(x) + " | ang: " + str(ang) + " | col: " + str(col)

        traj = np.vstack((traj, x))  # store state history

        if show_animation:
            plt.cla()
            # Random1
            plt.plot(r1_x[0], r1_x[1], "xr")
            # Random1
            plt.plot(r2_x[0], r2_x[1], "xb")
            # DWA & TangentBug
            if (goal_agent == "DWA"):
                plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
                dwa.plot_arrow(x[0], x[1], x[2])
            plt.plot(x[0], x[1], "xg")
            plt.plot(goal[0], goal[1], "ob")
            # ob[:, 0] -> The full first row of the array (all X numbers)
            # ob[:, 1] -> The full second row of the array (all Y numbers)
            #plt.plot(ob[:, 0], ob[:, 1], "ok")
            for obx,oby,obs in np.nditer([ob[:, 0], ob[:, 1], ob[:, 2]]):
                patch=plt.Circle((obx, oby), obs, color='black', fill=True)
                tmp=plt.gca()
                tmp.add_patch(patch)
            plt.axis("equal")
            plt.grid(True)
            # We save to a file
            imagefile = format(file_number, '05d')
            file_number += 1
            plt.savefig("/home/ignacio/Downloads/PyPlot/anim_" + imagefile + ".png")
            # And show as we used to
            plt.pause(0.0001)

        # check goal
        if (goal_agent == "DWA"):
            check_radius = dwa.robot_radius
        if (goal_agent == "tangentBug"):
            check_radius = atan1.robot_radius
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= check_radius:
            print("Goal!!")
            break

    path = "/home/ignacio/Downloads/PyPlot/"
    # Didn't work
    #cmd = "convert -delay 0.5 " + path + "anim_*.png -loop 0 -monitor " + path + "movie.gif"
    # Works, but not within Python
    cmd = 'ffmpeg -framerate 25 -pattern_type glob -i "anim_*.png" output.mkv'
    #status = subprocess.call(cmd, shell=True)
    print "First run:"
    print "    " + cmd
    print "And then run:"
    print "    " + "rm " + path + "anim_*.png"

    print("Done")
    if show_animation:
        # Random1
        plt.plot(r1_traj[:, 0], r1_traj[:, 1], "-r")
        # Random2
        plt.plot(r2_traj[:, 0], r2_traj[:, 1], "-b")
        # DWA & TangentBug
        plt.plot(traj[:, 0], traj[:, 1], "-g")
        # We save to a file
        plt.savefig("/home/ignacio/Downloads/PyPlot/movie_end.png")
        # And show as we used to
        plt.show()

if __name__ == '__main__':
    main()
