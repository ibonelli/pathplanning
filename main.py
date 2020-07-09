import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import logging

# Modules
import config
from navApfNavigation import ApfNavigation
from navLidar import Lidar
from navLidarPoint import LidarPoint
from navTrapNavigation import TrapNavigation
from navMap import Map
from navGraf import ShowNavigation
from navLidarLimit import LidarLimit
from navDeliverative import DeliverativeNavigation
from navFollowPath import FollowPath
from navBrushfire import BrushfireNavigation

show_animation = config.general['animation']
graf_delay = config.general['grafDelay']
motion_model_limit = config.general['motion_model_limit']
logging.basicConfig(filename=config.general['logFile'],level=config.general['logLevel'])
lidar_steps = config.general['lidar_steps']

def main():
	# Cargando el archivo de descripcion del mundo a recorrer
	if(len(sys.argv) < 2):
		print('Ingresar el mundo a recorrer. Por ejemplo world1.csv')
		exit(-1)
	else:
		fname = sys.argv[1]

	print("potential_field_planning start")

	grid_size = 1  # potential grid size [m]
	robot_radius = 5.0  # robot radius [m]
	vision_limit = 15

	ob = np.loadtxt(fname)
	flx = ob[:, 0]  # file x position list [m]
	fly = ob[:, 1]  # file y position list [m]
	sx = flx[0]	 # start x position [m]
	sy = fly[0]	 # start y positon [m]
	gx = flx[1]	 # goal x position [m]
	gy = fly[1]	 # goal y position [m]
	ox = flx[2:]   # obstacle x position list [m]
	oy = fly[2:]   # obstacle y position list [m]

	if show_animation:
		MyGraf = ShowNavigation()

	# Map generation
	myMap = Map()
	myMap.set_params(grid_size, gx, gy, ox, oy)
	myMap.create()
	myLidar = Lidar(grid_size, vision_limit, lidar_steps)
	limits = myLidar.fetch_all(sx, sy, ox, oy, "object")
	myLimits = LidarLimit()

	# Navigation Initializing
	myNavigation = ApfNavigation(grid_size, robot_radius)
	trap = TrapNavigation(grid_size, robot_radius, vision_limit)
	myDeliverative = DeliverativeNavigation(robot_radius, vision_limit, grid_size, gx, gy, myMap)
	myNavFollow = FollowPath(grid_size, gx, gy)
	myDeliverative.set_map(myLimits.limit2map(myMap.get_map(), myLidar.get_blocked_path(limits)))

	# Wavefront navigation initializing
	myNavWave = BrushfireNavigation()
	myNavWave.set_params(grid_size, gx, gy, ox, oy, vision_limit)
	myNavWave.set_map(myMap.get_map())

	# Start with a clean motion model
	motion_model_count = 0
	motion_model = trap.reset_motion_model()
	myNavigation.set_motion_model(motion_model)

	# Navigation 1st step
	stuck = False
	aborted = False
	d = float(np.hypot(sx - gx, sy - gy))
	rd, rx, ry = [d], [sx], [sy]
	nav = "apf"
	d, xp, yp, curdirx, curdiry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, True)
	myDeliverative.set_step((xp, yp), (curdirx, curdiry), nav)
	rd.append(d)
	rx.append(xp)
	ry.append(yp)
	if show_animation:
		MyGraf.draw_heatmap(myNavigation.get_pmap())
		MyGraf.show(sx, sy, gx, gy)
		MyGraf.step(xp, yp, graf_delay)

	# Map update and trap detection
	limits = myLidar.fetch_all(sx, sy, ox, oy, "object")
	myDeliverative.set_map(myLimits.limit2map(myDeliverative.get_map(), myLidar.get_blocked_path(limits)))
	myNavWave.update_map(myNavWave.get_map(), xp, yp, limits)
	path_blocked, path_blocked_dir, wall_detected = trap.detect(myDeliverative.get_map_obj(), sx, sy, curdirx, curdiry, gx, gy)

	# Main navigation loop
	while d >= grid_size and not aborted:
		if nav == "apf":
			d, xp, yp, curdirx, curdiry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, False)
			stuck = myNavigation.decide_status(rd)
		elif nav == "follow":
			d, xp, yp, curdirx, curdiry, wlimit = myNavFollow.follow(xp, yp, dirx, diry)
		limits = myLidar.fetch_all(xp, yp, ox, oy, "object")
		path_blocked, path_blocked_dir, wall_detected = trap.detect(myDeliverative.get_map_obj(), xp, yp, curdirx, curdiry, gx, gy)
		myDeliverative.set_status(stuck, path_blocked, path_blocked_dir)
		myDeliverative.set_step((xp, yp), (curdirx, curdiry), nav)
		myDeliverative.set_map(myLimits.limit2map(myDeliverative.get_map(), myLidar.get_blocked_path(limits)))
		myNavWave.update_map(myNavWave.get_map(), xp, yp, limits)

		rd.append(d)
		rx.append(xp)
		ry.append(yp)
		if show_animation:
			MyGraf.step(xp, yp, graf_delay)

		# We now check status
		if (stuck or path_blocked) and not aborted:
			if stuck:
				logging.debug("Stuck (start) ----------------")
				nav = "follow"
				stuck = False
				dirx, diry, limitx, limity = myDeliverative.choose_dir(xp, yp)
				myNavFollow.set_limit(limitx, limity)
				wlimit = False
				logging.debug("Switching to follow model | Direction: " + str((dirx, diry)) + " | limit: " + str((limitx, limity)))
				logging.debug("Stuck (end) ----------------")
			elif path_blocked:
				logging.debug("Path blocked (start) ----------------")
				pvec = myNavigation.get_pvec()
				logging.debug("\tApf vect:")
				for p in pvec:
					apf_pot, apf_dirx, apf_diry, apf_blocked = p.get()
					apf_blocked = myLidar.lidar_limits_direction(xp, yp, (apf_dirx, apf_diry), ox, oy)
					logging.debug("\t\t" + str((apf_pot, apf_dirx, apf_diry, apf_blocked)))
				logging.debug("Path blocked (continue) ----------------")
				motion_model = trap.propose_motion_model(myDeliverative.get_map_obj(), xp, yp)
				if len(motion_model) < 1:
					msg = "We can no longer navigate, because something went wrong with the motion model."
					print(msg)
					logging.debug(msg)
					logging.debug("Current motion model: " + str(motion_model))
					aborted = True
				dirx, diry, limitx, limity = myDeliverative.choose_dir(xp, yp)
				myNavFollow.set_limit(limitx, limity)
				wlimit = False
				logging.debug("Path blocked (end) ----------------")
			#elif nav == "apf" and path_blocked:
			#	nav = "follow_wall"
			#	???
			else:
				logging.debug("Current motion model: " + str(motion_model) + " | stuck: " + str(stuck) + " | path_blocked: " + str(path_blocked))
				msg = "For now we keep navigating..."
				print(msg)
				logging.debug(msg)
				#aborted = True

		if nav == "follow" and wlimit == True:
			logging.debug("wlimit reached, what should we do?")
			# First we check if APF proposed path is free
			myNavigation.set_cur_pos(xp, yp)
			myNavigation.set_motion_model(trap.reset_motion_model())
			d, posX, posY, curdirx, curdiry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, False)
			logging.debug("\tAPF new info... | curdirx: " + str(curdirx) + " | curdiry: " + str(curdiry))
			path_blocked, path_blocked_dir, wall_detected = trap.detect(myDeliverative.get_map_obj(), xp, yp, curdirx, curdiry, gx, gy)
			logging.debug("\ttrap.detect() | path_blocked: " + str(path_blocked) + " | path_blocked_dir: " + str(path_blocked_dir) + " | wall_detected: " + str(wall_detected))
			col,r = myLidar.lidar_limits(posX, posY, (curdirx, curdiry), ox, oy, "object")
			logging.debug("\tlidar_limits() | col: " + str(col) + " | r: " + str(path_blocked_dir))
			if col:
				# If not we get a new path to follow
				dirx, diry, limitx, limity = myDeliverative.checked_path_blocked_dir(xp, yp)
			else:
				# Otherwise we move back to APF
				logging.debug("\tMoving back to apf navigation...")
				nav = "apf"
			wlimit = False

		#if xp == 37 and yp == 45:
		#	msg = "Reached problematic point, xp=35 & yp=45. Stopping navigation."
		#	print(msg)
		#	logging.debug(msg)
		#	aborted = True

		if (xp == 60 and yp == 32):
			checkMyLimits = Lidar(grid_size, vision_limit, lidar_steps)
			limits = checkMyLimits.lidar(xp, yp, ox, oy, "limit")
			myLimits.save_limit(xp, yp, limits, "limit_x60_y32.json")
			aborted = True
		if (xp == 37 and yp == 55):
			checkMyLimits = Lidar(grid_size, vision_limit, lidar_steps)
			limits = checkMyLimits.lidar(xp, yp, ox, oy, "limit")
			myLimits.save_limit(xp, yp, limits, "limit_x37_y55.json")
		if (xp == 37 and yp == 56):
			checkMyLimits = Lidar(grid_size, vision_limit, lidar_steps)
			limits = checkMyLimits.lidar(xp, yp, ox, oy, "limit")
			myLimits.save_limit(xp, yp, limits, "limit_x37_y56.json")

	if aborted:
		checkMyLimits = Lidar(grid_size, vision_limit, 36)
		limits = checkMyLimits.lidar(xp, yp, ox, oy, "limit")
		myLimits.save_limit(xp, yp, limits, "limit.json")
		print("Navigation aborted...")
	else:
		print("Goal!!")

	myMap.set_map(myDeliverative.get_map())
	myMap.save_map("map.json")
	MyGraf.save("path.json")

	myNavWave.show_map("console")

	#if show_animation:
	#	# We save to a file
	#	base=os.path.basename(fname)
	#	plt.savefig(os.path.splitext(base)[0] + "_nav.png")
	#	plt.show()

if __name__ == '__main__':
	print(__file__ + " start!!")
	main()
	print(__file__ + " Done!!")
