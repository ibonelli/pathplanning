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
from navMap import Map
from navGraf import ShowNavigation
from navLidarLimit import LidarLimit
from navDeliverative import DeliverativeNavigation
from navFollowPath import FollowPath
from navBrushfire import BrushfireNavigation
from navData import NavigationData

create_animation = config.general['animation']
graf_delay = config.general['grafDelay']
lidar_steps = config.general['lidar_steps']
grid_size = config.general['grid_size']
robot_radius = config.general['robot_radius']
brushfire_map_debug = config.general['brushfire_map_debug']
saveResults = config.general['saveResults']
saveReport = config.general['saveReport']
robot_motion_model = config.general['robot_motion_model']

def main():
	# Cargando el archivo de descripcion del mundo a recorrer
	if(len(sys.argv) < 2):
		print('Ingresar el mundo a recorrer. Por ejemplo world1.csv')
		exit(-1)
	else:
		fname = sys.argv[1]
		if(len(sys.argv) == 3):
			vision_limit = int(sys.argv[2])
			print('Usando vision_limit por argumento: ' + str(vision_limit))
		else:
			vision_limit = config.general['vision_limit']
			print('Usando vision_limit por configuración: ' + str(vision_limit))

	print("potential_field_planning start")

	ob = np.loadtxt(fname)
	flx = ob[:, 0]  # file x position list [m]
	fly = ob[:, 1]  # file y position list [m]
	sx = flx[0]	 # start x position [m]
	sy = fly[0]	 # start y positon [m]
	globalgx = gx = flx[1]	 # goal x position [m]
	globalgy = gy = fly[1]	 # goal y position [m]
	ox = flx[2:]   # obstacle x position list [m]
	oy = fly[2:]   # obstacle y position list [m]

	secondary_goal = False
	org_gx, org_gy = gx, gy

	if create_animation:
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
	myNavWave = BrushfireNavigation()
	myNavWave.set_params(grid_size, gx, gy, ox, oy, vision_limit)
	myNavWave.set_map(myMap.get_map())
	pot = myNavWave.update_map(myNavWave.get_map(), sx, sy, limits)
	myDeliverative = DeliverativeNavigation(robot_radius, vision_limit, grid_size, gx, gy, myMap)
	myDeliverative.set_map(LidarLimit.limit2map(myMap.get_map(), myLidar.get_blocked_path(limits)))
	myNavFollow = FollowPath(grid_size, gx, gy)
	if brushfire_map_debug:
		myNavWave.show_map(sx, sy, "debug")

	# Start with a clean motion model
	myNavigation.set_motion_model(robot_motion_model)

	# Navigation 1st step
	stuck = False
	aborted = False
	d = float(np.hypot(sx - gx, sy - gy))
	rd, rx, ry = [d], [sx], [sy]
	nav = "apf"
	d, xp, yp, curdirx, curdiry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, True)

	# Map update and trap detection
	limits = myLidar.fetch_all(xp, yp, ox, oy, "object")
	myDeliverative.set_map(LidarLimit.limit2map(myDeliverative.get_map(), myLidar.get_blocked_path(limits)))
	pot = myNavWave.update_map(myNavWave.get_map(), xp, yp, limits)
	nav_type = "reactive"

	# We save the step
	myDeliverative.set_step((xp, yp), (curdirx, curdiry), nav, nav_type, pot, myNavigation.get_pvec(), limits, myNavWave)
	if brushfire_map_debug:
		myNavWave.show_map(xp, yp, "debug", curdirx, curdiry)
	rd.append(d)
	rx.append(xp)
	ry.append(yp)
	if create_animation:
		MyGraf.draw_heatmap(myNavigation.get_pmap())
		MyGraf.show(sx, sy, gx, gy)
		MyGraf.step(xp, yp, graf_delay)

	apf_reset = False
	steps_count = 0

	# Main navigation loop
	while (d >= grid_size or secondary_goal) and not aborted:
		logging.debug("====================================== NEXT STEP ======================================")
		if nav == "apf":
			if apf_reset:
				d, xp, yp, curdirx, curdiry = myNavigation.potential_field_planning(xp, yp, gx, gy, ox, oy, True)
				apf_reset = False
			else:
				d, xp, yp, curdirx, curdiry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, False)
			logging.debug("<MAIN> -- Navigation APF -- d: " + str(d) + " | xp,yp: " + str((xp,yp)) + " | dir: " + str((curdirx, curdiry)) + " | goal: " + str((gx, gy)) + " | secondary goal: " + str(secondary_goal) + " | steps count: " + str(steps_count))
			stuck = myNavigation.decide_status(rx, ry, xp, yp)
		elif nav == "follow":
			d, xp, yp, curdirx, curdiry, wlimit = myNavFollow.follow(xp, yp, dirx, diry)
			logging.debug("<MAIN> -- Navigation follow -- d: " + str(d) + " | xp,yp: " + str((xp,yp)) + " | dir: " + str((curdirx, curdiry)) + " | goal: " + str((gx, gy)) + " | secondary goal: " + str(secondary_goal) + " | steps count: " + str(steps_count))
			stuck = myNavFollow.decide_status(xp, yp, ox, oy)
		elif nav == "follow_steps":
			d, xp, yp, curdirx, curdiry = myNavFollow.follow_steps(xp, yp, myDeliverative.get_astar_path())
			logging.debug("<MAIN> -- Navigation follow_steps -- d: " + str(d) + " | xp,yp: " + str((xp,yp)) + " | dir: " + str((curdirx, curdiry)) + " | goal: " + str((gx, gy)) + " | secondary goal: " + str(secondary_goal) + " | steps count: " + str(steps_count))
			stuck = myNavFollow.decide_status(xp, yp, ox, oy)

		# If not stuck but we reached a limit
		if not stuck:
			stuck = myDeliverative.check_limits(xp, yp)

		if stuck:
			xp = rx[-1]
			yp = ry[-1]
			nav_changed, dirx, diry, limitx, limity, newnav, nav_type = myDeliverative.propose_new_aproach((xp, yp), (curdirx, curdiry), myNavWave)
		else:
			limits = myLidar.fetch_all(xp, yp, ox, oy, "object")
			pot = myNavWave.update_map(myNavWave.get_map(), xp, yp, limits)
			myDeliverative.set_map(LidarLimit.limit2map(myDeliverative.get_map(), myLidar.get_blocked_path(limits)))
			myDeliverative.set_step((xp, yp), (curdirx, curdiry), nav, nav_type, pot, myNavigation.get_pvec(), limits, myNavWave)
			if brushfire_map_debug:
				print("Step: " + str((xp,yp)))
				myNavWave.show_map(xp, yp, "debug", curdirx, curdiry)
			# Shall we continue with the same approach?
			nav_changed, dirx, diry, limitx, limity, newnav, nav_type, aborted = myDeliverative.decide_status(myNavWave)

		if nav_changed:
			logging.debug("<MAIN> -- decide_status() has chosen a new direction: " + str((dirx, diry)))
			logging.debug("<MAIN> -- \tNew limit: " + str((limitx, limity)) + " | newnav: " + str(newnav) + " | navtype: " + str(nav_type))
			nav = newnav
			if nav == "follow":
				myNavFollow.set_limit(limitx, limity)
			if nav == "apf":
				logging.debug("<MAIN> -- We require an APF reset.")
				myNavigation.set_cur_pos(xp, yp)
				apf_reset = True
			if limitx != org_gx or limity != org_gy:
				secondary_goal = True
				logging.debug("<MAIN> -- Secondary goal is True.")
			else:
				secondary_goal = False
				logging.debug("<MAIN> -- Secondary goal is False.")
			# We update navigation goal
			gx, gy = limitx, limity

		steps_count += 1

		# If we are stuck we did not accept the step
		if not stuck:
			d = np.hypot(globalgx - xp, globalgy - yp)

			rd.append(d)
			rx.append(xp)
			ry.append(yp)
			if create_animation:
				MyGraf.step(xp, yp, graf_delay)

			#if (xp == 19 and yp == 23):
			#	checkMyLimits = Lidar(grid_size, vision_limit, lidar_steps)
			#	limits = checkMyLimits.lidar(xp, yp, ox, oy, "limit")
			#	myLimits.save_limit(xp, yp, limits, "limit_x19_y23.json")
			#	aborted = True

			#if (steps_count>=68):
			#	aborted = True
			#	print("Navigation aborted...")
			#	newgx, newgy, rx, ry = myDeliverative.get_next_unknown_goal(myNavWave)
			#	logging.debug("\tProposed path by Astar | new_goal: " + str((newgx, newgy)))
			#	logging.debug("\t\trx: " + str(rx))
			#	logging.debug("\t\try: " + str(ry))

	# Navigation has finished
	if aborted:
		goal_reached = False
		steps_to_goal = float('inf')
		checkMyLimits = Lidar(grid_size, vision_limit, 36)
		limits = checkMyLimits.lidar(xp, yp, ox, oy, "limit")
		myLimits.save_limit(xp, yp, limits, "limit.json")
		print("Navigation aborted...")
	else:
		goal_reached = True
		steps_to_goal = len(rd)
		print("Goal!!")

	if saveResults:
		# We save navigation data files
		fbase=fname[0:-4] # We cut the extension
		myMap.set_map(myDeliverative.get_map())
		myMap.save_map(fbase + "_map.json")
		MyGraf.save(fbase)
		myMap.draw(fbase + "_objs.png")

	if saveReport:
		fbase=fname[0:-4] # We cut the extension
		filename = fbase + "_report.txt"
		freport = open(filename, 'w')
		freport.write("World: " + str(fbase) + "\n")
		freport.write("Method: Full Deliverative\n")
		if goal_reached:
			freport.write("Result: Found Goal\n")
		else:
			freport.write("Result: Failed\n")
		freport.write("Steps: " + str(steps_to_goal) + "\n\n")
		freport.close()

	#if brushfire_map_debug:
	#	myNavWave.show_map(xp, yp, "console")

if __name__ == '__main__':
	print(__file__ + " start!!")
	main()
	print(__file__ + " Done!!")
