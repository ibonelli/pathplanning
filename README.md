# Path Planning

Thesis Path Planning Simulations

Thanks to Atsushi Sakai (@Atsushi_twi) for publishing [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)

## Running

- run*.sh : Different scripts to run all worlds in batch
- world*.csv : Different worlds which you can run
- main.py : The main program to run (it uses the proposed hybrid algorithm).
- main_astar.py : An alternative resolution using Astar (used for comparisson purposes).
- navApfNavigation.py : Potential Field based path planner
- navFollowPath.py : FollowPath navigation
- navBrushfire.py : Brushfire map calculation
- navDeliverative.py : Deliverative planner & decision making
- navLidar*.py : LIDAR related processing
- navData.py : Class that handles data storing


## Tools

- WorldBuilder.ods : Spreadsheet where you create the world and export as CSV
- show_world.py : Tool to create a usable world from CSV output
- show_limits.py : Show a JSON encoded limit
- show_map.py : Show a JSON encoded map
- show_path.py: Show a JSON encoded path
