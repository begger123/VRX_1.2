# lowLevel
Houses the entirety of the low level systems for the 16' WAM-V and RoboBoat Vessels

# to run the simulation clone lowLevel, missionPlanner, usv16_simulation, and usv16_support packages.
# lowLevel, missionPlanner, and usv16_simulation packages should be on branch mastter.
# However, usv16_support needs to be on branch remoteCompile.

#After cloning into *any_ros_catkin_workspace/src, run catkin_make from /src.
#Finally, run:
roslaunch usv16_ctrl usv16_ctrl_sim.launch
#For the love of everything, do not work on the master branch

Notes on git

useful commands:
git status - this shows what items have been updated, and what items are/are not tracked
		   - also shows what branch you are on.  and it better not be master.

if it is on master use
"git checkout -b branchName" where branchName is the name of a new branch that you want to create
"git checkout branchName" where branchName is a name of a branch that already exists

if you are on the right branch, and you like the changes that have been made, stage them with
git add --all

once your files have been added, you are ready to commit with
git commit -m "Some commit message"

if you have an internet connection, push
git push origin branchName