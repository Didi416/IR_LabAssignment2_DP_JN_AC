## IR_LabAssignment2_DP_JN_AC

# Basic Project Overview

Two robot envrionment that stocks and retrieves items from shelves (most likely pantry shelves). One robot moves around the lower shelves and is responsible for giving and receiving storage boxes from the user and stocking the lower half of the shelves. The Other robot arm will cover the higher shelves, and, since it cannot reach the lower window to hand the items to the user, both robots will need to collaborate to pass the items between them as needed. User GUI wll be used to specify which items need to be retrieved or entered into the system, as well as stating the current inventory and available space.

# Peter Corke's RVC Toolbox changes to be made prior to running

A couple changes were made to the files in the RVC Toolbox for our files to work as intended. These include:
- adding 'nowrist' to 'self.model.plot3d(self.homeQ,'noarrow','workspace',self.workspace,'view',[ax,by],'notiles') line (203) in RobotBaseClass, to be able to view the gripper working properly and to not have too many axes in the simulation. (Aesthetic and presentation purposes, not explicitly necessary for the program to work.)
- LinearUR10 robot file changes:
  - Ensure base rotation is as follows (line 16): 'self.model.base = self.model.base.T * baseTr * trotx(pi/2);'

- Add Robot Model Files to directory: \PeterCorkesRoboticsToolbox_Modified\rvctools\robot\UTS\RobotModels
  - @TheClaww
  - @Robo
