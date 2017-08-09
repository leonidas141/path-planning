# path-planning
Path planning algorithm demonstration

1. APF.py

  Request: `Python 2.7.6, numpy, matplotlib`
  
  In this python program, the artifitial potential field method was used to avoid the obstacles and move towards the target.
  Modify `tango` to decide where your target is, modify `init` to determin your start position.
  
2. rrt.m

  Request:`matlab/Octave`
  
  In this program, a modified RRT algorithm is implamented. There is one obstacle and one target position, a path will be found and shown in the figure. Besides the original RRT method, a improvement was desigend. The modification is mainly about deleting the nodes of the found path to make it more smooth and feasible. 

  __Notice:__ it is possible that this APF mothod takes a hell lot of time to converge or maybe it doesn't converge at the end of the world. That is not a bug of the program, but a bug of APF method.
  
