# TODO list

## 20/02
[x] 16:30 Port to classic;
[x] 17:00 Fix the publisher;

## 26/02
[x] 15:22 Theres an invisible wall in front of the tractor (trailer wheel height);
[x] 15:38 Test localization problem with classic;
[x] 16:32 Configure launch for new world;

## 27/02
[x] 17:07 Fix the transformation of the publisher (again);
[x] 17:13 Include navigation in the launch file;
[x] 17:14 Bound the transformation from -45 to 45;

## 06/03
[x] 14:57 Start making the plugin (make a straight planner);
[x] 16:53 Interact with costmap in straight planner;

## 07/03
[x] 15:37 Make a simple A* where you can interact with obstacle layers;

## 11/03
[x] 15:35 Add max steering to the hybrid astar;

## 12/03
[x] 15:00 Remake astar to me more scalable;

## 13/03
[x] 15:40 Finished remaking Astar to be more like mine;
[X] 15:41 Start remaking hybrid astar (creating node object?????);
[x] 16:47 Add node in algorithm
[x] 17:33 Add tolerance;

## 14/03
[X] 16:40 Create voronoigraph using python;

## 18/03
[x] 16:03 Make export of the nodes to be read (csv with some issues);
[x] 16:15 Make export of the nodes to be read (txt);

## 20/03
[x] 15:42 Make voronoi class;

## 21/03
[x] 17:22 Read nodes' positions in the planner;

## 01/04
[x] 16:33 Use the read nodes instead of sampling cspace;

## 02/04
[x] 17:00 Understand the search algorithm;

## 03/04
[x] 16:28 Start introducing the dubins path;

## 08/04
[x] 15:52 Make Dubins path between 2 nodes;

## 09/04
[x] 16:19 Make loop through sub_goals to search for a dubins path;

## 10/04
[x] 11:20 Have the search update to other nodes when path to goal isnt found;
[x] 11:42 Dubins search fully functional;
[x] 14:17 Ask professor for advice on optimization (Solution: lower refresh rate);
[x] 14:31 Changed the nav_to_pose bt to lower the refresh rate;

## 11/04
- Found edge case where if a u turn is necessary in the beginnig or the end, the path isnt computed;
[x] 16:59 Make dubins nodes reference each other (linked list) 
    - ONLY FOR DUBINS AND VORONOI, WILL NEED REVISITING WHEN HYBRID ASTAR IS IMPLEMENTED;

## 15/04
[x] 14:48 Parameterize the vehicle dynamics in the hybrid astar;

## 16/04 NODE EXPANSION
[x] 15:47 Calculate the new directions based on the parameters introduced;

## 22/04 NODE EXPANSION
[x] 14:15 Refractored the code into new files for the voronoi and auxiliary funtions;
[x] 14:50 Calculate the new nodes;
[x] 14:50 Add new nodes to the open list;

## 23/04 NODE EXPANSION
[x] 16:02 Can create paths using the backup nodes;
[x] 17:00 Make Hybrid A*;

# Path creation is now fully operational through the tractor only

## 24/04 VEHICLE DYNAMICS
[x] 14:48 Added costs to the bubins path;
[x] 16:07 Add vehicle positions to the nodes variables;

## 29/04 VEHICLE DYNAMICS
[x] 14:11 Get initial transformation;
[x] 15:00 Check vehicle positions in collision checks;

## 02/05 VEHICLE DYNAMICS
[x] 15:50 Implement vehicle position calculation inside the collision check;

## 06/05 VEHICLE DYNAMICS
[x] 15:20 Change dynamics calculation to the dubins calculation;
[x] 15:27 Fix continuation of trailer positions in the node expansion;
[x] 16:00 Detect backwards movement to alter speed direction;

## 07/05 VEHICLE DYNAMICS
[x] 11:15 Fix memory issue with the trailers positions in the nodes;

## 08/05 VEHICLE DYNAMICS
[x] 16:00 FInd reason for crashing sometimes;

## 09/05 HYBRID ASTAR
[x] 11:30 Fix Dubins to not simply walk backwards;

## 13/05
[x] 16:22 Fix crash problem; 

## 14/05
[x] 15:41 Added headless mode for gazebo;
[x] 16:00 Make tolerance for the angle on the hybrid astar;

## 15/05
[x] Make create_hybrid_segment() function;
[x] Tune the cost not to switch directions a lot;
[x] Fix/tune the trailer position calculations;

## 20/05
[x] Make node selection reasonable;
[x] Make forward PP (dont know if itll work);

## 25/05
[x] Publish initial position in the amcl options;
[x] Make reverse PP;

## 26/06
[ ] Debug trailer rotations that seem to be reversed;



## next time
[ ] Make the search have a depth (can only go for some amount of time/nodes); // No need?
