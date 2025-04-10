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

## next time
[ ] Make dubins nodes reference each other (linked list);
[ ] Compute Dubins nodes f cost;
[ ] Parameterize the vehicle dynamica in the hybrid astar;
[ ] Add vehicle positions to the nodes variables;
[ ] Check vehicle positions in collision checks;
[ ] Make Hybrid A*;
[ ] Make forward PP;
[ ] Make Backwards PP;
[ ] Publish initial position in the launch options;
