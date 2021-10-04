/**
 * Perceptions expected by this program:
 * gps(Lat,Lon).	 	The location detected by the GPS sensor
 * compass(Bearing). 	The compass bearing to the magnetic north pole.
 * speed(Speed).		Speedometer measurement -> Not actually using this.
 *
 * Actions generated by this program:
 * steering(Setting).		Set a steering setting; a value between -1 and 1.
 * setSpeed(TargetSpeed).	Set a target speed for the car. Setting to 0 stops the car.
 *
 * Messages that this agent expects to receive:
 *
 * Messages that this agent sends:
 * 
 */

mission(mission).
+!mission(Goal,Parameters)
	:	Goal = navigate
		& Parameters = [Destination]
	<-	+mission(Goal,Parameters);
		!navigate(Destination);
		-mission(Goal,Parameters).

// Include obstacle avoidance
//{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/AgentInABox/obstacleAvoid.asl") }
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/AgentInABox/obstacleAvoidStopper.asl") }

/**
 * !navigate(Destination)
 * Used for setting up the navigation path to get from the current location to 
 * the destination.
 * Beliefs: Relevant map definitions in map.asl
 * Actions: None
 * Goals Adopted: !driveToward(Location)
 */

navigation(navigate).

 // Case where we are already at the destination
+!navigate(Destination)
	:	atLocation(Destination, Range)
	<-	.broadcast(tell, navigate(arrived(Destination,Range))).

// We don't have a route plan, get one and set the waypoints.
+!navigate(Destination)
	:	(not atLocation(Destination,_))
		& locationName(Destination,[DestLat,DestLon])
		& nearestLocation(Current,Range)
	<-	.broadcast(tell, navigate(gettingRoute(Destination), Range));
		.broadcast(tell, navigate(current(Current), CurrentRange));
		?a_star(Current,Destination,Solution,Cost);
		.broadcast(tell, navigate(route(Solution,Cost), Destination, Range));
		for (.member( op(drive,NextPosition), Solution)) {
			!waypoint(NextPosition);
		}
		!navigate(Destination).	
		
 // !navigate(Destination) - should be impossible
 +!navigate(Destination)
 	<-	.broadcast(tell, navigate(default, Destination)).

// Include rules for determining position
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/AgentInABox/driveTowardController.asl") }

// Include rules for determining position
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/AgentInABox/positioningRules.asl") }

// Steering controller.
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/AgentInABox/steeringController.asl") }
	
// Speed controller.
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/AgentInABox/speedController.asl") }

			
/**
 * A* Rules and Beliefs
 */
 
// Map of locations that the agent can visit.
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/AgentInABox/map.asl") }

// A* Nav Rules
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/AgentInABox/a_star.asl") }

// sucessor definition: suc(CurrentState,NewState,Cost,Operation)
suc(Current,Next,Range,drive) 
	:-	possible(Current,Next) 
		& locationName(Current,[CurLat,CurLon])
		& locationName(Next,[NextLat,NextLon])
		& savi_ros_java.savi_ros_bdi.navigation.range(CurLat,CurLon,NextLat,NextLon,Range).
		
// heutistic definition: h(CurrentState,Goal,H)
h(Current,Goal,Range) 
	:-	locationName(Current,[CurLat,CurLon])
		& locationName(Goal,[GoalLat,GoalLon])
		& savi_ros_java.savi_ros_bdi.navigation.range(CurLat,CurLon,GoalLat,GoalLon,Range).

