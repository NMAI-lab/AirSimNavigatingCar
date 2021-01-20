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

// Trigger the plan to drive to post3.
//!navigate(post3).

/*
// Benchmark version
+path(Path)
	:	startTime(StartTime)
	<-	// Print the results
		.print("solution A* =", Path, " in ", (system.time - StartTime), " ms.");
		+done.

+!navigate(Destination)
	: 	(not done)
	<-	+startTime(system.time);		// Get initial time stamp, for benchmarking performance
		getPath(a,Destination);
		!navigate(Destination).

+!navigate(_) <- .print("Done").
*/
		
/**
 * !navigate(Destination)
 * Used for setting up the navigation path to get from the current location to 
 * the destination.
 * Beliefs: Relevant map definitions in map.asl
 * Actions: None
 * Goals Adopted: !driveToward(Location)
 */
 
 // Case where we are already at the destination
+!navigate(Destination)
	:	atLocation(Destination, Range)
	<-	.broadcast(tell, navigate(arrived(Destination,Range)));
		-destinaton(Destination).

// We don't have a route plan, get one and set the waypoints.
+!navigate(Destination)
	:	(not atLocation(Destination,_))
		& locationName(Destination,[DestLat,DestLon])
		& nearestLocation(Current,Range)
	<-	.broadcast(tell, navigate(gettingRoute(Destination), Range));
		.broadcast(tell, navigate(current(Current), CurrentRange));
		+destination(Destination);
		savi_ros_java.savi_ros_bdi.navigation.getPath(Current,Destination,Solution);
		.broadcast(tell, navigate(route(Solution), Destination));
		for (.member(NextPosition, Solution)) {
			!driveToward(NextPosition);
		}
		!navigate(Destination).	
		
 // !navigate(Destination) - should be impossible
 +!navigate(Destination)
 	<-	.broadcast(tell, navigate(default, Destination)).

	
// Include rules for determining position
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/driveTowardController.asl") }

// Include rules for determining position
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/positioningRules.asl") }

// Steering controller.
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/steeringController.asl") }
	
// Speed controller.
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/speedController.asl") }

// Map of locations that the agent can visit.
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/map.asl") }

