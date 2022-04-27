/**
 * !waypoint(Location)
 * Plans for driving the car toward a location called Location
 * Beliefs: gps(curLat,curtLon) - received by perception
 * 			locationName(Location,[destLat,destLon]) - should be in knowledge base
 * Actions: None
 * Goals Adopted: !controlSpeed(Speed), !controlSteering(Steering, lkaStatus), !driveToward(_)
 */
 
movement(waypoint).

// Close enough to the location, stop.
+!waypoint(Location)
	: 	atLocation(Location,_)
	<-	.broadcast(tell, driveToward(arrived, Location));
		!controlSpeed(0);
		!controlSteering(0, lkaOff).
		
+!waypoint(Location)
	: 	obstacleStop
	<-	.broadcast(tell, driveToward(obstacleStop, Location));
		!controlSpeed(0).

// Approaching the location, slow down, LKA off
+!waypoint(Location)
	: 	nearLocation(Location,_)
		& (not atLocation(Location,_))
		& destinationBearing(Location,Bearing) 
		& (not obstacleStop)
	<-	.broadcast(tell, driveToward(near, Location));
		!controlSteering(Bearing, lkaOff);
		!controlSpeed(3);
		!waypoint(Location).
		
// Drive toward the location.
+!waypoint(Location)
	: 	(not nearLocation(Location,_))
		& destinationBearing(Location,Bearing)
		& (not obstacleStop)
	<-	.broadcast(tell, driveToward(main, Location));
		!controlSteering(Bearing, lkaOn);	
		!controlSpeed(8);
		!waypoint(Location).
		
// !driveToward(Location) default
+!waypoint(Location)
	<-	.broadcast(tell, driveToward(default, Location));
		!waypoint(Location).
