/**
 * !waypoint(Location)
 * Plans for driving the car toward a location called Location
 * Beliefs: gps(curLat,curtLon) - received by perception
 * 			locationName(Location,[destLat,destLon]) - should be in knowledge base
 * Actions: None
 * Goals Adopted: !controlSpeed(Speed), !controlSteering(Steering, lkaStatus), !driveToward(_)
 */
 
mission(obstacle).
mission(navigate).

+navigate(Destination)
	<-	.broadcast(tell, navigate(Destination)).

+obstacle(Distance)
	:	obstacleStop
	<-	setSpeed(0);
		.broadcast(tell, obstacle(stop));.
		
// Close enough to the location, stop.
+obstacle(Distance)
	: 	(not obstacleStop)
		& navigate(Location)
		& atLocation(Location,_)
	<-	.broadcast(tell, driveToward(arrived, Location));
		setSpeed(0).

// Approaching the location, slow down, LKA off
+obstacle(Distance)
	: 	navigate(Location)
		& nearLocation(Location,_)
		& (not atLocation(Location,_))
		& destinationBearing(Location,Bearing) 
		& (not obstacleStop)
		& steeringSetting(Bearing, Steering)
	<-	.broadcast(tell, driveToward(near, Location));
		steering(Steering);
		setSpeed(3).
		
// Drive toward the location, LKA available.
+obstacle(Distance)
	: 	navigate(Location)
		& (not nearLocation(Location,_))
		& destinationBearing(Location,Bearing)
		& (not obstacleStop)
		& lkaSteering(Steering)
	<-	.broadcast(tell, driveToward(lka(on), Location));
		steering(Steering);	
		setSpeed(8).

// Drive toward the location, LKA not available.
+obstacle(Distance)
	: 	navigate(Location)
		& (not nearLocation(Location,_))
		& destinationBearing(Location,Bearing)
		& (not obstacleStop)
		& (not lkaSteering(_))
		& steeringSetting(Bearing, Steering)
	<-	.broadcast(tell, driveToward(lka(off), Location));
		steering(Steering);	
		setSpeed(8).
		
//+obstacle(Distance)
//	<-	.broadcast(tell, obstacle(default)).
	
+!busy 
	<-	.broadcast(tell, busy);
		!busy.

