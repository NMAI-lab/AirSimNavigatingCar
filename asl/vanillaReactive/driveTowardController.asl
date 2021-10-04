/**
 * !waypoint(Location)
 * Plans for driving the car toward a location called Location
 * Beliefs: gps(curLat,curtLon) - received by perception
 * 			locationName(Location,[destLat,destLon]) - should be in knowledge base
 * Actions: None
 * Goals Adopted: !controlSpeed(Speed), !controlSteering(Steering, lkaStatus), !driveToward(_)
 */
 
+obstacle(Distance)
	:	Distance < 5.5
	<-	setSpeed(0).
		
// Close enough to the location, stop.
+gps(_,_)
	: 	navigate(Location)
		& atLocation(Location,_)
	<-	.broadcast(tell, driveToward(arrived, Location));
		setSpeed(0).

// Approaching the location, slow down, LKA off
+gps(_,_)
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
+gps(_,_)
	: 	navigate(Location)
		& (not nearLocation(Location,_))
		& destinationBearing(Location,Bearing)
		& (not obstacleStop)
		& lkaSteering(Steering)
	<-	.broadcast(tell, driveToward(main, Location));
		steering(Steering);	
		setSpeed(8).	

// Drive toward the location, LKA not available.
+gps(_,_)
	: 	navigate(Location)
		& (not nearLocation(Location,_))
		& destinationBearing(Location,Bearing)
		& (not obstacleStop)
		& not lkaSteering(_)
		& steeringSetting(Bearing, Steering)
	<-	.broadcast(tell, driveToward(main, Location));
		steering(Steering);	
		setSpeed(8).	

