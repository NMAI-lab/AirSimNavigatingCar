/**
 * !driveToward(Location)
 * Plans for driving the car toward a location called Location
 * Beliefs: gps(curLat,curtLon) - received by perception
 * 			locationName(Location,[destLat,destLon]) - should be in knowledge base
 * Actions: None
 * Goals Adopted: !controlSpeed(Speed), !controlSteering(Steering, lkaStatus), !driveToward(_)
 */
 
// Close enough to the location, stop.
+!driveToward(Location)
	: 	atLocation(Location,_)
	<-	.broadcast(tell, driveToward(arrived, Location));
		!controlSpeed(0);
		!controlSteering(0, lkaOff).

// Approaching the location, slow down, LKA off
+!driveToward(Location)
	: 	nearLocation(Location,_)
		& (not atLocation(Location,_))
		& destinationBearing(Location,Bearing) 
	<-	.broadcast(tell, driveToward(near, Location));
		!controlSteering(Bearing, lkaOff);
		!controlSpeed(3);
		!driveToward(Location).
		
// Drive toward the location.
+!driveToward(Location)
	: 	(not nearLocation(Location,_))
		& destinationBearing(Location,Bearing) 
	<-	.broadcast(tell, driveToward(main, Location));
		!controlSteering(Bearing, lkaOn);	
		!controlSpeed(8);
		!driveToward(Location).
		
// !driveToward(Location) default
+!driveToward(Location)
	<-	.broadcast(tell, driveToward(default, Location));
		!driveToward(Location).
