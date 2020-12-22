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
//!goTo(post3).
!driveToward(post2).

// plans for updating position and bearing knowledge
//+gps(Lat,Lon)
//	:	position(_,_)
//	<-	-position(_,_);
//		+position(Lat,Lon).
		
//+gps(Lat,Lon).

// Expected compass declination. TODO: make this tunable.
declanation(7.5).

// Initial speed of the car.
speedSetting(0).

// Used for testing the steering plans.
//!steer(100).
//compass(90).

/**
 * Rule used for calculating the steering angles based on compass angle and
 * target bearing.
 */ 
courseCorrection(TargetBearing, Correction) :- compass(CurrentBearing) & 
								declanation(Declanation) &
								Correction = TargetBearing - (CurrentBearing + Declanation).

destinationRangeBearing(Location,Range,Bearing) :- locationName(Location,[DestLat,DestLon])
										& gps(CurLat,CurLon)
										& savi_ros_java.savi_ros_bdi.navigation.rangeBearing(CurLat,CurLon,DestLat,DestLon,Range,Bearing).
										
		
/**
 * !driveToward(Location)
 * Plans for driving the car toward a location called Location
 * Beliefs: gps(curLat,curtLon) - received by perception
 * 			locationName(Location,[destLat,destLon]) - should be in knowledge base
 * Actions: None
 * Goals Adopted: !drive(_), !steer(_), !driveToward(_)
 */
 
// Drive toward the location.
+!driveToward(Location)
	: 	destinationRangeBearing(Location,Range,Bearing)
	 	& Range >= 40
	<-	.broadcast(tell, driveToward(main, Location, Range, Bearing));
		!drive(8);
		!steer(Bearing);
		!driveToward(Location).
		
// Close enough to the location, stop.
+!driveToward(Location)
	: 	destinationRangeBearing(Location,Range,Bearing)
	 	& Range < 40
	<-	.broadcast(tell, driveToward(arrived, Location, Range,Bearing));
		!drive(0);
		!steer(Bearing).
		
/**
 * Steering controller plans, based on compass angles for target bearing and
 * compass 
 * measurement.
 *
 * Beliefs:		declanation(Declanation)
 				compass(CurrentBearing)[percept]
 * Actions:		steering(steeringSetting)
 */
+!steer(Bearing)
	:	courseCorrection(Bearing, Correction) &
		math.abs(Correction) < 20
	<-	.broadcast(tell, steer(1, Bearing));
		steering(Correction/180).
	
+!steer(Bearing)
	:	courseCorrection(Bearing, Correction) &
		math.abs(Correction) >= 20 &
		Correction > 0
	<-	.broadcast(tell, steer(2, Bearing));
		steering(1).
	
+!steer(Bearing)
	:	courseCorrection(Bearing, Correction) &
		math.abs(Correction) >= 20 &
		Correction < 0
	<-	.broadcast(tell, steer(3, Bearing));
		steering(-1).
	
	
/**
 * !drive(Speed)plans - used for setting speed.
 * Maintain knowledge of the speed setting, avoids needlessly setting it over
 * and over again.
 */
 
// Speed is out of date, update
+!drive(Speed)
	:	speedSetting(Old) &
		not (Old = Speed)
	<-	.broadcast(tell, drive(1, Speed));
		-speedSetting(_);
		+speedSetting(Speed);
		setSpeed(Speed).
 
// No speed setting in the knowledge base
+!drive(Speed)
	:	not speedSetting(_)
	<-	.broadcast(tell, drive(2, Speed));
		+speedSetting(Speed);
		setSpeed(Speed).
		
// Nothing to do, speed already set
+!drive(Speed)
	:	speedSetting(Speed)
	<-	.broadcast(tell, drive(3, Speed)).
	

/**
 * Default plans
 */
 
// !driveToward(Location) default
+!driveToward(Location)
	<-	.broadcast(tell, driveToward(default, Location));
		!driveToward(Location).
 
// !steer(Bearing), don't drop.
+!steer(Bearing) 
	<- 	.broadcast(tell, steer(default, Bearing));
		!steer(Bearing).
	
// !drive(Speed), should be impossible to reach.
+!drive(Speed) 
	<-	.broadcast(tell, drive(default, Speed)).
 

// Map of locations that the agent can visit.
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/map.asl") }

// A Star Nav
//{ include("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/a_star.asl") }
