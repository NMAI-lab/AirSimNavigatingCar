
// Trigger the plan to drive to post3.
!driveToward(post4).

// Expected compass declination. TODO: make this tunable.
declanation(7.5).

// Initial speed of the car.
gps(47.6426242556,-122.140354517).
compass(-90).

// Map of locations that the agent can visit.
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/map.asl") }

/**
 * Rule used for calculating the steering angles based on compass angle and
 * target bearing.
 */ 
 
courseCorrection(TargetBearing, Correction)
	:-	compass(CurrentBearing)
		& declanation(Declanation)
		& (Correction = TargetBearing - (CurrentBearing + Declanation)).

destinationRange(Location,Range) 
	:- 	locationName(Location,[DestLat,DestLon])
		& gps(CurLat,CurLon)
		& navigation.range(CurLat,CurLon,DestLat,DestLon,Range).
		
destinationBearing(Location,Bearing) 
	:- 	locationName(Location,[DestLat,DestLon])
		& gps(CurLat,CurLon)
		& navigation.bearing(CurLat,CurLon,DestLat,DestLon,Bearing).	

// Close enough to the location, stop.
+!driveToward(Location)
	: 	destinationBearing(Location,Bearing)
		& destinationRange(Location,Range)
	 	& (Range < 12)
	<-	.print("tell, driveToward(arrived, Location, Range,Bearing)");
		!steer(Bearing).

// Approaching the location, slow down
+!driveToward(Location)
	: 	destinationBearing(Location,Bearing)
		& destinationRange(Location,Range)
	 	& (Range < 20) 
		& (Range >= 12)
	<-	.print("tell, driveToward(near, Location, Range,Bearing)");
		!steer(Bearing);.
		
// Drive toward the location.
+!driveToward(Location)
	: 	destinationBearing(Location,Bearing)
		//& destinationRange(Location,Range)
	 	//& (Range >= 20)
	<-	.print("tell, driveToward(main, Location, Range, Bearing)");
		!steer(Bearing).


+!steer(Bearing)
	:	courseCorrection(Bearing, Correction)
	  	& (Correction >= 20)
	<-	.print("tell, steer(1, Bearing, Correction)");
		.print(steering(1)).
	
+!steer(Bearing)
	:	courseCorrection(Bearing, Correction)
		& (Correction <= -20)
	<-	.print("tell, steer(2, Bearing, Correction)");
		.print(steering(-1)).
 
+!steer(Bearing)
	:	courseCorrection(Bearing, Correction)
		//& (Correction < 20)
		//& (Correction > -20)
	<-	.print("tell, steer(3, Bearing, Correction)");
		.print(steering(Correction/180)).
		
+!driveToward(_) <- .print("Drive default").
+!steer(_) <- .print("Steer default").
