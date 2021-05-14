/**
 * Steering controller plans, based on compass angles for target bearing and
 * compass 
 * measurement.
 *
 * Beliefs:		declanation(Declanation)
 				compass(CurrentBearing)[percept]
 * Actions:		steering(steeringSetting)
 * Messages:	diagnostic messages.
 */
 
// Expected compass declination. TODO: make this tunable.
declanation(7.5).

/**
 * Calculate the course correction
 */
courseCorrection(TargetBearing, Correction)
	:-	compass(CurrentBearing)
		& declanation(Declanation)
		& (Correction = TargetBearing - (CurrentBearing + Declanation)).
		
/**
 * Rule used for calculating the steering setting based on course correction
 * target bearing.
 * steeringSetting(TargetBearing, SteeringSetting)
 */ 

steeringSetting(TargetBearing, 1)
	:-	courseCorrection(TargetBearing, Correction)
		& (Correction >= 20).
 
steeringSetting(TargetBearing, -1)
	:-	courseCorrection(TargetBearing, Correction)
		& (Correction <= -20).
		
steeringSetting(TargetBearing, Correction/180)
	:-	courseCorrection(TargetBearing, Correction)
		& (Correction < 20)
		& (Correction > -20).
		
lkaSteering(Steering)
	:-	lane(Steering,A,B,C,D)
		& ((not (C == 0)) | (not (D == 0))).
	
		
/**
 * Plans for steering the car.
 */

movement(controlSteering).

 // LKA not available or turned off, use compass
 //+!controlSteering(Bearing, lkaSetting)
+!controlSteering(Bearing,LkaSetting)
	:	steeringSetting(Bearing, Steering)
		& ( (not lkaSteering(_)) | (LkaSetting == lkaOff))
	<-	steering(Steering);
		.broadcast(tell, compassSteer(steering(Steering),bearing(Bearing))).

// LKA available and enabled, use it
+!controlSteering(Bearing,lkaOn)
	:	lkaSteering(Steering)
	<-	steering(Steering);
		.broadcast(tell, lkaSteer(steering(Steering),bearing(Bearing))).
		
// Default plan, should not be possible.
+!controlSteering(Bearing) <- .broadcast(tell, steer(default,bearing(Bearing))).

