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
		
/**
 * Plans for steering the car.
 */
+!controlSteering(Bearing)
	:	steeringSetting(Bearing, Steering)
	<-	steering(Steering);
		.broadcast(tell, steer(steering(Steering),bearing(Bearing))).

// Default plan, should not be possible.
+!controlSteering(Bearing) <- .broadcast(tell, steer(default,bearing(Bearing))).
