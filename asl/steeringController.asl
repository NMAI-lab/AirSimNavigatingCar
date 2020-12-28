/**
 * Steering controller plans, based on compass angles for target bearing and
 * compass 
 * measurement.
 *
 * Beliefs:		declanation(Declanation)
 				compass(CurrentBearing)[percept]
 * Actions:		steering(steeringSetting)
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
+!steer(Bearing)
	:	steeringSetting(Bearing, Steering)
	<-	.print(steering(Steering)).

+!steer(_) <- .print("Steer default").
