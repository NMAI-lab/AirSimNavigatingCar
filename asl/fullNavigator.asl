
// Expected compass declination. TODO: make this tunable.
declanation(7.5).

// Used for testing
!steer(100).
compass(90).

/**
 * Rule used for calculating the steering angles based on compass angle and
 * target bearing.
 */ 
courseCorrection(TargetBearing, Correction) :- compass(CurrentBearing) & 
								declanation(Declanation) &
								Correction = TargetBearing - (CurrentBearing + Declanation).

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
	<-	steering(Correction/180).
	
+!steer(Bearing)
	:	courseCorrection(Bearing, Correction) &
		math.abs(Correction) >= 20 &
		Correction > 0
	<-	steering(1).
	
+!steer(Bearing)
	:	courseCorrection(Bearing, Correction) &
		math.abs(Correction) >= 20 &
		Correction < 0
	<-	steering(-1).
	
// Default plan, don't drop.
+!steer(Bearing) <- !steer(Bearing).
