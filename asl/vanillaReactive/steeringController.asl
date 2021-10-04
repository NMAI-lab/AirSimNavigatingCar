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

