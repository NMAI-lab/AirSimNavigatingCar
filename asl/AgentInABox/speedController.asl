/**
 * !setSpeed(Speed)plans - used for setting speed.
 * Maintain knowledge of the speed setting, avoids needlessly setting it over
 * and over again.
 */

// Initial speed of the car.
speedSetting(0).
movement(controlSpeed).

// Speed is out of date, update
+!controlSpeed(Speed)
	:	speedSetting(Old)
		& (Old \== Speed)
	<-	.broadcast(tell, controlSpeed(speedSetting(Speed),oldSpeedSetting(Old)));
		-speedSetting(_);
		+speedSetting(Speed);
		setSpeed(Speed).
 
// No speed setting in the knowledge base
+!controlSpeed(Speed)
	:	not speedSetting(_)
	<-	.broadcast(tell, controlSpeed(speedSetting(Speed),oldSpeedSetting(na)));
		+speedSetting(Speed);
		setSpeed(Speed).
		
// Default plan, speed already set. Nothing to do.
+!controlSpeed(Speed)
	<-	.broadcast(tell, controlSpeed(speedSetting(Speed),default)).
