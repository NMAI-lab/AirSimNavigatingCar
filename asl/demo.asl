/**
 * @author	Patrick Gavigan
 * @date	6 November 2020
 */

/** 
 * !goTo(LOCATION,WATCHDOG)
 * Used for navigating the robot via the post points. Decide if the robot needs 
 * to turn or drive forward. Uses the sub goal of !followPath to move between 
 * post points.
 * 
 * LOCATION: The location we want to go to
 * SET_DESTINATION: The location that the agent has set itself to navigate to
 */

 // Case where the robot has not yet set a destination to navigate to. Need to 
 // set the destination.
+!goTo(LOCATION)
	:	direction(unknown,_)
	<-	.broadcast(tell, navigationUpdate(setDestination,LOCATION));
		setDestination(LOCATION);	// Set the destination in the navigation module
		!goTo(LOCATION).

 // The robot has a different destination than the one we need to go to.
 +!goTo(LOCATION)
	:	direction(OLD,_) &
		(not (OLD = LOCATION))
	<-	.broadcast(tell, navigationUpdate(updateDestination,LOCATION));
		setDestination(LOCATION);	// Set the destination in the navigation module
		!goTo(LOCATION).
		
// Case where the robot has arrived at the destination.
+!goTo(LOCATION)
	:	direction(LOCATION,arrived)
	<-	.broadcast(tell, navigationUpdate(arrived));
		setSpeed(0.0).
	
// Destination is behind us: turn and start following the path.
+!goTo(LOCATION)
	:	direction(LOCATION,behind)
	<-	.broadcast(tell, navigationUpdate(behind));
		turn(left);
		!goTo(LOCATION).
		
// Destiantion is forward. Drive forward, follow the path.
+!goTo(LOCATION)
	:	direction(LOCATION,forward)
	<-	.broadcast(tell, navigationUpdate(forward));
		setSpeed(8.0);
		!goTo(LOCATION).

// Destiantion is either left or right. Turn and then follow the path.
+!goTo(LOCATION)
	:	direction(LOCATION,DIRECTION) &
		((DIRECTION = left) | (DIRECTION = right))
	<-	.broadcast(tell, navigationUpdate(DIRECTION));
		turn(DIRECTION);
		!goTo(LOCATION).


		
/**
 * Default plans.
 * These can run when unrelated perceptions are received, resulting in a 
 * reasoning cycle where no plan context is applicable for that reasoning cycle.
 */
// Deal with the scenario where the reasoning cycle runs on a perception other 
// than a post point. Increment WATCHDOG and try again.
+!goTo(LOCATION)
	<-	.broadcast(tell, goTo(default, LOCATION));
		!goTo(LOCATION).

