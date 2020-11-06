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
		drive(stop).
	
// Destination is behind us: turn and start following the path.
+!goTo(LOCATION)
	:	direction(LOCATION,behind)
	<-	.broadcast(tell, navigationUpdate(behind));
		turn(left);
		!followPath;
		!goTo(LOCATION).
		
// Destiantion is forward. Drive forward, follow the path.
+!goTo(LOCATION)
	:	direction(LOCATION,forward)
	<-	.broadcast(tell, navigationUpdate(forward));
		drive(forward);
		!followPath;
		!goTo(LOCATION).

// Destiantion is either left or right. Turn and then follow the path.
+!goTo(LOCATION)
	:	direction(LOCATION,DIRECTION) &
		((DIRECTION = left) | (DIRECTION = right))
	<-	.broadcast(tell, navigationUpdate(DIRECTION));
		turn(DIRECTION);
		!followPath;
		!goTo(LOCATION).
		
/** 
 * !followPath
 * Follow the line until a post point is visible, then stop.
 * Search for the line if it is not visible, adjust course if the line is 
 * drifting to the left or right.
 */
 
 // Case where the postPoint is visible, no driving.
 +!followPath
 	:	postPoint(_,_) | 
		direction(_,_)
	<-	.broadcast(tell, followPath(PostPointStop));
		drive(stop).
 
 
// Line is center, no post point, drive forward
+!followPath
	:	line(center) &
		(not postPoint(_,_)) &
		(not direction(_,_))
	<-	.broadcast(tell, followPath(center));
		drive(forward);
		!followPath.
		
// Line is lost, use the spiral action to try and find it.
+!followPath
	:	line(lost) & 
		(not postPoint(_,_)) &
		(not direction(_,_))
	<-	.broadcast(tell, followPath(lost));
		drive(spiral);
		!followPath.
		
// Line is accross, use the turn(left) action to re center it
+!followPath
	:	line(across) & 
		(not postPoint(_,_)) &
		(not direction(_,_))
	<-	.broadcast(tell, followPath(across));
		drive(left);
		!followPath.

// Handle cases for left and right turns.
+!followPath
	:	line(DIRECTION) & 
		((DIRECTION = left) | (DIRECTION = right)) &
		(not postPoint(_,_)) &
		(not direction(_,_))
	<-	.broadcast(tell, followPath(DIRECTION));
		drive(DIRECTION);
		!followPath.

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
		
// Ensure recursion. For example, if only a battery perception is received, you
// don't want to lose the followPath intention.
+!followPath
	<-	.broadcast(tell, followPath(default));
		drive(stop);	// Safest thing to do is not drive anywhere until something more useful is perceived.
		!followPath.

