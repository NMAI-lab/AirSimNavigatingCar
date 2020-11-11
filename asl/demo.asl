/**
 * @author	Patrick Gavigan
 * @date	6 November 2020
 */

!goTo(post4).
 
/** 
 * !goTo(LOCATION)
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
		!stopDriving.
	
// Destination is behind us: turn and start following the path.
+!goTo(LOCATION)
	:	direction(LOCATION,behind)
	<-	.broadcast(tell, navigationUpdate(behind));
		!stopDriving;
		turn(left);
		!goTo(LOCATION).
		
// Destiantion is forward. Drive forward, follow the path.
+!goTo(LOCATION)
	:	direction(LOCATION,forward)
	<-	.broadcast(tell, navigationUpdate(forward));
		!followPath;
		!goTo(LOCATION).

// Destiantion is either left or right. Turn and then follow the path.
+!goTo(LOCATION)
	:	direction(LOCATION,DIRECTION) &
		((DIRECTION = left) | (DIRECTION = right))
	<-	.broadcast(tell, navigationUpdate(DIRECTION));
		//!stopDriving;
		turn(DIRECTION);
		!goTo(LOCATION).

+!followPath
	: 	(not driving) | 
		(speed(SPEED) & SPEED = 0.0)
	<-	.broadcast(tell, followPath(startDriving));
		+driving;
		setSpeed(8.0).
		
+!stopDriving
	:	driving |
		(speed(SPEED) & not (SPEED = 0.0))
	<-	.broadcast(tell, followPath(startDriving));
		-driving;
		setSpeed(0.0).
		
/**
 * Default plans.
 * These can run when unrelated perceptions are received, resulting in a 
 * reasoning cycle where no plan context is applicable for that reasoning cycle.
 */
+!goTo(LOCATION)
	<-	.broadcast(tell, goTo(default, LOCATION));
		!goTo(LOCATION).
		
+!followPath
	<-	.broadcast(tell, followPath(default)).

+!stopDriving
	<-	.broadcast(tell, stopDriving(default)).

