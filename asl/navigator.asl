// Demo program of Jason based navigation using A*

!navigate(d).

/*
+!navigate
	<-	InitRule = system.time;		// Get initial time stamp, for benchmarking performance
		
		// a_star(InitialState, Goal, Solution, Cost) 
		?a_star(a,d,Solution,Cost);
		
		// Print the results
		.print("solution A* =", Solution, " with cost ",Cost," in ", (system.time - InitRule), " ms.");
		.
*/

// Case where we are already at the destination
+!navigate(Destination)
	:	position(X,Y) & locationName(Destination,[X,Y])
	<-	.print("Made it to the destination!");
		-destinaton(Destination).

// We don't have a route plan, get one and set the waypoints.
+!navigate(Destination)
	:	position(X,Y) & locationName(Current,[X,Y])
	<-	.print("Generating a plan to get to: ", Destination);
		+destination(Destination);
		?a_star(Current,Destination,Solution,Cost);
		.print("Plan solution: ", Solution);
		for (.member( op(Direction,NextPosition), Solution)) {
			!waypoint(Direction,NextPosition);
		}
		!navigate(Destination).
	
// Move through the map, if possible.
+!waypoint(Direction,_)
	:	isDirection(Direction) &
		map(Direction) &
		not obstacle(Direction)
	<-	move(Direction);
		.print("move(",Direction,")");.
	
// Move through the map, if possible.
+!waypoint(Direction, Next)
	:	isDirection(Direction) &
		map(Direction) &
		obstacle(Direction)
	<-	.print("!updateMap(",Direction,", ", Next,")");.
		!updateMap(Direction, Next).



+!updateMap(Direction, NextName)
	:	position(X,Y) &
		locationName(PositionName, [X,Y]) &
		possible(PositionName,NextName) &
		destination(Destination)
	<-	-possible(PositionName,NextName)
		.print("Did map update ", Direction, " ", NextName);
		.drop_all_intentions;
		!navigate(Destination).
	

		

// Check that Direction is infact a direction
isDirection(Direction) :- (Direction = up) |
						  (Direction = down) |
						  (Direction = left) |
						  (Direction = right).
		
		
/* The following two rules are domain dependent and have to be redefined accordingly */

// sucessor definition: suc(CurrentState,NewState,Cost,Operation)
suc(Current,Next,1,up) :- ([X2,Y2] = [X1,Y1-1]) & possible(Current,Next) & nameMatch(Current,[X1,Y1],Next,[X2,Y2]).
suc(Current,Next,1,down) :- ([X2,Y2] = [X1,Y1+1]) & possible(Current,Next) & nameMatch(Current,[X1,Y1],Next,[X2,Y2]).
suc(Current,Next,1,left) :- ([X2,Y2] = [X1-1,Y1]) & possible(Current,Next) & nameMatch(Current,[X1,Y1],Next,[X2,Y2]).
suc(Current,Next,1,right) :- ([X2,Y2] = [X1+1,Y1]) & possible(Current,Next) & nameMatch(Current,[X1,Y1],Next,[X2,Y2]).

nameMatch(Current,CurrentPosition,Next,NextPosition) :- locationName(Current,CurrentPosition) &
														  locationName(Next,NextPosition).

// Map of locations that the agent can visit.
{ include("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/map.asl") }

// heutistic definition: h(CurrentState,Goal,H)
h(Current,Goal,H) :- H = math.sqrt( ((X2-X1) * (X2-X1)) + ((Y2-Y1) * (Y2-Y1)) ) &
						 nameMatch(Current,[X1,Y1],Goal,[X2,Y2]).

{ include("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/a_star.asl") }


// Default plans
+!navigate(Destination)
	<-	.print("Navigate default plan").

// Deal with case where Direction is not a valid way to go.
+!waypoint(A,B)
	<-	.print("Waypoint default plan ", A, " ", B).

+!updateMap(Direction,NextName)
	<-	.print("Map update default ",Direction, " ", NextName);
		!updateMap(Direction,NextName).
