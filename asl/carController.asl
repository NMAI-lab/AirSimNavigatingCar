
obstacleStop 
	:-	obstacle(Distance)
		& Distance < 7.0.

// Agent in a Box Car Controller
{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/AgentInABox/carController.asl") }

// Vanilla, Goal Directed Car Controller
//{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/vanillaGoalDirected/CarController.asl") }

// Vanilla, Reactive Car Controller
//{ include("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/vanillaReactive/CarController.asl") }
