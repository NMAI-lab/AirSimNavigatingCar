safety(obstacle).
@obstacleAvoidance [atomic]
+obstacle(Distance)
	:	Distance < 5.5
	<-	!controlSpeed(0.0);
		//steering(-0.3);
		.broadcast(tell, obstacleAvoid(controlSpeed(0.0))).
