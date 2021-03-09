
+obstacle(Distance)
	:	Distance < 10
	<-	steering(-0.6);
		.broadcast(tell, obstacleAvoid(steering(-0.4))).
