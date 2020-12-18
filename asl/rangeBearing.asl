
!test.

// Bearing calculation
// https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/


// Range Calculation
// https://en.wikipedia.org/wiki/Haversine_formula#:~:text=The%20haversine%20formula%20determines%20the,and%20angles%20of%20spherical%20triangles.
// lat and lon need to be in radians

//range(A,B,D) :-	D = 2 * Re * math.arcsin(math.sqrt((math.sin((Lat2-Lat1)/2 * math.sin((Lat2-Lat1)/2))) + (math.cos(Lat1) * math.cos(Lat2) * (math.sin((Lon2-Lon1)/2) * math.sin((Lon2-Lon1)/2)))))
//					& r_e(Re)
//					& [Lat1Deg,Lon1Deg] = A
//					& [Lat2Deg,Lon2Deg] = B
//					& deg2rad(Lat1Deg, Lat1)
//					& deg2rad(Lon1Deg, Lon1)
//					& deg2rad(Lat2Deg, Lat2)
//					& deg2rad(Lon2Deg, Lon2).



//calc(Output, Input) :- Output = functions.math.sin(Input).

					//& [Lat1Deg,Lon1Deg] = A
					//& [Lat2Deg,Lon2Deg] = B
					//& deg2rad(Lat1Deg, Lat1)
					//& deg2rad(Lon1Deg, Lon1)
					//& deg2rad(Lat2Deg, Lat2)
					//& deg2rad(Lon2Deg, Lon2).

// R = 6371 km; radius of Earth
//r_e(Re) :- Re = 6371000. // m

// Degrees to radians
// 1Deg × ?/180 = 0.01745Rad
//deg2rad(Deg, Rad) :- Rad = Deg * (math.pi / 180.0). 


+!test
	:	navigation.rangeBearing(45.4215,-75.6972,43.6532,-79.3832,Range,Bearing)
	<-	.print("Range (m): ", Range);
		.print("Bearing (rad): ", Bearing).
		
		
		
