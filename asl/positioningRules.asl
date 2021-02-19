/**
 * Rules for determining the nearest location, and if we are at or near a location
 */
 
// Rule for determining if the location is nearby.
nearLocation(Location, Range)
	:-	gps(CurLat,CurLon)
		& locationName(Location,[Lat,Lon])
		& savi_ros_java.savi_ros_bdi.navigation.range(CurLat,CurLon,Lat,Lon,Range)
		& Range < 20.
		
// Rule for determining if the location is nearby.
atLocation(Location, Range)
	:-	gps(CurLat,CurLon)
		& locationName(Location,[Lat,Lon])
		& savi_ros_java.savi_ros_bdi.navigation.range(CurLat,CurLon,Lat,Lon,Range)
		& Range < 7.

// Rule for determining the name, range and bearing to the nearest location
nearestLocation(Location,Range)
	:-	gps(CurLat,CurLon)
		& locationName(Location,[Lat,Lon])
		& locationName(OtherLocation,[OtherLat,OtherLon])
		& OtherLocation \== Location
		& savi_ros_java.savi_ros_bdi.navigation.range(CurLat,CurLon,Lat,Lon,Range)
		& savi_ros_java.savi_ros_bdi.navigation.range(CurLat,CurLon,OtherLat,OtherLon,OtherRange)
		& Range < OtherRange.

/**
 * Rules for calculating the range and bearing to destination.
 */ 

destinationRange(Location,Range) 
	:- 	locationName(Location,[DestLat,DestLon])
		& gps(CurLat,CurLon)
		& savi_ros_java.savi_ros_bdi.navigation.range(CurLat,CurLon,DestLat,DestLon,Range).
		
destinationBearing(Location,Bearing) 
	:- 	locationName(Location,[DestLat,DestLon])
		& gps(CurLat,CurLon)
		& savi_ros_java.savi_ros_bdi.navigation.bearing(CurLat,CurLon,DestLat,DestLon,Bearing).	
