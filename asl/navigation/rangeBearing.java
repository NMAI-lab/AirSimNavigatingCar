// Internal action code for project test.mas2j

package navigation;

import jason.*;
import jason.asSemantics.*;
import jason.asSyntax.*;
import java.lang.Math; 


public class rangeBearing extends DefaultInternalAction {

	private static final long serialVersionUID = 1L;
	
	public static final int lat1Index = 0;
	public static final int lon1Index = 1;
	public static final int lat2Index = 2;
	public static final int lon2Index = 3;
	public static final int rangeResultIndex = 4;
	public static final int bearingResultIndex = 5;
	
    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
        // execute the internal action
        ts.getAg().getLogger().info("executing internal action 'navigation.range'");
        
		try {
			// Get the parameters
			NumberTerm lat1Term = (NumberTerm) args[lat1Index];
			NumberTerm lon1Term = (NumberTerm) args[lon1Index];
			NumberTerm lat2Term = (NumberTerm) args[lat2Index];
			NumberTerm lon2Term = (NumberTerm) args[lon2Index];
			
			// Lat and Lon as radians
			double lat1 = Math.toRadians(lat1Term.solve());
			double lon1 = Math.toRadians(lon1Term.solve());
			double lat2 = Math.toRadians(lat2Term.solve());
			double lon2 = Math.toRadians(lon2Term.solve());
			
			// https://en.wikipedia.org/wiki/Haversine_formula#:~:text=The%20haversine%20formula%20determines%20the,and%20angles%20of%20spherical%20triangles
			// http://edwilliams.org/avform.htm
			double radius_m = 6356752; // m
			double distance_radians = 2 * Math.asin(Math.sqrt(Math.pow((Math.sin((lat1-lat2)/2)),2) + Math.cos(lat1)*Math.cos(lat2)*Math.pow((Math.sin((lon1-lon2)/2)),2)));
			double d = radius_m * distance_radians;
		
			// We obtain the initial course, tc1, (at point 1) from point 1 to point 2 by the following. The formula fails if the initial point is a pole. We can special case this with:
			double tc1 = 0;
			double eps = 0.0001;
			if (Math.cos(lat1) < eps) { // EPS a small number ~ machine precision
				if (lat1 > 0) {
					tc1 = Math.PI;    //  starting from N pole
				} else {
					tc1 = 2 * Math.PI; //  starting from S pole
				}
			}
			// For starting points other than the poles:
			if (Math.sin(lon2-lon1) < 0) {
				tc1 = Math.acos((Math.sin(lat2)-Math.sin(lat1)*Math.cos(d))/(Math.sin(d)*Math.cos(lat1)));
			} else {
				tc1 = 2 * Math.PI - Math.acos((Math.sin(lat2)-Math.sin(lat1)*Math.cos(d))/(Math.sin(d)*Math.cos(lat1)));
			}
			
			// Create result terms
			NumberTerm rangeTerm = new NumberTermImpl(d);
			NumberTerm bearingTerm = new NumberTermImpl(tc1);
		
			// Unify and return
			boolean successRange = un.unifies(rangeTerm, args[rangeResultIndex]);
			boolean successBearing = un.unifies(bearingTerm, args[bearingResultIndex]);
			return successRange && successBearing;
		
		} catch (Exception e) {
			throw new Exception("Error in 'rangeBearing'.");
		}
    }
}
