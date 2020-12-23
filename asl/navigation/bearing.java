// Internal action code for project test.mas2j

package navigation;
//package savi_ros_java.savi_ros_bdi.navigation;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.NumberTerm;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Term;

public class bearing extends DefaultInternalAction {
    private static final long serialVersionUID = 1L;

    public static final int lat1Index = 0;
    public static final int lon1Index = 1;
    public static final int lat2Index = 2;
    public static final int lon2Index = 3;
    public static final int bearingResultIndex = 4;
	
    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
        // execute the internal action
        ts.getAg().getLogger().info("executing internal action 'navigation.bearing'");
        try {
            // Get the parameters
            NumberTerm lat1Term = (NumberTerm) args[lat1Index];
            NumberTerm lon1Term = (NumberTerm) args[lon1Index];
            NumberTerm lat2Term = (NumberTerm) args[lat2Index];
            NumberTerm lon2Term = (NumberTerm) args[lon2Index];

            // Lat and Lon as radians
            double lat1 = lat1Term.solve();
            double lon1 = lon1Term.solve();
            double lat2 = lat2Term.solve();
            double lon2 = lon2Term.solve();

            // Calculate results
            //double range = calculateRange(lat1, lon1, lat2, lon2, "meters");
            double bearingResult = bearing.calculateBearing(lat1, lon1, lat2, lon2);

            // Create result terms
            //NumberTerm rangeTerm = new NumberTermImpl(range);
            NumberTerm bearingTerm = new NumberTermImpl(bearingResult);

            // Unify and return
            boolean successBearing = un.unifies(bearingTerm, args[bearingResultIndex]);
            return successBearing;

        } catch (Exception e) {
            ts.getAg().getLogger().info(e.toString());
            throw new Exception("Error in 'navigation.bearing'.");
        }
    }

    public static double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
        // https://stackoverflow.com/questions/9457988/bearing-from-one-coordinate-to-another

        if (range.calculateRange(lat1, lon1, lat2, lon2, "meters") < 1) {
            return 0.0;
        }

        double longitude1 = lon1;
        double longitude2 = lon2;
        double latitude1 = Math.toRadians(lat1);
        double latitude2 = Math.toRadians(lat2);
        double longDiff = Math.toRadians(longitude2 - longitude1);
        double y = Math.sin(longDiff) * Math.cos(latitude2);
        double x = Math.cos(latitude1) * Math.sin(latitude2) - Math.sin(latitude1) * Math.cos(latitude2) * Math.cos(longDiff);

        double bearing = (Math.toDegrees(Math.atan2(y, x)) + 360) % 360;

        while (bearing > 180) {
            bearing = bearing - 360.0;
        }
        while (bearing < -180) {
            bearing = bearing + 360;
        }
        return bearing;
    }

}

