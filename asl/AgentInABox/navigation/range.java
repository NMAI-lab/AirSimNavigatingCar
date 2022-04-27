//package savi_ros_java.savi_ros_bdi.navigation;
package navigation;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.NumberTerm;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Term;

public class range extends DefaultInternalAction {
    private static final long serialVersionUID = 1L;

    public static final int lat1Index = 0;
    public static final int lon1Index = 1;
    public static final int lat2Index = 2;
    public static final int lon2Index = 3;
    public static final int rangeResultIndex = 4;

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
            double lat1 = lat1Term.solve();
            double lon1 = lon1Term.solve();
            double lat2 = lat2Term.solve();
            double lon2 = lon2Term.solve();

            // Calculate results
            double rangeResult = range.calculateRange(lat1, lon1, lat2, lon2, "meters");

            // Create result terms
            NumberTerm rangeTerm = new NumberTermImpl(rangeResult);

            // Unify and return
            boolean successRange = un.unifies(rangeTerm, args[rangeResultIndex]);
            return successRange;

        } catch (Exception e) {
            ts.getAg().getLogger().info(e.toString());
            throw new Exception("Error in 'range'.");
        }
    }

    // https://www.geodatasource.com/developers/java
    public static double calculateRange(double lat1, double lon1, double lat2, double lon2, String unit) {
        if ((lat1 == lat2) && (lon1 == lon2)) {
            return 0;
        } else {
            double theta = lon1 - lon2;
            double dist = Math.sin(Math.toRadians(lat1)) * Math.sin(Math.toRadians(lat2)) + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) * Math.cos(Math.toRadians(theta));
            dist = Math.acos(dist);
            dist = Math.toDegrees(dist);
            dist = dist * 60 * 1.1515;

            if (unit.equals("K")) {
                dist = dist * 1.609344;
            } else if (unit.equals("N")) {
                dist = dist * 0.8684;
            } else if (unit.equals("meters")) {
                dist = dist * 1.609344 * 1000.0;
            }
            return dist;
        }
    }
}

