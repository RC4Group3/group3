import java.util.List;

import brics.MapPlanner;
import brics.MarkerPlanner;
import brics.Planner;
import brics.Planner.Edge;
import brics.TransitionPlanner;
import brics.types.MapPose;
import brics.types.MarkerPose;

public class Main {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		MapPlanner mapPlanner = new MapPlanner(null);

		MarkerPlanner markerPlanner = new MarkerPlanner(new DummyMarkerPathExecutor());
		markerPlanner.addMarkerPath(new MarkerPose(1), new MarkerPose(2),
				new MarkerPose(3), new MarkerPose(4));

		TransitionPlanner transitionPlanner = new TransitionPlanner(
				markerPlanner, mapPlanner, null);
		transitionPlanner.addMapToMarker(new MapPose("A", 0, 0, 0),
				new MarkerPose(1));
		transitionPlanner.addMarkerToMap(new MarkerPose(4), new MapPose("B", 0,
				0, 0));

		Planner planner = new Planner();

		planner.addPlanner(markerPlanner);
		planner.addPlanner(mapPlanner);
		planner.addPlanner(transitionPlanner);

		List<Edge> path;
		mapPlanner.setCurrentPose(new MapPose("A", 1, 0, 0));
		markerPlanner.setCurrentPose(null);
		System.out.println("Trying to go to A: 2.0/0.0");
		path = planner.findPath(new MapPose("A", 2, 0, 0));
		planner.executePath(path);
		System.out.println();

		mapPlanner.setCurrentPose(new MapPose("A", 1, 0, 0));
		markerPlanner.setCurrentPose(null);
		System.out.println("Trying to go to Marker 1");
		path = planner.findPath(new MarkerPose(1));
		planner.executePath(path);
		System.out.println();

		mapPlanner.setCurrentPose(new MapPose("A", 1, 0, 0));
		markerPlanner.setCurrentPose(null);
		System.out.println("Trying to go to Marker 3");
		path = planner.findPath(new MarkerPose(3));
		planner.executePath(path);
		System.out.println();

		mapPlanner.setCurrentPose(new MapPose("A", 1, 0, 0));
		markerPlanner.setCurrentPose(null);
		System.out.println("Trying to go to B: 2.0/0.0");
		path = planner.findPath(new MapPose("B", 2, 0, 0));
		planner.executePath(path);
		System.out.println();

	}

}
