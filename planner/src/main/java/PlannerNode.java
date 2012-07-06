import java.util.ArrayList;
import java.util.List;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import brics.MapPlanner;
import brics.MarkerPlanner;
import brics.Planner;
import brics.TransitionPlanner;
import brics.types.MapPose;
import brics.types.MarkerPose;

/**
 * A Planner ros node wrapper {@link PlannerNode}.
 * 
 * @author schierl@informatik.uni-augsburg.de (Andreas Schierl)
 */
public class PlannerNode extends AbstractNodeMain {

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("planner");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {

		// set up subplanners
		MarkerPlanner marker = new MarkerPlanner(new RosMarkerPathExecutor(
				connectedNode));

		MapPlanner map = new MapPlanner(new RosMapExecutor(connectedNode));

		TransitionPlanner transition = new TransitionPlanner(marker, map,
				new RosTransitionExecutor(connectedNode));

		// set up global planner
		Planner planner = new Planner();
		planner.addPlanner(marker);
		planner.addPlanner(map);
		planner.addPlanner(transition);

		// set up ros event processing
		new RosListener(connectedNode, map, marker);

		// configure planners
		addMarkerPath(marker, 0, 1, 2, 3, 4, 5, 6, 7, 8);
		addMarkerPath(marker, 10, 11, 12, 13, 14, 15, 16, 17, 18);
		transition.addMapToMarker(new MapPose("/map1", 0, 0, 0),
				new MarkerPose(0));
		transition.addMarkerToMap(new MarkerPose(8), new MapPose("/map2", 0, 0,
				0));

		transition.addMapToMarker(new MapPose("/map2", 0, 0, 0),
				new MarkerPose(10));
		transition.addMarkerToMap(new MarkerPose(18), new MapPose("/map1", 0,
				0, 0));

		// execute a path

		// planner.executePath(planner.findPath(new MapPose("/map2", 1, 0, 0)));
	}

	public void addMarkerPath(MarkerPlanner planner, int... ids) {
		List<MarkerPose> path = new ArrayList<MarkerPose>();
		for (int id : ids)
			path.add(new MarkerPose(id));
		planner.addMarkerPath(path);
	}
}
