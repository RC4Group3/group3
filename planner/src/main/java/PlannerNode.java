import java.util.Arrays;

import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import brics.MarkerPlanner;
import brics.types.MarkerPose;
import brics_msgs.markerFollowerRequest;
import brics_msgs.markerFollowerResponse;

/**
 * A Planner ros node wrapper {@link PlannerNode}.
 * 
 * @author schierl@informatik.uni-augsburg.de (Andreas Schierl)
 */
public class PlannerNode extends AbstractNodeMain {

	private ConnectedNode connectedNode;

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("planner");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		this.connectedNode = connectedNode;
		
		MarkerPlanner mp = new MarkerPlanner(new RosMarkerPathExecutor(connectedNode));
//		MarkerPlanner mp = new MarkerPlanner(new DummyMarkerPathExecutor());
		mp.addMarkerPath(new MarkerPose(1),new MarkerPose(2), new MarkerPose(3), new MarkerPose(4), new MarkerPose(5));
		mp.setCurrentPose(new MarkerPose(1));
		boolean result = mp.moveTo(new MarkerPose(3));
		if(result) System.out.println("goal reached");
		else System.out.println("goal not reached");
	}
}
