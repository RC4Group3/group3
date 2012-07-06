import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import brics.MapExecutor;
import brics.types.MapPose;
import brics_msgs.mapPlannerRequest;
import brics_msgs.mapPlannerResponse;

public class RosMapExecutor implements MapExecutor {
	Boolean success = null;
	private ConnectedNode connectedNode;

	public RosMapExecutor(ConnectedNode node) {
		this.connectedNode = node;
	}

	@Override
	public boolean execute(MapPose goal) {

		ServiceClient<brics_msgs.mapPlannerRequest, brics_msgs.mapPlannerResponse> service;
		try {
			service = connectedNode.newServiceClient("/run_map_planner",
					brics_msgs.mapPlanner._TYPE);
		} catch (ServiceNotFoundException e) {
			System.out.println("Failed to find service.");
			return false;
		}
		mapPlannerRequest request = service.newMessage();
		request.setFrameId(goal.getReference());
		request.setGoalX((float) goal.getX());
		request.setGoalY((float) goal.getY());
		request.setGoalTh((float) goal.getTheta());
		success = null;
		service.call(request,
				new ServiceResponseListener<mapPlannerResponse>() {
					@Override
					public void onSuccess(mapPlannerResponse arg0) {
						System.out.println("Map navigation succeeded.");
						success = true;
					}

					@Override
					public void onFailure(RemoteException arg0) {
						System.out.println("Map navigation failed.");
						success = false;
					}
				});

		while (success == null)
			;
		return success;
	}
}
