import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import brics.TransitionExecutor;
import brics.types.MapPose;
import brics.types.MarkerPose;
import brics_msgs.mapMarkerTransitionRequest;
import brics_msgs.mapMarkerTransitionResponse;
import brics_msgs.markerMapTransitionRequest;
import brics_msgs.markerMapTransitionResponse;

public class RosTransitionExecutor implements TransitionExecutor {
	Boolean success = null;
	private ConnectedNode connectedNode;

	public RosTransitionExecutor(ConnectedNode node) {
		this.connectedNode = node;
	}

	@Override
	public boolean switchToMap(MapPose pose) {
		ServiceClient<brics_msgs.markerMapTransitionRequest, brics_msgs.markerMapTransitionResponse> service;
		try {
			service = connectedNode.newServiceClient("/transition_marker2map",
					brics_msgs.markerMapTransition._TYPE);
		} catch (ServiceNotFoundException e) {
			System.out.println("Failed to find service.");
			return false;
		}
		markerMapTransitionRequest request = service.newMessage();
		if (pose.getReference().equals("/map1"))
			request.setMapId(markerMapTransitionRequest.MAP_A);
		else if (pose.getReference().equals("/map2"))
			request.setMapId(markerMapTransitionRequest.MAP_B);
		else
			throw new IllegalArgumentException();
		request.setMapX((float) pose.getX());
		request.setMapY((float) pose.getY());
		request.setMapTh((float) pose.getTheta());
		success = null;
		service.call(request,
				new ServiceResponseListener<markerMapTransitionResponse>() {
					@Override
					public void onSuccess(markerMapTransitionResponse arg0) {
						System.out.println("Switched successfully.");
						success = true;
					}

					@Override
					public void onFailure(RemoteException arg0) {
						System.out.println("Switch failed.");
						success = false;
					}
				});

		while (success == null)
			;
		return success;
	}

	@Override
	public boolean switchToMarker(MarkerPose pose) {
		ServiceClient<brics_msgs.mapMarkerTransitionRequest, brics_msgs.mapMarkerTransitionResponse> service;
		try {
			service = connectedNode.newServiceClient("/transition_map2marker",
					brics_msgs.mapMarkerTransition._TYPE);
		} catch (ServiceNotFoundException e) {
			System.out.println("Failed to find service.");
			return false;
		}
		mapMarkerTransitionRequest request = service.newMessage();
		request.setMarkerId((byte) pose.getMarkerId());
		success = null;
		service.call(request,
				new ServiceResponseListener<mapMarkerTransitionResponse>() {
					@Override
					public void onSuccess(mapMarkerTransitionResponse arg0) {
						System.out.println("Switched successfully.");
						success = true;
					}

					@Override
					public void onFailure(RemoteException arg0) {
						System.out.println("Switch failed.");
						success = false;
					}
				});

		while (success == null)
			;
		return success;
	}
}
