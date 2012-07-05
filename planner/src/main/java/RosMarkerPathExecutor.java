import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import brics.MarkerPathExecutor;
import brics_msgs.markerFollowerRequest;
import brics_msgs.markerFollowerResponse;

public class RosMarkerPathExecutor implements MarkerPathExecutor {
	Boolean success = null;
	private ConnectedNode connectedNode;

	public RosMarkerPathExecutor(ConnectedNode node) {
		this.connectedNode = node;
	}

	@Override
	public boolean execute(byte[] markerPath) {

		ServiceClient<brics_msgs.markerFollowerRequest, brics_msgs.markerFollowerResponse> service;
		try {
			service = connectedNode.newServiceClient("run_ar_follower",
					brics_msgs.markerFollower._TYPE);
		} catch (ServiceNotFoundException e) {
			System.out.println("Failed to find service.");
			return false;
		}
		markerFollowerRequest request = service.newMessage();
		request.setIds(markerPath);
		success = null;
		service.call(request,
				new ServiceResponseListener<markerFollowerResponse>() {
					@Override
					public void onSuccess(markerFollowerResponse arg0) {
						System.out.println("Path following succeeded.");
						success = true;
					}

					@Override
					public void onFailure(RemoteException arg0) {
						System.out.println("Path following failed");
						success = false;
					}
				});

		while (success == null)
			;
		return success;
	}
}
