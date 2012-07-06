import java.util.ArrayList;
import java.util.List;

import mbn_msgs.MarkersIDs;

import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import brics.MapPlanner;
import brics.MarkerPlanner;
import brics.types.MapPose;
import brics.types.MarkerPose;
import brics_msgs.map_mode;

public class RosListener {

	public RosListener(ConnectedNode node, final MapPlanner map,
			final MarkerPlanner marker) {
		Subscriber<brics_msgs.map_mode> subscriber = node.newSubscriber(
				"/global_map_status", brics_msgs.map_mode._TYPE);
		subscriber.addMessageListener(new MessageListener<map_mode>() {
			@Override
			public void onNewMessage(map_mode mode) {
				if (mode.getMode() == map_mode.MARKER_MODE) {
					marker.setCurrentPose(new MarkerPose(mode.getMarker()));
					map.setCurrentPose(null);
				} else {
					marker.setCurrentPose(null);
					switch (mode.getMode()) {
					case map_mode.MAP_A:
						map.setCurrentPose(new MapPose("/map1", 0, 0, 0));
						break;
					case map_mode.MAP_B:
						map.setCurrentPose(new MapPose("/map2", 0, 0, 0));
						break;
					}
				}
			}
		});

		Subscriber<mbn_msgs.MarkersIDs> visibleMarkers = node.newSubscriber(
				"/markers_ids_topic", mbn_msgs.MarkersIDs._TYPE);
		visibleMarkers.addMessageListener(new MessageListener<MarkersIDs>() {
			@Override
			public void onNewMessage(MarkersIDs ids) {
				List<MarkerPose> visibleMarkers = new ArrayList<MarkerPose>();
				for (int id : ids.getMarkersIDs()) {
					visibleMarkers.add(new MarkerPose(id));
				}
				marker.setVisibleMarkers(visibleMarkers);
			}
		});
	}
}
