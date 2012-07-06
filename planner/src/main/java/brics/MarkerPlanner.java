package brics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import brics.types.MarkerPose;
import brics.types.Pose;

public class MarkerPlanner implements SubPlanner {
	private MarkerPose currentPose = null;
	private final MarkerPathExecutor executor;
	private List<List<MarkerPose>> markerPaths = new ArrayList<List<MarkerPose>>();
	private List<Pose> interestingPoses = new ArrayList<Pose>();
	private List<MarkerPose> visibleMarkers = new ArrayList<MarkerPose>();

	public MarkerPlanner(MarkerPathExecutor executor) {
		this.executor = executor;
	}

	public void addMarkerPath(List<MarkerPose> path) {
		markerPaths.add(path);
	}

	public void addMarkerPath(MarkerPose... path) {
		addMarkerPath(Arrays.asList(path));
	}

	@Override
	public Map<Pose, Double> getReachablePoses(Pose start) {
		Map<Pose, Double> ret = new HashMap<Pose, Double>();
		if (start == null) {
			for (MarkerPose visibleMarker : getVisibleMarkers()) {
				ret.put(visibleMarker, 0d);
				Map<Pose, Double> reachablePoses = getReachablePoses(visibleMarker);
				for (Entry<Pose, Double> pose : reachablePoses.entrySet()) {
					ret.put(pose.getKey(), pose.getValue());
				}
			}
			return ret;
		} else {
			for (List<MarkerPose> path : markerPaths) {
				if (path.contains(start)) {
					boolean found = false;
					for (MarkerPose pose : path) {
						if (found && interestingPoses.contains(pose))
							ret.put(pose, 1d);
						if (pose.equals(start))
							found = true;
					}
				}
			}
			return ret;
		}
	}

	@Override
	public Map<Pose, Double> getReachingPoses(Pose goal) {
		Map<Pose, Double> ret = new HashMap<Pose, Double>();
		for (List<MarkerPose> path : markerPaths) {
			if (path.contains(goal)) {
				for (MarkerPose pose : path) {
					if (pose.equals(goal))
						break;
					if (interestingPoses.contains(pose)
							|| getVisibleMarkers().contains(pose))
						ret.put(pose, 1d);
				}
			}
		}
		return ret;
	}

	public List<MarkerPose> getVisibleMarkers() {
		ArrayList<MarkerPose> ret = new ArrayList<MarkerPose>(visibleMarkers);
		if (currentPose != null)
			ret.add(currentPose);
		return ret;
	}

	@Override
	public void addInterestingPose(Pose pose) {
		interestingPoses.add(pose);
	}

	@Override
	public boolean moveTo(Pose goal) {
		System.out.println("Moving to marker "
				+ ((MarkerPose) goal).getMarkerId());

		for (MarkerPose start : getVisibleMarkers()) {
			for (List<MarkerPose> path : markerPaths) {
				if (path.contains(start) && path.contains(goal)
						&& path.indexOf(start) < path.indexOf(goal)) {
					int from = path.indexOf(start);
					int count = path.indexOf(goal) - from;
					byte[] ids = new byte[path.indexOf(goal)
							- path.indexOf(start)];
					for (int i = from + 1; i <= from + count; i++) {
						ids[i - from - 1] = (byte) path.get(i).getMarkerId();
					}
					setCurrentPose((MarkerPose) goal);
					return executor.execute(ids);
				}
			}
		}
		return false;
	}

	public void setCurrentPose(MarkerPose markerPose) {
		if (markerPose != null && markerPose.equals(currentPose))
			System.out.println("I think we are at " + markerPose);
		currentPose = markerPose;
	}

	public void setVisibleMarkers(List<MarkerPose> visibleMarkers) {
		this.visibleMarkers = visibleMarkers;
		System.out.println("Seeing markers "
				+ Arrays.toString(visibleMarkers.toArray()));
	}
}
