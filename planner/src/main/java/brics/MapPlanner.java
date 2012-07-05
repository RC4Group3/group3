package brics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import brics.types.MapPose;
import java.util.Map;

import brics.types.Pose;

public class MapPlanner implements SubPlanner {

	private List<MapPose> interestingPoses = new ArrayList<MapPose>();
	private MapPose currentPose;

	@Override
	public Map<Pose, Double> getReachablePoses(Pose start) {
		Map<Pose, Double> ret = new HashMap<Pose, Double>();
		if (start == null) {
			if (getCurrentReference() == null)
				return ret;
			ret.put(currentPose, 0d);
		} else if (!(start instanceof MapPose)) {
			return ret;
		}
		MapPose mapStart = (MapPose) start;
		String startReference = ((mapStart == null) ? getCurrentReference()
				: mapStart.getReference());
		for (MapPose pose : interestingPoses) {
			if (pose.getReference().equals(startReference)) {
				ret.put(pose, 1d);
			}
		}
		return ret;
	}

	public String getCurrentReference() {
		if (currentPose == null)
			return null;
		return currentPose.getReference();
	}

	@Override
	public Map<Pose, Double> getReachingPoses(Pose goal) {
		Map<Pose, Double> ret = new HashMap<Pose, Double>();
		if (goal instanceof MapPose) {
			MapPose mapGoal = (MapPose) goal;
			for (MapPose pose : interestingPoses) {
				if (mapGoal.getReference().equals(pose.getReference())) {
					ret.put(pose, 1d);
				}
			}
			if (mapGoal.getReference().equals(getCurrentReference()))
				ret.put(currentPose, 1d);
		}
		return ret;
	}

	@Override
	public void addInterestingPose(Pose pose) {
		if (pose instanceof MapPose) {
			interestingPoses.add((MapPose) pose);
		}
	}

	@Override
	public boolean moveTo(Pose goal) {
		MapPose mapGoal = (MapPose) goal;
		System.out.println("Moving to " + mapGoal.getReference() + ": "
				+ mapGoal.getX() + "/" + mapGoal.getY());
		setCurrentPose((MapPose) goal);
		return true;
	}

	public void setCurrentPose(MapPose mapPose) {
		currentPose = mapPose;
	}

}
