package brics;

import java.util.Map;

import brics.types.Pose;

public interface SubPlanner {
	public Map<Pose,Double> getReachablePoses(Pose start);
	public Map<Pose,Double> getReachingPoses(Pose goal);
	public void addInterestingPose(Pose pose);
	
	public boolean moveTo(Pose goal);
}
