package brics;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import brics.types.MapPose;
import brics.types.MarkerPose;
import brics.types.Pose;

public class TransitionPlanner implements SubPlanner {

	private Map<MarkerPose, MapPose> markerToMap = new HashMap<MarkerPose, MapPose>();
	private Map<MapPose, MarkerPose> mapToMarker = new HashMap<MapPose, MarkerPose>();
	private TransitionExecutor executor;
	private MarkerPlanner markerPlanner;
	private MapPlanner mapPlanner;
	
	public TransitionPlanner(MarkerPlanner markerPlanner, MapPlanner mapPlanner, TransitionExecutor executor) {
		this.markerPlanner = markerPlanner; 
		this.mapPlanner = mapPlanner;
		this.executor = executor;
	}
	
	public void addMarkerToMap(MarkerPose marker, MapPose map) {
		markerToMap.put(marker, map);
		markerPlanner.addInterestingPose(marker);
		mapPlanner.addInterestingPose(map);
	}

	public void addMapToMarker(MapPose map, MarkerPose marker) {
		mapToMarker.put(map, marker);
		markerPlanner.addInterestingPose(marker);
		mapPlanner.addInterestingPose(map);
	}

	@Override
	public Map<Pose, Double> getReachablePoses(Pose start) {
		Map<Pose, Double> ret = new HashMap<Pose, Double>();
		if (start == null)
			return ret;
		if (start instanceof MapPose && mapToMarker.containsKey(start))
			ret.put(mapToMarker.get(start), 1d);
		if (start instanceof MarkerPose && markerToMap.containsKey(start))
			ret.put(markerToMap.get(start), 1d);
		return ret;
	}

	@Override
	public Map<Pose, Double> getReachingPoses(Pose goal) {
		Map<Pose, Double> ret = new HashMap<Pose, Double>();
		if (markerToMap.containsValue(goal)) {
			for (Entry<MarkerPose, MapPose> entry : markerToMap.entrySet()) {
				if (entry.getValue().equals(goal))
					ret.put(entry.getKey(), 1d);
			}
		}

		if (mapToMarker.containsValue(goal)) {
			for (Entry<MapPose, MarkerPose> entry : mapToMarker.entrySet()) {
				if (entry.getValue().equals(goal))
					ret.put(entry.getKey(), 1d);
			}
		}

		return ret;
	}

	@Override
	public void addInterestingPose(Pose pose) {
	}

	@Override
	public boolean moveTo(Pose goal) {
		if (goal instanceof MarkerPose) {
			executor.switchToMarker((MarkerPose) goal);
			System.out.println("Transition to Marker "
					+ ((MarkerPose) goal).getMarkerId());
			markerPlanner.setCurrentPose((MarkerPose) goal);
			mapPlanner.setCurrentPose(null);
		}
		else if (goal instanceof MapPose) {
			executor.switchToMap((MapPose) goal);
			System.out.println("Transition to map "
					+ ((MapPose) goal).getReference());
			markerPlanner.setCurrentPose(null);
			mapPlanner.setCurrentPose((MapPose) goal);
		}
		return true;
	}

}
