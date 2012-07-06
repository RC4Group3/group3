package brics;

import brics.types.MapPose;
import brics.types.MarkerPose;

public interface TransitionExecutor {
	public boolean switchToMap(MapPose pose);
	public boolean switchToMarker(MarkerPose pose);
}
