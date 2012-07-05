package brics.types;


public class MarkerPose implements Pose {
	private final int markerId;
	
	public MarkerPose(int markerId) {
		this.markerId = markerId;
	}

	public int getMarkerId() {
		return markerId;
	}
	
	@Override
	public boolean equals(Object other) {
		if(other instanceof MarkerPose) {
			return ((MarkerPose) other).getMarkerId() == getMarkerId();
		}
		return false;
	}
	
	@Override
	public int hashCode() {
		return markerId;
	}
	
	@Override
	public String toString() {
		return "Marker " + markerId;
	}
}
