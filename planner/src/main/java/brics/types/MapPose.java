package brics.types;

public class MapPose implements Pose {
	private final String reference;
	private final double x, y, theta;

	public MapPose(String reference, double x, double y, double theta) {
		this.reference = reference;
		this.x = x;
		this.y = y;
		this.theta = theta;
	}

	public String getReference() {
		return reference;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getTheta() {
		return theta;
	}

	@Override
	public boolean equals(Object other) {
		if (other instanceof MapPose) {
			MapPose mapPose = (MapPose) other;
			return mapPose.getReference().equals(getReference())
					&& mapPose.getX() == getX() && mapPose.getY() == getY()
					&& mapPose.getTheta() == getTheta();
		}
		return false;
	}
	
	@Override
	public int hashCode() {
		return reference.hashCode();
	}
	
	@Override
	public String toString() {
		return reference + ": " + x + "/" + y;
	}
}
