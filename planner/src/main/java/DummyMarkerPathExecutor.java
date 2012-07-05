import java.util.Arrays;

import brics.MarkerPathExecutor;

public class DummyMarkerPathExecutor implements MarkerPathExecutor {

	@Override
	public boolean execute(byte[] markerPath) {
		System.out.println("Following " + Arrays.toString(markerPath));
		return true;
	}
}
