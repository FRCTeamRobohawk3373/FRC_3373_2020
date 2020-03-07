package frc.team3373;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
	private NetworkTableInstance inst;
	private NetworkTable vTable;
	private NetworkTable streaming;
	private NetworkTable processing;

	private boolean loaded = false;
	private boolean lock = false;
	private boolean refresh = false;

	private Map<Integer, String> cammap;
	private ArrayList<VisionObject> objects = new ArrayList<VisionObject>();

	public Vision() {
		inst = NetworkTableInstance.getDefault();
		vTable = inst.getTable("vision");
		processing = vTable.getSubTable("processing");
		streaming.getEntry("CamMap").addListener((event) -> loadCammap(event), EntryListenerFlags.kNew);
		processing.getEntry("targetInfo").addListener((event) -> setRefresh(event),
				EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		// update();
	}

	public int size() {
		return objects.size();
	}

	public void switchStreamCam(int streamIndex) {
		if (cammap.containsKey(streamIndex)) {
			streaming.getEntry("selectedCamIndex").setNumber(streamIndex);
		} else {
			System.out.println("ERROR: Invalid camera stream index");
		}
	}

	public void switchStreamCam(String streamName) {
		if (cammap.containsValue(streamName)) {
			streaming.getEntry("selectedCamName").setString(streamName);
		} else {
			System.out.println("ERROR: Invalid camera stream name");
		}
	}

	public VisionObject getClosestObject() {
		int closest = -1;
		double cdist = 100000.0;
		for (int i = 0; i < objects.size(); i++) {
			VisionObject object = objects.get(i);
			if (object.distance < cdist) {
				closest = i;
				cdist = object.distance;
			}
		}
		if (closest == -1) {
			return null;
		}
		return objects.get(closest);
	}

	public ArrayList<VisionObject> getObjectsInRange(double max, double min) {
		ArrayList<VisionObject> inRange = new ArrayList<VisionObject>();
		for (int i = 0; i < objects.size(); i++) {
			VisionObject object = objects.get(i);
			if (object.distance < max && object.distance > min) {
				inRange.add(object);
			}
		}
		if (inRange.size() == 0) {
			return null;
		}
		return inRange;
	}

	public ArrayList<VisionObject> getObjectsInRotation(double max, double min) {
		ArrayList<VisionObject> inRange = new ArrayList<VisionObject>();
		for (int i = 0; i < objects.size(); i++) {
			VisionObject object = objects.get(i);
			if (object.rAngle < max && object.rAngle > min) {
				inRange.add(object);
			}
		}
		if (inRange.size() == 0) {
			return null;
		}
		return inRange;
	}

	public VisionObject getObjectClosestToCenter() {
		int closest = -1;
		double cx = 2.0;
		// ArrayList<VisionObject> objs = new ArrayList<VisionObject>();
		// objs.addAll(objects);
		lock = true;
		for (int i = 0; i < objects.size(); i++) {
			VisionObject object = objects.get(i);
			if (Math.abs(object.x) < cx) {
				closest = i;
				cx = Math.abs(object.x);
			}
		}
		lock = false;
		if (closest == -1) {
			return null;
		}
		try {
			return objects.get(closest);
		} catch (IndexOutOfBoundsException e) {
			return null;
		}
	}

	public void setRefresh(EntryNotification event) {
		if (loaded) {
			refresh = true;
		}
	}

	public void update() {
		if (refresh && !lock) {
			refresh();
		}
	}

	private void loadCammap(EntryNotification event) {
		String[] c = streaming.getEntry("CamMap").getStringArray(null);
		if (c != null) {
			String[] sa;
			for (String str : c) {
				str.replace("[", "");
				str.replace("]", "");
				sa = str.split(",");
				cammap.put(Integer.parseInt(sa[0]), sa[1].replace(" ", ""));
			}
		}
		loaded = true;
	}

	private void refresh() {
		try {
			if (!lock) {
				String[] data = processing.getEntry("targetInfo").getStringArray(null);
				if (data == null || data.length == 0) {
					return;
				}
				objects.clear();
				String[] sa;
				for (String str : data) {
					str.replace("[", "");
					str.replace("]", "");
					str.replace(" ", "");
					data = str.split(",");
					if (Arrays.binarySearch(data, "None") < 0)
						objects.add(new VisionObject(Integer.parseInt(data[1]), Integer.parseInt(data[2]),
								Integer.parseInt(data[3]), Integer.parseInt(data[4]), Integer.parseInt(data[5])));
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
