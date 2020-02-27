package frc.team3373;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Vision {
    private NetworkTableInstance inst;
    private NetworkTable vTable;
    private NetworkTableEntry vEntry;
    
	private NetworkTableEntry CamChoice1;
	private NetworkTableEntry CamChoice2;
	private NetworkTableEntry CamPreload;

    private boolean lock;

    private Map<String, Integer> cammap;
    private ArrayList<VisionObject> objects = new ArrayList<VisionObject>();

    public Vision() {
        inst = NetworkTableInstance.getDefault();
        vTable = inst.getTable("VisionData");
        vEntry = vTable.getEntry("Objects");
        vEntry.addListener((event) -> dataRefresh(event), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        update();

        CamChoice1 = vTable.getEntry("Camera1");
        CamChoice2 = vTable.getEntry("Camera2");
        CamPreload = vTable.getEntry("PreLoad");

        cammap.put("front", 0);
        cammap.put("left", 1);
        cammap.put("right", 2);
        cammap.put("back", 3);

    }

    public int size() {
        return objects.size();
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
			if (object.rotation < max && object.rotation > min) {
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
		//ArrayList<VisionObject> objs = new ArrayList<VisionObject>();
		//objs.addAll(objects);
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
		try{
			return objects.get(closest);
		} catch (IndexOutOfBoundsException e) {
			return null;
		}
    }
    


    public void update() {
		try {
			String[] objectData = vEntry.getStringArray(new String[0]);
			if (objectData.length == 0) {
				return;
			}
			while (lock) {
				Thread.sleep(0, 100); // ? Doesn't this stop all code?
			}

			objects.clear();
			for (int i = 0; i < objectData.length; i++) {
				objectData[i] = objectData[i].replace("[", "");
				objectData[i] = objectData[i].replace("]", "");
				String[] data = objectData[i].split(", ");
				if (Arrays.binarySearch(data, "None") < 0) {
					objects.add(new VisionObject(Double.parseDouble(data[0]), Double.parseDouble(data[1]),
							                     Double.parseDouble(data[4]), Double.parseDouble(data[5])));
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

    private void dataRefresh(EntryNotification event) {
        update();
    }

}