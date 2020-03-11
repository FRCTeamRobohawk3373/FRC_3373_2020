package frc.team3373;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
	private NetworkTableInstance inst;
	private NetworkTable visionTable;
	private NetworkTable streamingTable;
	private NetworkTable processingTable;

	//private boolean loaded = false;
	//private boolean lock = false;
	//private boolean refresh = false;

	private static Vision instance;

	private int activeCamNum = -1;
	private String activeCamName = "";

	//private Map<Integer, String> cammap;
	//private ArrayList<VisionObject> objects = new ArrayList<VisionObject>();

	private VisionData vData;
	private CameraData cData;

	class VisionData{
		private ArrayList<VisionObject> targetObjects = new ArrayList<VisionObject>();
	
		//private VisionObject[] targetObjects;
	
		private boolean lock = false;
	
		public synchronized ArrayList<VisionObject> getData(){
			if(lock){
				try {
					wait();
				 } catch (InterruptedException e) {
					e.printStackTrace();
				 }
			}
			lock=true;
			ArrayList<VisionObject> data = (ArrayList<VisionObject>)targetObjects.clone();
			lock=false;
			notify();
			return data;
		}
	
		public synchronized int getSize(){
			if(lock){
				try {
					wait();
				 } catch (InterruptedException e) {
					e.printStackTrace();
				 }
			}
			lock=true;
			int size = targetObjects.size();
			lock=false;
			notify();
			return size;
		}
	
		public synchronized void updateVisionData(EntryNotification event){
			if(lock){
				try {
					wait();
				 } catch (InterruptedException e) {
					e.printStackTrace();
				 }
			}
			lock=true;
			String[] data = event.getEntry().getStringArray(null);
	
			if (data != null || data.length != 0) {
				targetObjects.clear();
				String[] sa;
				for (String str : data) {
					str.replace("[", "");
					str.replace("]", "");
					str.replace(" ", "");
					sa = str.split(",");
					if (Arrays.binarySearch(sa, "None") < 0)
						targetObjects.add(new VisionObject(Double.parseDouble(sa[1]), Double.parseDouble(sa[2]),
								Double.parseDouble(sa[3]), Double.parseDouble(sa[4]), Double.parseDouble(sa[5])));
				}
			}
			//update code
			lock=false;
			notify();
		}
	}
	
	class CameraData{
		private boolean lock = false;
		private Map<Integer, String> cammap = new HashMap<Integer, String>();
	
		public synchronized Map<Integer, String> getCameraMap(){
			if(lock){
				try {
					wait();
				 } catch (InterruptedException e) {
					e.printStackTrace();
				 }
			}
			lock=true;
			Map<Integer, String> data = Map.copyOf(cammap);
			lock=false;
			notify();
			return data;
		}
	
		public synchronized void updateCamMap(EntryNotification event){
			updateCamMap(event.getEntry());
		}

		public synchronized void updateCamMap(NetworkTableEntry entry){
			System.out.println("camMap updating");
			if(lock){
				try {
					wait();
				 } catch (InterruptedException e) {
					e.printStackTrace();
				 }
			}
			lock=true;
			String[] c = entry.getStringArray(null);
			System.out.println("got: "+c);
			if (c != null) {
				String[] sa;
				for (String str : c) {
					str.replace("[", "");
					str.replace("]", "");
					sa = str.split(",");
					cammap.put(Integer.parseInt(sa[0]), sa[1].replace(" ", ""));
				}
			}
			System.out.println(cammap);
			lock=false;
			notify();
		}

	}

	public static Vision getInstance(){
		if(instance == null){
			instance = new Vision();
		}
		return instance;
	}

	public Vision() {
		cData=new CameraData();
		vData=new VisionData();

		inst = NetworkTableInstance.getDefault();
		visionTable = inst.getTable("vision");
		processingTable = visionTable.getSubTable("processing");
		streamingTable = visionTable.getSubTable("switching");
		streamingTable.getEntry("CamMap").addListener(event -> cData.updateCamMap(event),//{System.out.println("New CamMap Data: "+event.name); cData.updateCamMap(event); }, 
				EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
		//streamingTable.getEntry("CamMap").addListener(event -> cData.updateCamMap(event), 
		//		EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		processingTable.getEntry("targetInfo").addListener(event -> vData.updateVisionData(event),
				EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		cData.updateCamMap(streamingTable.getEntry("CamMap"));
		// update();
	}

	public int size() {
		return vData.getSize();
	}

	public void switchStreamCam(int streamIndex) {
		if (cData.getCameraMap().containsKey(streamIndex)) {
			if(activeCamNum!=streamIndex){
				streamingTable.getEntry("selectedCamIndex").setNumber(streamIndex);
				activeCamNum = streamIndex;
				activeCamName = cData.getCameraMap().get(streamIndex);
			}
		} else {
			//System.out.println("ERROR: Invalid camera stream index");
		}
	}

	public void switchStreamCam(String streamName) {
		if (cData.getCameraMap().containsValue(streamName)) {
			if(!streamingTable.getEntry("selectedCamName").getString("").equals(streamName))
				streamingTable.getEntry("selectedCamName").setString(streamName);
		} else {
			//System.out.println("ERROR: Invalid camera stream name");
		}
	}

	/** Gets any object that is the closest*/
	public VisionObject getClosestObject() {
		int closest = -1;
		double cdist = Double.POSITIVE_INFINITY;
		ArrayList<VisionObject> objects = vData.getData();
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

	/**
	 * gets the firt vision target, Null if there are no targets
	 */
	public VisionObject getFirstObject() {
		ArrayList<VisionObject> objects = vData.getData();
		if(objects.size()>0)
			return objects.get(0);
		return null;
	}

	/** Gets objects within a range of distance in inches
	 * @param min the minimum distance
	 * @param max the maximum distance
	*/
	public ArrayList<VisionObject> getObjectsInRange(double min, double max) {
		ArrayList<VisionObject> objects = vData.getData();
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

	public ArrayList<VisionObject> getObjectsInRotation(double min, double max) {
		ArrayList<VisionObject> objects = vData.getData();
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

	/**Gets object with x absolute value closest to zero (center of screen)*/
	public VisionObject getObjectClosestToCenter() {
		int closest = -1;
		double cx = 2.0;
		ArrayList<VisionObject> objects = vData.getData();
		// ArrayList<VisionObject> objs = new ArrayList<VisionObject>();
		// objs.addAll(objects);
		for (int i = 0; i < objects.size(); i++) {
			VisionObject object = objects.get(i);
			if (Math.abs(object.x) < cx) {
				closest = i;
				cx = Math.abs(object.x);
			}
		}
		if (closest == -1) {
			return null;
		}
		return objects.get(closest);
	}

	/* public void setRefresh(EntryNotification event) {
		if (loaded) {
			refresh = true;
		}
	}

	public void update() {
		if (refresh && !lock) {
			refresh();
		}
	} */

	/* private void loadCammap(EntryNotification event) {
		String[] c = streamingTable.getEntry("CamMap").getStringArray(null);
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
	} */

	/* private void refresh() {
		try {
			if (!lock) {
				String[] data = processingTable.getEntry("targetInfo").getStringArray(null);
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
	} */
}
