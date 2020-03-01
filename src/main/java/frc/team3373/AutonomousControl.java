package frc.team3373;

import java.util.ArrayList;

import frc.team3373.autonomous.AutoScript;
import frc.team3373.autonomous.DriveandShoot;

public class AutonomousControl {
	private ArrayList<AutoScript> autoList;
	private AutoScript auto;
	private boolean hasRun = false;

	private static AutonomousControl instance;

	public static AutonomousControl getInstance() {
		if (instance == null) {
			instance = new AutonomousControl();
		}
		return instance;
	}

	/**
	 * Initializes self and defines AHRS, swerve, and ultrasonic sensor
	 */
	public AutonomousControl() {
		autoList = new ArrayList<AutoScript>();
		autoList.add(new DriveandShoot());
		hasRun=false;
	}

	// Initializes auto
	public void init(int id) {
		auto = autoList.get(id);
		auto.init();
	}

	// Starts the autonomous running
	public void start() {
		if(!hasRun){
			auto.start();
			hasRun=true;
		}
	}

	public void stop() {
		auto.stop();
	}
}