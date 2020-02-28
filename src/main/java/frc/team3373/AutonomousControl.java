package frc.team3373;

import java.util.ArrayList;

import frc.team3373.autonomous.AutoScript;
import frc.team3373.autonomous.DriveandShoot;

public class AutonomousControl {
	private ArrayList<AutoScript> autoList;
	private AutoScript auto;

	private AutonomousControl instance;

	public AutonomousControl getInstance() {
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
	}

	// Initializes auto
	public void init(int id) {
		auto = autoList.get(id);
		auto.init();
	}

	// Starts the autonomous running
	public void start() {
		auto.start();
	}

	public void stop() {
		auto.stop();
	}
}