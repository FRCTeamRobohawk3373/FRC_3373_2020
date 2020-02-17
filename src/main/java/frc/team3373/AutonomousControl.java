package frc.team3373;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotState;

public class AutonomousControl {
	private ArrayList<AutoScript> autoList;
	private AutoScript auto;

	/**
	 * Initializes self and defines AHRS, swerve, and ultrasonic sensor
	 */
	public AutonomousControl(int autoId) {
		autoList = new ArrayList<AutoScript>();
	}

	// Initializes auto by file name and passes in itself as a parameter
	public void init(int id) {
		auto = autoList.get(id);
		auto.init();
	}

	// Starts the autonomous running
	public void start() {
		if (RobotState.isAutonomous()) {
			auto.update();
		} else {
			auto.stop();
		}
	}
}