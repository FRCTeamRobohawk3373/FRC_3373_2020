package frc.team3373;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;

public class AutonomousControl {
	private ArrayList<AutoScript> auto;

	/**
	 * Initializes self and defines AHRS, swerve, and ultrasonic sensor
	 */
	public AutonomousControl() {

	}

	// Initializes auto by file name and passes in itself as a parameter
	/* public boolean start(String au) {
		String aut = "frc.team3373.autonomous." + au;
		try {
			auto = Class.forName(aut);
		} catch (ClassNotFoundException | IllegalAccessException | InstantiationException | InvocationTargetException
				| NoSuchMethodException ex) {
			System.out.println(ex);
			ex.printStackTrace();
			return false;
		}

		return true;
	} */

	// Starts the autonomous running
	public void run() {
		try {
			Method run = auto.getClass().getMethod("run");
			run.invoke(auto);
		} catch (NoSuchMethodException | SecurityException | IllegalArgumentException | InvocationTargetException
				| IllegalAccessException ex) {
			System.out.println(ex);
		}
	}
}