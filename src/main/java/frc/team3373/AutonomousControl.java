package frc.team3373;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotState;
import frc.team3373.autonomous.DriveandShoot;
import frc.team3373.util.DelayTrueBoolean;
import frc.team3373.util.MathUtil;

public class AutonomousControl {
	private ArrayList<Runnable> autoList;
	private Thread auto;

	private PIDController rotationPID;
	private DelayTrueBoolean timer;
	private int moveTime = 0;
	private boolean notMoving = true;
	private double maxTargetSpeed = 0.45;

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
		autoList = new ArrayList<Runnable>();
		autoList.add(new DriveandShoot());
	}

	// Starts the autonomous running
	public void start(int id) {
		auto = new Thread(autoList.get(id), "Auto " + id);
		auto.start();
	}

	public void update() {
		if (!RobotState.isAutonomous()) {
			auto.interrupt();
		}
	}

	//public boolean finishedMoving() {

	//}

	private void initPIDController() {
		rotationPID.reset();
		rotationPID.enableContinuousInput(-180, 180);
		rotationPID.setTolerance(0.5);
	}
	
	/**
	 * moves the robot in a direction for a certain amount of time
	 * @param angle the angle in degrees to drive at (0,360)
	 * @param speed	the speed the drive at (0,1)
	 * @param time the time in seconds to drive for
	 */
	public void relativeMoveRobot(double angle, double speed, double time) {
		SwerveControl swerve = SwerveControl.getInstance();
		double oldSpeed = swerve.getDriveSpeed();
		swerve.setDriveSpeed(speed);
		swerve.calculateSwerveControl(Math.sin(Math.toRadians(angle)), -Math.cos(Math.toRadians(angle)), 0);
		try{
			Thread.sleep((long) (time * 1000));
		}catch(Exception e){
			//Do nothing
		}
		swerve.calculateSwerveControl(0, 0, 0);
		swerve.setDriveSpeed(oldSpeed);
	}

	/**
	 * Rotates the robot a relative amount
	 * @param angle	The amount to rotate by
	 */
	public void relativeRotateRobot(double angle) {
		SwerveControl swerve = SwerveControl.getInstance();
		SuperAHRS ahrs = SuperAHRS.getInstance();
		//SmartDashboard.putNumber("Delta Angle", angle);
		double currentAngle = ahrs.getYaw();
		SmartDashboard.putNumber("Current Angle:", currentAngle);
		double targetAngle = currentAngle + angle;

		if(targetAngle >= 180){
			targetAngle -= 360;
		} else if(targetAngle < -180){
			targetAngle += 360;
		}
		SmartDashboard.putNumber("Target Angle: ", targetAngle);
		
		initPIDController();
		//rotationPID.setSetpoint(targetAngle);
		double power = rotationPID.calculate(currentAngle,targetAngle);
		//SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
		while(!rotationPID.atSetpoint() && RobotState.isEnabled()){ //waits until we are within range of the angle
			//SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
			//rotationPID.setSetpoint(targetAngle); //tells PID loop to go to the targetAngle
			currentAngle = ahrs.getYaw();
			SmartDashboard.putNumber("Current Angle:", currentAngle);
			SmartDashboard.putNumber("Position Error", rotationPID.getPositionError());
			//SmartDashboard.putNumber("Velocity Error", rotationPID.getVelocityError());
			power = rotationPID.calculate(currentAngle);
			//SmartDashboard.putNumber("PID Power: ", power);
			//calculateSwerveControl(0,0,0.2);
			swerve.calculateSwerveControl(0, 0, MathUtil.clamp(power, -.75, .75)); //sets the wheels to rotate based off PID loop
			try{
				Thread.sleep(1);
			} catch(Exception e){
				//Do nothing
			}
		}
		//SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
		swerve.calculateSwerveControl(0,0,0); //stops robot spinning
		SmartDashboard.putNumber("Current Angle:", currentAngle);
	}

	/**
	 * Used in Autonomous Mode Only, Rotates robot to a certain angle regardless of robots current position
	 * @param targetAngle Angle(degrees) to which the robot will rotate
	 */
	
	public void absoluteRotateRobot(double targetAngle) {
		SwerveControl swerve = SwerveControl.getInstance();
		SuperAHRS ahrs = SuperAHRS.getInstance();
		double currentAngle = ahrs.getYaw();
		if(targetAngle >= 180){
			targetAngle-=360;
		} else if(targetAngle < -180){
			targetAngle +=360;
		}

		SmartDashboard.putNumber("Target Angle: ", targetAngle);
		initPIDController();
		//rotationPID.setSetpoint(targetAngle);
		double power = rotationPID.calculate(currentAngle,targetAngle);
		//SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
		while(!rotationPID.atSetpoint() && RobotState.isEnabled()){ //waits until we are within range of the angle
			//SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
			//rotationPID.setSetpoint(targetAngle); //tells PID loop to go to the targetAngle
			currentAngle = ahrs.getYaw();
			SmartDashboard.putNumber("Current Angle:", currentAngle);
			SmartDashboard.putNumber("Position Error", rotationPID.getPositionError());
			//SmartDashboard.putNumber("Velocity Error", rotationPID.getVelocityError());
			power = rotationPID.calculate(currentAngle);
			//SmartDashboard.putNumber("PID Power: ", power);
			//calculateSwerveControl(0,0,0.2);
			swerve.calculateSwerveControl(0, 0, MathUtil.clamp(power, -.75, .75)); //sets the wheels to rotate based off PID loop
			try{
				Thread.sleep(1);
			} catch(Exception e){
				//Do nothing
			}
		}

		swerve.calculateSwerveControl(0,0,0); //stops robot spinning
		SmartDashboard.putNumber("Current Angle:", currentAngle);
	}
}