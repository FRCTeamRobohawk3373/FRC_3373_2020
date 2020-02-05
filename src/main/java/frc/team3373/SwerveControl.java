package frc.team3373;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.Constants;
import frc.team3373.util.MathUtil;

public class SwerveControl {

	public static enum Side {
		NORTH, SOUTH, EAST, WEST, UNKNOWN;
	}
	
	public static enum DriveMode {
		ROBOTCENTRIC, FIELDCENTRIC, OBJECTCENTRIC;
	}

	private SwerveWheel FLWheel;
	private SwerveWheel BLWheel;
	private SwerveWheel FRWheel;
	private SwerveWheel BRWheel;

	private SwerveWheel[] wheelArray;

	private double distanceToCenter;

	private double robotLength;
	private double robotWidth;

	private double orientationOffset = 0;
	private double frontDirectionOffset = 0;

	private boolean isFieldCentric = false;
	private boolean isObjectCentric = false;

	private double maxTargetSpeed = 0.5;
	private double targetRobotAngle = 0;

	private SuperAHRS ahrs;

	private static SwerveControl instance;

	public static SwerveControl GetInstance(){
		if(instance == null){
			instance = new SwerveControl();
		}
		return instance;
	}

	/*public SwerveControl(int LFrotateMotorID, int LFdriveMotorID, int LFEncMin, int LFEncMax, int LFEncHome,
			int LBrotateMotorID, int LBdriveMotorID, int LBEncMin, int LBEncMax, int LBEncHome, int RFrotateMotorID,
			int RFdriveMotorID, int RFEncMin, int RFEncMax, int RFEncHome, int RBrotateMotorID, int RBdriveMotorID,
			int RBEncMin, int RBEncMax, int RBEncHome, SuperAHRS AHRS, double width, double length) {*/

	public SwerveControl(){
		robotWidth = Constants.robotWidth;
		robotLength = Constants.robotLength;

		double rotAngle = Math.atan((Constants.robotWidth / 2) / (Constants.robotLength / 2));
		System.out.println("Rotational Offset: "+Math.toDegrees(rotAngle));
		

		ahrs = SuperAHRS.GetInstance();
		ahrs.reset();
		// System.out.println(rotAngle);

		FLWheel = new SwerveWheel("FrontLeft", Constants.FLRotateMotorID, Constants.FLDriveMotorID, Constants.FLEncMin, 
			Constants.FLEncMax, Constants.FLEncHome, Constants.relativeEncoderRatio, rotAngle+Math.PI);
		FRWheel = new SwerveWheel("FrontRight", Constants.FRRotateMotorID, Constants.FRDriveMotorID, Constants.FREncMin, 
			Constants.FREncMax, Constants.FREncHome, Constants.relativeEncoderRatio, rotAngle+(Math.PI/2));
		BLWheel = new SwerveWheel("BackLeft", Constants.BLRotateMotorID, Constants.BLDriveMotorID, Constants.BLEncMin, 
			Constants.BLEncMax, Constants.BLEncHome, Constants.relativeEncoderRatio, rotAngle-(Math.PI/2));
		BRWheel = new SwerveWheel("BackRight", Constants.BRRotateMotorID, Constants.BRDriveMotorID, Constants.BREncMin, 
			Constants.BREncMax, Constants.BREncHome, Constants.relativeEncoderRatio, rotAngle);

		FLWheel.setPIDController(Constants.ROTATE_FL_PID);
		FRWheel.setPIDController(Constants.ROTATE_FR_PID);
		BLWheel.setPIDController(Constants.ROTATE_BL_PID);
		BRWheel.setPIDController(Constants.ROTATE_BR_PID);

		wheelArray = new SwerveWheel[] { FLWheel, BLWheel, BRWheel, FRWheel };

	}
	/**
	 * Autonomus control for the swerve drive
	 * @param driveAngle the angle to drive at (0 to 359). 0 is to the right of the front, 90 is front, 180 is to the left of the front, 270 is back,
	 * @param driveSpeed the speed to drive at the angle (0 to 1). 0 is stop and 1 is full speed.
	 * @param rotateSpeed the speed to rotate around the center (-1 to 1). -1 is counter clockwise and 1 is clockwise.
	 */
	public void calculateAutoSwerveControl(double driveAngle, double driveSpeed, double rotateSpeed) {
		double translationalXComponent=0;
		double translationalYComponent=0;
		double translationalMagnitude;
		double translationalAngle;

		double rAxis=rotateSpeed;
		double rotateXComponent;
		double rotateYComponent;

		driveSpeed = Math.abs(driveSpeed);
		if (driveSpeed > 1) {
			driveSpeed = 1;
		}
		
		if (rotateSpeed > 1) {
			rotateSpeed = 1;
		} else if(rotateSpeed<-1){
			rotateSpeed = -1;
		}
		
		if (driveAngle >= 360) {
			driveAngle=driveAngle % 360;
		} else if (driveAngle < 0) {
			driveAngle=driveAngle % 360;
			driveAngle += 360;
		}
		
		/*if (rotateSpeed == 0) {
			if (stoppedRotating) {
				targetRobotAngle = ahrs.getRotation();
				stoppedRotating = false;
			}
			rAxis = getRotationalCorrection();
			rotateSpeed = rAxis;
		}else{
			stoppedRotating = true;
		}*/
		 

		/* if (isFieldCentric) {
			// if in field centric mode make offset equal to the current angle of the navX
			orientationOffset = ahrs.getRawRotation();
		}*/
		
		double rotationMagnitude = Math.abs(rAxis);

		// We break up the axis to create two vectors for the robot(and each wheel)
		// translational vector
		// rotation vector

		translationalMagnitude = driveSpeed;
		translationalAngle = driveAngle;

		// sets the robot front to be at the ang
		translationalAngle += orientationOffset;
		if (translationalAngle >= 360) {
			translationalAngle -= 360;
		} else if (translationalAngle < 0) {
			translationalAngle += 360;
		}
		// calculates y component of translation vector
		translationalYComponent = Math.sin(translationalAngle) * translationalMagnitude;
		// calculates x component of translation vector
		translationalXComponent = Math.cos(translationalAngle) * translationalMagnitude;

		// calculates y component of translation vector

		if (driveSpeed == 0 && rotateSpeed == 0) {
			stopMoving();
		} else {
			// math for rotation vector, different for every wheel so we calculate for each
			// one seperately
			for (SwerveWheel wheel : wheelArray) {

				// calculates x component of rotation vector
				rotateXComponent = Math.cos(wheel.getRAngle()) * rotationMagnitude;
				// calculates y component of rotation vector
				rotateYComponent = Math.sin(wheel.getRAngle()) * rotationMagnitude;
				if (rAxis > 0) {// inverts the X and Y to change the direction of the wheels when rotating.
					rotateXComponent = -rotateXComponent;
					rotateYComponent = -rotateYComponent;
				}

				// sets the speed based off translational and rotational vectors
				wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
						+ Math.pow((rotateYComponent + translationalYComponent), 2)));

				wheel.setTargetAngle(MathUtil.translatePIRange(Math.atan2((rotateYComponent + translationalYComponent),
						(rotateXComponent + translationalXComponent))));// sets the target angle based off translation
				// and rotational vectors
			}
		}
		// System.out.println("");
		// Makes the wheels go to calculated target angle
		FRWheel.goToAngle();
		FLWheel.goToAngle();
		BRWheel.goToAngle();
		BLWheel.goToAngle();
		// Make the wheels drive at their calculated speed
		FRWheel.drive();
		FLWheel.drive();
		BRWheel.drive();
		BLWheel.drive();
	}

	// ######################################################
	// ########### Teleop ############
	// ######################################################

	/**
	 * Controls the swerve drive based on joystick input
	 * @param LX Left Joystick X axis (-1,1)
	 * @param LY Left Joystick Y axis (-1,1)
	 * @param RX Right Joystick X axis (-1,1)
	 */
	public void calculateSwerveControl(double LX, double LY, double RX) {
		LY = -LY; // inverts joystick LY to match the Cartesian plane
		//LX = -LX; // inverts joystick LY to match the Cartesian plane
		double translationalXComponent = LX * Math.abs(LX);
		double translationalYComponent = LY * Math.abs(LY);
		double translationalMagnitude;
		double translationalAngle;

		double rAxis = RX * Math.abs(RX);
		double rotateXComponent;
		double rotateYComponent;

		// Deadband
		if (Math.abs(translationalXComponent) < 0.002) {
			translationalXComponent = 0;
			LX = 0;
		}

		if (Math.abs(translationalYComponent) < 0.002) {
			translationalYComponent = 0;
			LY = 0;
		}

		if (Math.abs(rAxis) < 0.002) {
			rAxis = 0;
			RX = 0;
		}

		// We break up the axis to create two vectors for the robot(and each wheel)
		// translational vector
		// rotation vector

		// Same for all wheels so therefore we only do the transitional vector math once.
		// magnitude of joystick
		translationalMagnitude = Math.sqrt((translationalYComponent * translationalYComponent) + (translationalXComponent * translationalXComponent));
		// angle of joystick
		translationalAngle = Math.atan2(translationalYComponent, translationalXComponent);
		//translationalAngle =Math.toDegrees(Math.atan2(translationalYComponent, translationalXComponent));

		//Math.floor(LX/(360/8))*(360/8);

		// if in field centric mode make offset equal to the current angle of the navX
		if(RX!=0){
			targetRobotAngle = Math.toRadians(ahrs.getYaw());
		}else{
			rAxis=getRotationalCorrection();
			if(LX != 0 && LY !=0 ){
				rAxis=getRotationalCorrection();
				RX=rAxis;
			}
		} 

		if (isFieldCentric) {
			orientationOffset = Math.toRadians(ahrs.getYaw());
			translationalAngle += orientationOffset;
			if (translationalAngle >= Math.PI) {
				translationalAngle -= 2 * Math.PI;
			} else if (translationalAngle < -Math.PI) {
				translationalAngle += 2 * Math.PI;
			}
		}

		double rotationMagnitude = Math.abs(rAxis);

		// sets the robot front to be at the ang
		translationalAngle += frontDirectionOffset;
		if (translationalAngle >= Math.PI) {
			translationalAngle -= 2*Math.PI;
		} else if (translationalAngle < -Math.PI) {
			translationalAngle += 2*Math.PI;
		}

		// calculates y component of translation vector
		translationalYComponent = Math.sin(translationalAngle) * translationalMagnitude;
		// calculates x component of translation vector
		translationalXComponent = Math.cos(translationalAngle) * translationalMagnitude; 

		if (LY == 0 && LX == 0 && RX == 0) {
			stopMoving();
		} else {
			// math for rotation vector, different for every wheel so we calculate for each
			// one seperately
			double maxSpeed = 0;
			for (SwerveWheel wheel : wheelArray) {

				// calculates x component of rotation vector
				rotateXComponent = Math.cos(wheel.getRAngle()) * rotationMagnitude;
				// calculates y component of rotation vector
				rotateYComponent = Math.sin(wheel.getRAngle()) * rotationMagnitude;
				if (rAxis > 0) {// inverts the X and Y to change the direction of the wheels when rotating.
					rotateXComponent = -rotateXComponent;
					rotateYComponent = -rotateYComponent;
				}
				
				// sets the speed based off translational and rotational vectors
				wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
						+ Math.pow((rotateYComponent + translationalYComponent), 2)) * maxTargetSpeed);

				wheel.setTargetAngle(MathUtil.translatePIRange(Math.atan2((rotateYComponent + translationalYComponent),
						(rotateXComponent + translationalXComponent))));// sets the target angle based off translation
				// and rotational vectors
				if(maxSpeed<wheel.getSpeed()){
					maxSpeed=wheel.getSpeed();
				}
			}

			if(maxSpeed>maxTargetSpeed){
				for (SwerveWheel wheel : wheelArray) {
					wheel.setSpeed((wheel.getSpeed()/maxSpeed)*maxTargetSpeed);
				}
			}
		}
		// System.out.println("");
		// Makes the wheels go to calculated target angle
		FRWheel.goToAngle();
		FLWheel.goToAngle();
		BRWheel.goToAngle();
		BLWheel.goToAngle();
		// Make the wheels drive at their calculated speed
		FRWheel.drive();
		FLWheel.drive();
		BRWheel.drive();
		BLWheel.drive();
	}

	public double getRotationalCorrection() {
		double currentRotation = Math.toRadians(ahrs.getYaw());
		double angleError = targetRobotAngle - currentRotation;
		if(angleError>Math.PI){
			angleError-=2*Math.PI;
		}else if(angleError<-Math.PI){
			angleError+=2*Math.PI;
		}
		//int directionMod = -1;
		//int optimalDirection = 1;
		SmartDashboard.putNumber("Current Target", targetRobotAngle);
		SmartDashboard.putNumber("angleError", angleError);
		double speed = (angleError/Math.PI)*Constants.ROTATIONAL_CORRECTION_FACTOR;

		if (speed > 1)
			speed = 1;
		if (speed < -1)
			speed = -1;

		SmartDashboard.putNumber("Rotational Speed", speed);
		return speed;
	} 

	public void calculateObjectControl(double RX) {
		double distanceToFront = distanceToCenter - robotLength / 2;
		double distanceToBack = distanceToCenter + robotLength / 2;

		FLWheel.setTargetAngle(180 - Math.toDegrees(Math.atan2(robotWidth / 2, distanceToFront)));
		FRWheel.setTargetAngle(180 + Math.toDegrees(Math.atan2(robotWidth / 2, distanceToFront)));
		BLWheel.setTargetAngle(180 - Math.toDegrees(Math.atan2(robotWidth / 2, distanceToBack)));
		BRWheel.setTargetAngle(180 + Math.toDegrees(Math.atan2(robotWidth / 2, distanceToBack)));

		BLWheel.setSpeed(RX);
		BRWheel.setSpeed(RX);

		double speedRatio = Math.sqrt(Math.pow((robotWidth / 2), 2) + Math.pow(distanceToFront, 2))
				/ Math.sqrt(Math.pow((robotWidth / 2), 2) + Math.pow(distanceToBack, 2));

		FLWheel.setSpeed(speedRatio * RX);
		FRWheel.setSpeed(speedRatio * RX);

		FRWheel.goToAngle();
		FLWheel.goToAngle();
		BRWheel.goToAngle();
		BLWheel.goToAngle();

		FRWheel.drive();
		FLWheel.drive();
		BRWheel.drive();
		BLWheel.drive();
	}

	public void setDistanceToObject(double distance) {
		distanceToCenter = distance;
	}

	public void recalculateWheelPosition(){ 
		for (SwerveWheel wheel : wheelArray) {
			wheel.resetPosition();
		}
	}

	/* public double getTravelDistance() {
		double x = 0;
		double y = 0;
		for (SwerveWheel wheel : wheelArray) {
			double[] distance = wheel.getPosition();
			x += distance[0];
			y += distance[1];
		}
		x = x / 4;
		y = y / 4;
		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	} */

	/* public void resetTravelDistance() {
		for (SwerveWheel wheel : wheelArray) {
			wheel.resetPosition();
		}
	} */

	public void setControlMode(DriveMode mode) {
		switch (mode) {
		case ROBOTCENTRIC:
			isFieldCentric = false;
			isObjectCentric = false;
			frontDirectionOffset = 0;
			break;
		case FIELDCENTRIC:
			isFieldCentric = true;
			isObjectCentric = false;
			break;
		case OBJECTCENTRIC:
			isFieldCentric = false;
			isObjectCentric = true;
			break;
		}
	}
	
	public DriveMode getControlMode() {
		if(isFieldCentric){
			return DriveMode.FIELDCENTRIC;
		}else if(isObjectCentric){
			return DriveMode.OBJECTCENTRIC;
		}
		return DriveMode.ROBOTCENTRIC;
	}

	public void changeFront(Side side) {
		// switch out of field centric
		// set the robot front (N,E,S,W)
		switch (side) {
		case NORTH:
			isFieldCentric = false;
			isObjectCentric = false;
			frontDirectionOffset = 0;
			break;
		case EAST:
			isFieldCentric = false;
			isObjectCentric = false;
			frontDirectionOffset = -(Math.PI/2);
			break;
		case SOUTH:
			isFieldCentric = false;
			isObjectCentric = false;
			frontDirectionOffset = Math.PI;
			break;
		case WEST:
			isFieldCentric = false;
			isObjectCentric = false;
			frontDirectionOffset = (Math.PI/2);
			break;
		default:
			isFieldCentric = false;
			isObjectCentric = false;
			frontDirectionOffset = 0;
			break;
		}
	}

	public Side getFront() {
		// switch out of field centric
		// set the robot front (N,E,S,W)
		if(frontDirectionOffset==0) {
			return Side.NORTH;
		}else if(frontDirectionOffset==-(Math.PI/2)) {
			return Side.EAST;
		}else if(frontDirectionOffset==Math.PI) {
			return Side.SOUTH;
		}else if(frontDirectionOffset==(Math.PI/2)) {
			return Side.WEST;
		}else {
			return Side.UNKNOWN;
		}
	}

	public void resetOrentation() {
		ahrs.reset();
		orientationOffset = 0;
		frontDirectionOffset = 0;
		targetRobotAngle = 0;
	}

	public void setDriveSpeed(double speed) {
		speed = Math.abs(speed);
		if (speed > 1)
			speed = 1;
		maxTargetSpeed = speed;
	}

	/* public void printPositions() {
		for (SwerveWheel wheel : wheelArray) {
			//System.out.print(wheel.name + "'s position: " + wheel.getRawEncoderValue() + ", ");
			SmartDashboard.putNumber(wheel.name, wheel.getMotorPosition());
		}
		//System.out.println();
	} */

	/* public void calibrateMinMax() {
		System.out.println("Getting the Mins and Maxs of the Swerve Wheels.");
		for (SwerveWheel wheel : wheelArray) {
			double speed;
			double currentValue = wheel.getMotorPosition();
			double previousValue = currentValue;
			double min = currentValue;
			double max = currentValue;
			byte sameNumberCount = 0;

			for (int i = 0; i < 3; i++) {
				speed = 0.01;
				System.out.println(i);
				wheel.rawRotate(speed);
				while (previousValue <= currentValue) {
					previousValue = currentValue;
					currentValue = wheel.getMotorPosition();
					if (previousValue == currentValue) {
						sameNumberCount++;
						if (sameNumberCount > 20) {
							if(speed<.1)
								speed += 0.001;
								wheel.rawRotate(speed);
							sameNumberCount = 0;
						}
					} else {
						sameNumberCount = 0;
					}
					if (currentValue - previousValue > 4) {
						speed -= 0.01;
						wheel.rawRotate(speed);
					}

					if (currentValue > max)
						max = currentValue;
					if (currentValue < min)
						min = currentValue;

					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				System.out.println(wheel.name + " min: " + min + " max: " + max);
			}
			wheel.rawRotate(0);
		}
	}

	public void calibrateHome() {
		for (SwerveWheel wheel : wheelArray)
			System.out.print(wheel.name + " Home: " + wheel.getMotorPosition()+" ");
		System.out.println();
	} */

	private void stopMoving() {
		for (SwerveWheel wheel : wheelArray) {
			wheel.setSpeed(0);
			wheel.drive();
		}
	}

	public void showPositions(){
		for (SwerveWheel wheel : wheelArray) {
			SmartDashboard.putNumber(wheel.name + " Position", wheel.getRawRotation());
			double pos = wheel.getRawAnalogRotation();
			SmartDashboard.putNumber(wheel.name + " Analog Position", pos);
			//SmartDashboard.putNumber(wheel.name + " Analog Raw Position", pos / 0.00080566406);
		  }
	}

	public void calibrateHome(){
		for(SwerveWheel wheel: wheelArray){
			wheel.calabrateWheel();
		}
	}
}
