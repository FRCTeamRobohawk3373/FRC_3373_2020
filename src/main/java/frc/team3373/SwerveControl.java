package frc.team3373.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

	// private double rotateTarget;
	private double distanceToCenter;

	double robotLength;
	double robotWidth;

	private float orientationOffset = 0;

	private boolean isFieldCentric = true;
	private boolean isObjectCentric = false;

	private double maxTargetSpeed = 0;
	// private boolean stoppedRotating = true;
	private float targetRobotAngle = 0;

	private SuperAHRS ahrs;

	public SwerveControl(int LFrotateMotorID, int LFdriveMotorID, int LFEncMin, int LFEncMax, int LFEncHome,
			int LBrotateMotorID, int LBdriveMotorID, int LBEncMin, int LBEncMax, int LBEncHome, int RFrotateMotorID,
			int RFdriveMotorID, int RFEncMin, int RFEncMax, int RFEncHome, int RBrotateMotorID, int RBdriveMotorID,
			int RBEncMin, int RBEncMax, int RBEncHome, SuperAHRS AHRS, double width, double length) {

		double rotAngle = Math.toDegrees(Math.atan((width / 2) / (length / 2)));

		robotWidth = width;
		robotLength = length;

		ahrs = AHRS;
		// System.out.println(rotAngle);
		/*
		 * FLWheel=new
		 * SwerveWheel("FrontLeft",LFrotateMotorID,LFdriveMotorID,LFEncMin,LFEncMax,
		 * LFEncHome,rotAngle+270); BLWheel=new
		 * SwerveWheel("BackLeft",LBrotateMotorID,LBdriveMotorID,LBEncMin,LBEncMax,
		 * LBEncHome,rotAngle); FRWheel=new
		 * SwerveWheel("FrontRight",RFrotateMotorID,RFdriveMotorID,RFEncMin,RFEncMax,
		 * RFEncHome,270-rotAngle); BRWheel=new
		 * SwerveWheel("BackRight",RBrotateMotorID,RBdriveMotorID,RBEncMin,RBEncMax,
		 * RBEncHome,rotAngle+90);
		 */

		FLWheel = new SwerveWheel("FrontLeft", LFrotateMotorID, LFdriveMotorID, LFEncMin, LFEncMax, LFEncHome,
				270 - rotAngle);
		BLWheel = new SwerveWheel("BackLeft", LBrotateMotorID, LBdriveMotorID, LBEncMin, LBEncMax, LBEncHome,
				rotAngle + 270);
		FRWheel = new SwerveWheel("FrontRight", RFrotateMotorID, RFdriveMotorID, RFEncMin, RFEncMax, RFEncHome,
				rotAngle + 90);
		BRWheel = new SwerveWheel("BackRight", RBrotateMotorID, RBdriveMotorID, RBEncMin, RBEncMax, RBEncHome,
				rotAngle);

		FLWheel.setPID(Constants.getNumber("FLP"), Constants.getNumber("FLI"), Constants.getNumber("FLD"));
		BLWheel.setPID(Constants.getNumber("BLP"), Constants.getNumber("BLI"), Constants.getNumber("BLD"));
		FRWheel.setPID(Constants.getNumber("FRP"), Constants.getNumber("FRI"), Constants.getNumber("FRD"));
		BRWheel.setPID(Constants.getNumber("BRP"), Constants.getNumber("BRI"), Constants.getNumber("BRD"));

		wheelArray = new SwerveWheel[] { FLWheel, BLWheel, BRWheel, FRWheel };

	}
	/**
	 * Autonomus control for the swerve drive
	 * @param driveAngle the angle to drive at (0 to 359). 0 is to the right of the front, 90 is front, 180 is to the left of the front, 270 is back,
	 * @param driveSpeed the speed to drive at the angle (0 to 1). 0 is stop and 1 is full speed.
	 * @param rotateSpeed the speed to rotate around the center (-1 to 1). -1 is counter clockwise and 1 is clockwise.
	 */
	public void calculateAutoSwerveControl(double driveAngle, double driveSpeed, double rotateSpeed) {// Driving Angle,
																										// Driving
																										// Speed, rotate
																										// speed
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
		 

		if (isFieldCentric) {
			// if in field centric mode make offset equal to the current angle of the navX
			orientationOffset = ahrs.getRawRotation();
		}

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
		translationalYComponent = Math.sin(Math.toRadians(translationalAngle)) * translationalMagnitude;
		// calculates x component of translation vector
		translationalXComponent = Math.cos(Math.toRadians(translationalAngle)) * translationalMagnitude;

		// calculates y component of translation vector

		if (driveSpeed == 0 && rotateSpeed == 0) {
			stopMoving();
		} else {
			// math for rotation vector, different for every wheel so we calculate for each
			// one seperately
			for (SwerveWheel wheel : wheelArray) {

				// calculates x component of rotation vector
				rotateXComponent = Math.cos(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
				// calculates y component of rotation vector
				rotateYComponent = Math.sin(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
				if (rAxis > 0) {// inverts the X and Y to change the direction of the wheels when rotating.
					rotateXComponent = -rotateXComponent;
					rotateYComponent = -rotateYComponent;
				}

				// sets the speed based off translational and rotational vectors
				wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
						+ Math.pow((rotateYComponent + translationalYComponent), 2)));

				wheel.setTargetAngle(Math.toDegrees(Math.atan2((rotateYComponent + translationalYComponent),
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
	public void calculateSwerveControl(double LX, double LY, double RX) {
		LY = -LY; // inverts joystick LY to match the Cartesian plane
		double translationalXComponent = LX * Math.abs(LX);
		double translationalYComponent = LY * Math.abs(LY);
		double translationalMagnitude;
		double translationalAngle;

		double rAxis = RX * Math.abs(RX);
		double rotateXComponent;
		double rotateYComponent;

		// Deadband
		if (Math.abs(translationalXComponent) < 0.0015) {
			translationalXComponent = 0;
			LX = 0;
		}

		if (Math.abs(translationalYComponent) < 0.0015) {
			translationalYComponent = 0;
			LY = 0;
		}

		if (Math.abs(rAxis) < 0.0015) {
			rAxis = 0;
			RX = 0;
		}

		// We break up the axis to create two vectors for the robot(and each wheel)
		// translational vector
		// rotation vector

		// Same for all wheels so therefore we only do the transitional vector math
		//
		//
		// once.
		// magnitude of joystick
		translationalMagnitude = Math.sqrt(Math.pow(translationalYComponent, 2) + Math.pow(translationalXComponent, 2));
		// angle of joystick
		translationalAngle = Math.round(Math.toDegrees(Math.atan2(translationalYComponent, translationalXComponent)) / (360 / 8)) * (360 / 8);
		//translationalAngle =Math.toDegrees(Math.atan2(translationalYComponent, translationalXComponent));

		//Math.floor(LX/(360/8))*(360/8);
		/*if (RX == 0) {
			if (stoppedRotating) {
				targetRobotAngle = ahrs.getRotation();
				stoppedRotating = false;
			}
			rAxis = getRotationalCorrection();
			RX = rAxis;
		} else {
			stoppedRotating = true;
		}*/
		
		 

		if (isFieldCentric) {
			// if in field centric mode make offset equal to the current angle of the navX
			orientationOffset = ahrs.getRawRotation();
			SmartDashboard.putNumber("orientationOffset", orientationOffset);
		}

		double rotationMagnitude = Math.abs(rAxis);

		// sets the robot front to be at the ang
		translationalAngle += orientationOffset;
		if (translationalAngle >= 360) {
			translationalAngle -= 360;
		} else if (translationalAngle < 0) {
			translationalAngle += 360;
		}
		// calculates y component of translation vector
		translationalYComponent = Math.sin(Math.toRadians(translationalAngle)) * translationalMagnitude;
		// calculates x component of translation vector
		translationalXComponent = Math.cos(Math.toRadians(translationalAngle)) * translationalMagnitude; 
		// calculates y component of translation vector

		if (LY == 0 && LX == 0 && RX == 0) {
			stopMoving();
		} else {
			// math for rotation vector, different for every wheel so we calculate for each
			// one seperately
			for (SwerveWheel wheel : wheelArray) {

				// calculates x component of rotation vector
				rotateXComponent = Math.cos(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
				// calculates y component of rotation vector
				rotateYComponent = Math.sin(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
				if (rAxis > 0) {// inverts the X and Y to change the direction of the wheels when rotating.
					rotateXComponent = -rotateXComponent;
					rotateYComponent = -rotateYComponent;
				}

				// sets the speed based off translational and rotational vectors
				wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
						+ Math.pow((rotateYComponent + translationalYComponent), 2)) * maxTargetSpeed);

				wheel.setTargetAngle(Math.toDegrees(Math.atan2((rotateYComponent + translationalYComponent),
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

	private double getRotationalCorrection() {
		float currentRotation = ahrs.getRotation();
		float angleError = targetRobotAngle - currentRotation;
		int directionMod = -1;
		int optimalDirection = 1;

		if (angleError < 0)
			directionMod = 1;

		if (Math.abs(angleError) > 180) {
			SmartDashboard.putBoolean("optimalPath", true);
			angleError = (360 - Math.abs(angleError)) % 360;
			optimalDirection = -1;
			// angleError*=-1;
		} else {
			SmartDashboard.putBoolean("optimalPath", false);
		}

		SmartDashboard.putNumber("Angle Error", angleError);
		double speed = (Math.sqrt(Math.abs(angleError)) + 1) * .04 * directionMod * optimalDirection;
		// (Math.abs(angleError)*0.01111111)*directionMod*optimalDirection;
		if (speed > .7)
			speed = .7;
		if (speed < -.7)
			speed = -.7;

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

	public double getTravelDistance() {
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
	}

	public void resetTravelDistance() {
		for (SwerveWheel wheel : wheelArray) {
			wheel.resetPosition();
		}
	}

	public void setControlMode(DriveMode mode) {
		switch (mode) {
		case ROBOTCENTRIC:
			isFieldCentric = false;
			isObjectCentric = false;
			orientationOffset = 0;
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
			orientationOffset = 0;
			break;
		case EAST:
			isFieldCentric = false;
			isObjectCentric = false;
			orientationOffset = -90;
			break;
		case SOUTH:
			isFieldCentric = false;
			isObjectCentric = false;
			orientationOffset = 180;
			break;
		case WEST:
			isFieldCentric = false;
			isObjectCentric = false;
			orientationOffset = 90;
			break;
		}
	}

	public Side getFront() {
		// switch out of field centric
		// set the robot front (N,E,S,W)
		if(orientationOffset==0) {
			return Side.NORTH;
		}else if(orientationOffset==-90) {
			return Side.EAST;
		}else if(orientationOffset==180) {
			return Side.SOUTH;
		}else if(orientationOffset==90) {
			return Side.WEST;
		}else {
			return Side.UNKNOWN;
		}
	}

	public void resetOrentation() {
		orientationOffset = 0;
		ahrs.reset();
		targetRobotAngle = 0;
	}

	/* public void setRotationTarget(double distance) {
		rotateTarget = distance;
	}

	public void clearRotationTarget() {
		rotateTarget = 0;
	} */

	public void setDriveSpeed(double speed) {
		speed = Math.abs(speed);
		if (speed > 1)
			speed = 1;
		maxTargetSpeed = speed;
	}

	public void printPositions() {
		for (SwerveWheel wheel : wheelArray) {
			//System.out.print(wheel.name + "'s position: " + wheel.getRawEncoderValue() + ", ");
			SmartDashboard.putNumber(wheel.name, wheel.getRawEncoderValue());
		}
		//System.out.println();
	}

	public void calibrateMinMax() {
		System.out.println("Getting the Mins and Maxs of the Swerve Wheels.");
		for (SwerveWheel wheel : wheelArray) {
			double speed;
			int currentValue = wheel.getRawEncoderValue();
			int previousValue = currentValue;
			int min = currentValue;
			int max = currentValue;
			byte sameNumberCount = 0;

			for (int i = 0; i < 3; i++) {
				speed = 0.01;
				System.out.println(i);
				wheel.rawRotate(speed);
				while (previousValue <= currentValue) {
					previousValue = currentValue;
					currentValue = wheel.getRawEncoderValue();
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
			System.out.print(wheel.name + " Home: " + wheel.getRawEncoderValue()+" ");
		System.out.println();
	}

	private void stopMoving() {
		for (SwerveWheel wheel : wheelArray) {
			wheel.setSpeed(0);
			wheel.drive();
		}
	}
}
