package frc.team3373;

import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.PIDConstant;

//import frc.team3373.Constants;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWheel {
	private static final double TWOPI = 2.0*Math.PI;

	private CANSparkMax rotateMotor; // The motor which spins the assembly
	private CANSparkMax driveMotor; // The motor powering the wheel

	private CANPIDController m_pidController;

	private CANEncoder rotateEncoder;
	private CANEncoder driveEncoder;

	private String name; // used for helpful debug
	private double EMin;
	private double EMax;
	private double EHome;
	private double angleOffset = 0;
	private double absRadfactor = 1;
	private double relRadfactor = 1;


	private double targetAngle = 0;
	private double targetSpeed = 0;

	private double drivePosition;
	private double driveVelocity;

	////private double stepPerDegree;
	////private double positionOffset;

	////private double[] position;

	public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

	/**
	 * @param DebugName Name of the Swerve Modual for easy debug
	 * @param rotateMotorID	CanID of the rotation motor
	 * @param driveMotorID CanID of the drive motor
	 * @param absEncMin The minimum value of the absolute position encoder
	 * @param absEncMax The maximum value of the absolute position encoder
	 * @param absEncHome The value of the absolute position encoder when the wheel is in the home position
	 * @param relEncRatio the ratio of swerve rotation to motor rotations
	 * @param rotationOffset the angle the wheel should be in when rotating in place
	 */
	public SwerveWheel(String DebugName, int rotateMotorID, int driveMotorID, 
			double absEncMin, double absEncMax, double absEncHome, double relEncRatio, double rotationOffset) {

		EMin = absEncMin;
		EMax = absEncMax;
		EHome = absEncHome;
		name = DebugName;
		angleOffset = rotationOffset;
		absRadfactor = (EMax - EMin) / (TWOPI);
		relRadfactor = relEncRatio / (TWOPI);

		rotateMotor = new CANSparkMax(rotateMotorID, MotorType.kBrushless);
		driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

		rotateEncoder = rotateMotor.getEncoder();
		driveEncoder = driveMotor.getEncoder();

		m_pidController = rotateMotor.getPIDController();
		m_pidController.setReference(0, ControlType.kPosition);
		m_pidController.setFeedbackDevice(rotateEncoder);

		//positionOffset = 0;
		//position = new double[2];

		rotateMotor.setInverted(false);
		rotateMotor.getAnalog(AnalogMode.kAbsolute).setInverted(true);
		rotateMotor.setIdleMode(IdleMode.kBrake);// Activates brake mode

		/*System.out.println(DebugName+"'s calculated ABS position: " + Math.toDegrees(getCurrentAbsAngle())+
			" Degrees, calculated Rel position: "+getCurrentAbsAngle()*relRadfactor); */
		
		resetPosition();
		//rotateEncoder.setPosition(getCurrentAbsAngle()*relRadfactor);

		driveMotor.setIdleMode(IdleMode.kCoast);
		driveMotor.setClosedLoopRampRate(2);
		driveMotor.setOpenLoopRampRate(1.5);

		//System.out.println(DebugName + "'s absEncoder conversion factor: " + absRadfactor);
		System.out.println(DebugName + "'s current ABS Position: " + Math.toDegrees(getCurrentAbsAngle()));
	}

	public void setPID(double P, double I, double D) {
		setPIDF(P, I, D, 0);
	}

	public void setPIDF(double P, double I, double D, double FF) {
		m_pidController.setP(P);
		m_pidController.setI(I);
		m_pidController.setD(D);
		m_pidController.setFF(FF);
	}

	public void setPIDController(PIDConstant pidValues){
		pidValues.configPID(m_pidController);
	}
	
	/** 
	 * Updates the Relative encoder's position the the Absolute position
	 */
	public void resetPosition(){
		rotateMotor.set(0);
		Runnable delayedAction = new Runnable(){ //! this is a Test! Remove if wheels have home position issues
			public void run() {
				double newPos=getCurrentAbsAngle()*relRadfactor;
				rotateEncoder.setPosition(newPos);
				m_pidController.setReference(newPos, ControlType.kPosition);
			}
		};
		Notifier delay = new Notifier(delayedAction);
		delay.startSingle(0.1);
		//delay.close();	
		
		/*double newPos=getCurrentAbsAngle()*relRadfactor;
		rotateEncoder.setPosition(newPos);
		m_pidController.setReference(newPos, ControlType.kPosition); */
	}

	/**
	 * returns the name
	 */
	public String getName() {
		return name;
	}

	//################################
	//##	Rotation measurments	##
	//################################

	/**
	 * gets the current angle of the wheel in radians;
	 */
	public double getCurrentAngle() {
		double rad = (rotateEncoder.getPosition()/relRadfactor);
		//rad = rad%TWOPI;
		return rad;
	}

	/**
	 * gets the current angle of the abosolute postion encoder of the wheel in radians;
	 */
	public double getCurrentAbsAngle(){
		double avg = 0;
		for(int i = 0; i<10; i++){
			avg+=rotateMotor.getAnalog(AnalogMode.kAbsolute).getVoltage();
		}
		avg /= 10;

		double pos = avg-EHome;
		if (pos < 0) {
			pos += (EMax-EMin);
		}
		return pos / absRadfactor;
	} 

	//################################
	//##	Distance measurments	##
	//################################

	/* private void updatePosition() {
		double encValue = driveMotor.getEncoder().getPosition() - positionOffset;
		position[0] = encValue * Math.cos(targetAngle); // x
		position[1] = encValue * Math.sin(targetAngle); // y

		positionOffset = encValue;
	}

	public double[] getPosition() {
		updatePosition();
		return position;
	}

	public void resetPosition() {
		positionOffset = driveMotor.getEncoder().getPosition();
		position[0] = 0;
		position[1] = 0;
		updatePosition();
	} */


	//################################
	//##   Swerve Control Methods	##
	//################################

	/**
	 * Sets the targeted angle to spin the wheel to
	 * @param angle the angle of the wheel in radians [0, 2PI]
	 */
	public void setTargetAngle(double angle) {
		//angle = angle % TWOPI;
		targetAngle = angle;
	}

	/**
	 * Gets the current target angle
	 */
	public double getTargetAngle(){
		return targetAngle;
	}

	/**
	 * Sets the targeted percent speed of the motor
	 * @param speed speed [0, 1]
	 */
	public void setSpeed(double speed) {
		SmartDashboard.putNumber(name+" Speed", speed);
		if (speed > 1)
			speed = 1;
		if (speed < -1)
			speed = -1;

		targetSpeed = speed;
	}

	/**
	 * gets the current target speed.
	 */
	public double getSpeed() {
		return targetSpeed;
	}

	/**
	 * gets the rotaional offset of the wheel
	 */
	public double getOffsetAngle() {
		return angleOffset;
	}

	public void goToAngle() {
		double current = getCurrentAngle();
		double localAngle = current % TWOPI;
		double deltaTarget = targetAngle - localAngle;
		double altDeltaTarget;
		if(deltaTarget>0){
			altDeltaTarget= (deltaTarget-TWOPI);
		}else{
			altDeltaTarget= (deltaTarget+TWOPI);
		}
		//double altPosDeltaTarget = (targetAngle+TWOPI)-localAngle;
		double target;

		if(Math.abs(altDeltaTarget)<Math.abs(deltaTarget)){
			target = altDeltaTarget + current;
		}else{
			target = deltaTarget + current;
		}
		//target = deltaTarget + current;
		SmartDashboard.putNumber(name+" targetAngle",targetAngle);
		SmartDashboard.putNumber(name+" new Target",target*relRadfactor);
		m_pidController.setReference(target*relRadfactor, ControlType.kPosition);
		
		// System.out.println(target);
		// SmartDashboard.putNumber(name + "'s target angle'", target);
		// rotateMotor.set(ControlMode.Position,target);
	}

	public void drive() {
		driveMotor.set(targetSpeed);
	}

	//################################
	//##	Calibration Methods 	##
	//################################

	public void calibBegin(){
		rotateMotor.setIdleMode(IdleMode.kCoast);
		rotateMotor.set(0);
	}

	public void calibSetHome(){
		EHome = rawGetAnalogRotation();
	}

	public void calibFindMinMax(){
		EMin = Double.POSITIVE_INFINITY;
		EMax = Double.NEGATIVE_INFINITY;

		double current = rawGetAnalogRotation();
		double previous = current;
		int dir = 1;

		for(double s=0.1; s>0.02;s-=0.01){
			rotateMotor.set(s*dir);
			while(Math.abs(current-previous)< 0.5){
				SmartDashboard.putNumber("Min", EMin);
				SmartDashboard.putNumber("Max", EMax);

				
				if (current > EMax)
					EMax = current;
				if (current < EMin)
					EMin = current;

				previous = current;
				current = rawGetAnalogRotation();
			}
			previous = current;
			dir*=-1;
		}

		for(int i =0; i<10;i++){
			rotateMotor.set(0.015*dir);
			while(Math.abs(current-previous)< 0.5){
				SmartDashboard.putNumber("Min", EMin);
				SmartDashboard.putNumber("Max", EMax);
				if (current > EMax)
					EMax = current;
				if (current < EMin)
					EMin = current;

				previous = current;
				current = rawGetAnalogRotation();
			}
			previous = current;
			dir*=-1;
		}
		rotateMotor.set(0);
		System.out.println(name+"'s Min: "+EMin+", Max: "+EMax);
	}

	public void calibZeroRotation(){
		rotateMotor.getEncoder().setPosition(0);
		m_pidController.setReference(0, ControlType.kPosition);
	}

	public void calibEnd(){
		rotateMotor.setIdleMode(IdleMode.kBrake);
		m_pidController.setReference(EHome, ControlType.kPosition);
		absRadfactor = (EMax - EMin) / (TWOPI);

	}

	public boolean calabrateWheel(){
		rotateMotor.setIdleMode(IdleMode.kCoast);
		
		//System.out.println(getRawAnalogRotation());
		return true;
	}

	public void UIConfigPID(){
		double p = SmartDashboard.getNumber("P Gain", 0);
		double i = SmartDashboard.getNumber("I Gain", 0);
		double d = SmartDashboard.getNumber("D Gain", 0);
		double iz = SmartDashboard.getNumber("I Zone", 0);
		double ff = SmartDashboard.getNumber("Feed Forward", 0);
		double max = SmartDashboard.getNumber("Max Output", 0);
		double min = SmartDashboard.getNumber("Min Output", 0);
		double rotations = SmartDashboard.getNumber("Set Rotations", 0);

		// if PID coefficients on SmartDashboard have changed, write new values to controller
		if((p != kP)) { m_pidController.setP(p); kP = p; }
		if((i != kI)) { m_pidController.setI(i); kI = i; }
		if((d != kD)) { m_pidController.setD(d); kD = d; }
		if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
		if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
		if((max != kMaxOutput) || (min != kMinOutput)) { 
			m_pidController.setOutputRange(min, max); 
			kMinOutput = min; kMaxOutput = max; 
		}
	}


	//################################
	//##	Raw Control Methods 	##
	//################################

	public double rawGetRotation() {
		return rotateEncoder.getPosition();
	}

	public double rawGetAnalogRotation() {
		return rotateMotor.getAnalog(AnalogMode.kAbsolute).getVoltage();
	}

	public void rawRotate(double speed) {
		rotateMotor.set(speed);
	}

	public void rawSetPosition(double pos) {
		m_pidController.setReference(pos, ControlType.kPosition);
	}

	public String toString(){
		return "Swerve drive, " + name + ", is at " + Math.toDegrees(getCurrentAngle())+
			" degrees with "+(targetSpeed*100)+"% power.";
	}
}
