package frc.team3373;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.PIDConstant;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

//import frc.team3373.Constants;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWheel {
	private static final double TWOPI = 2.0*Math.PI;

	private CANSparkMax rotateMotor; // The motor which spins the assembly
	private CANSparkMax driveMotor; // The motor powering the wheel

	private CANPIDController m_pidController;

	private CANEncoder rotateEncoder;
	private CANEncoder driveEncoder;

	public String name; // used for helpful debug
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

		rotateEncoder = rotateMotor.getEncoder(EncoderType.kHallSensor,42);
		driveEncoder = driveMotor.getEncoder();

		m_pidController = rotateMotor.getPIDController();
		m_pidController.setReference(0, ControlType.kPosition);
		m_pidController.setFeedbackDevice(rotateEncoder);

		//positionOffset = 0;

		////position = new double[2];

		kMaxOutput=1;
		kMinOutput=-1;
		rotateMotor.setInverted(false);
		rotateMotor.getAnalog(AnalogMode.kAbsolute).setInverted(true);
		//? investivate setPositionConversionFactor() and getPositionConversionFactor()
		rotateMotor.setIdleMode(IdleMode.kBrake);// Activates brake mode

		System.out.println(DebugName+"'s calculated ABS position: " + Math.toDegrees(getCurrentAbsAngle())+
			" Degrees, calculated Rel position: "+getCurrentAbsAngle()*relRadfactor);
		
		rotateEncoder.setPosition(getCurrentAbsAngle()*relRadfactor);


		driveMotor.setIdleMode(IdleMode.kCoast);
		driveMotor.setClosedLoopRampRate(2);
		driveMotor.setOpenLoopRampRate(1.5);

		System.out.println(DebugName + "'s absEncoder conversion factor: " + absRadfactor);
	}

	public void setPID(double P, double I, double D) {
		setPIDF(P, I, D, 0);
	}

	public void setPIDF(double P, double I, double D, double FF) {
		kP = P;
		kI = I;
		kD = D;
		kFF=FF;
		m_pidController.setP(P);
		m_pidController.setI(I);
		m_pidController.setD(D);
		m_pidController.setFF(FF);
	}

	public void setPIDController(PIDConstant pidValues){
		pidValues.configPID(m_pidController);
	}

	/**
	 * Sets the targeted angle to spin the wheel to
	 * @param angle the angle of the wheel in radians [0, 2PI]
	 */
	public void setTargetAngle(double angle) {
		//angle = angle % TWOPI;
		targetAngle = angle;
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
		double pos = rotateMotor.getAnalog(AnalogMode.kAbsolute).getPosition()-EHome;
		if (pos < 0) {
			pos += (EMax-EMin);
		}
		return pos / absRadfactor;
	} 

	public void resetPosition(){
		rotateMotor.set(0);
		double newPos=getCurrentAbsAngle()*relRadfactor;
		rotateEncoder.setPosition(newPos);
		m_pidController.setReference(newPos, ControlType.kPosition);
	}

	/**
	 * gets the rotaional offset of the wheel
	 */
	public double getRAngle() {
		return angleOffset;
	}

	public void goToAngle() {
		double current = getCurrentAngle();
		double localAngle = current % TWOPI;
		double deltaTarget = targetAngle - localAngle;
		double altDeltaTarget = (targetAngle+TWOPI)-localAngle;
		double target;

		if(Math.abs(deltaTarget)<Math.abs(altDeltaTarget)){
			target = deltaTarget + current;
		}else{
			target = altDeltaTarget + current;
		}
		
		SmartDashboard.putNumber(name+" new Target",target*relRadfactor);
		m_pidController.setReference(target*relRadfactor, ControlType.kPosition);
		
		// System.out.println(target);
		// SmartDashboard.putNumber(name + "'s target angle'", target);
		// rotateMotor.set(ControlMode.Position,target);
	}

	public void drive() {
		driveMotor.set(targetSpeed);
	}

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

	public boolean calabrateWheel(){
		rotateMotor.setIdleMode(IdleMode.kCoast);
		m_pidController.setFeedbackDevice(rotateMotor.getAnalog(AnalogMode.kAbsolute));
		m_pidController.setReference(EHome, ControlType.kPosition);
		//System.out.println(getRawAnalogRotation());
		return true;
	}


	public double getRawRotation() {
		return rotateEncoder.getPosition();
	}

	public double getRawAnalogRotation() {
		return rotateMotor.getAnalog(AnalogMode.kAbsolute).getVoltage();
	}

	public void rawRotate(double speed) {
		rotateMotor.set(speed);
	}

	public void setPosition(double pos) {
		m_pidController.setReference(pos, ControlType.kPosition);
	}

	public void zeroRotation(){
		rotateMotor.getEncoder().setPosition(0);
		m_pidController.setReference(0, ControlType.kPosition);
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

	public String toString(){
		return "Swerve drive, " + name + ", is at " + Math.toDegrees(getCurrentAngle())+
			" degrees with "+(targetSpeed*100)+"% power.";
	}
}
