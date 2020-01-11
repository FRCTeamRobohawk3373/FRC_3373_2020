package frc.team3373;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWheel {
	WPI_TalonSRX rotateMotor; // The motor which spins the assembly
	CANSparkMax driveMotor; // The motor powering the wheel
	
	private double targetAngle=0;
	private double targetSpeed=0;
	private double rotAngle;
	public String name; //used for helpful debug
	private int EMin;
	private int EMax;
	private int EHome;
	private boolean reverseDir=false;
	
	private double stepPerDegree;
	private double positionOffset;

	private double[] position;
	
	public SwerveWheel(String Name,int rotateMotorID, int driveMotorID,int EncMin,int EncMax,int EncHome,double rotationAngle){
		rotateMotor = new WPI_TalonSRX(rotateMotorID);
		driveMotor = new CANSparkMax(driveMotorID,MotorType.kBrushless);		
		EMin = EncMin;
		EMax=EncMax;
		EHome=EncHome;
		rotAngle=rotationAngle;
		stepPerDegree= (EMax-EMin)/360.0;
		positionOffset = 0;

		position = new double[2];
		
		name = Name;
		
		System.out.println(Name + "'s Steps per Degree: " + stepPerDegree);
		rotateMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
		rotateMotor.setSensorPhase(false);
		rotateMotor.setSelectedSensorPosition(rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		rotateMotor.overrideLimitSwitchesEnable(false);
		rotateMotor.setInverted(false);
		rotateMotor.configFeedbackNotContinuous(true, 0);
		
		rotateMotor.setNeutralMode(NeutralMode.Brake); // Activates brake mode
		rotateMotor.configAllowableClosedloopError(0, 1);
		rotateMotor.selectProfileSlot(0, 0);

		
		driveMotor.setIdleMode(IdleMode.kCoast);
		driveMotor.setClosedLoopRampRate(2);
		driveMotor.setOpenLoopRampRate(1.5);
	}
	
	public void setTargetAngle(double angle){
		targetAngle=angle;
	}

	private void updatePosition() {
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
	}
	
	public void setSpeed(double speed){
		if(speed>1) speed=1;
		if(speed<-1) speed=-1;
		
		targetSpeed=speed;
	}
	
	public double getCurrentAngle() {
		double deg = (rotateMotor.getSensorCollection().getAnalogInRaw() - EHome);
		if (deg < 0) {
			deg += EMax;
			deg -= EMin;
		}
		return deg / stepPerDegree;
	}
	
	public void setPID(double P,double I,double D) {
		rotateMotor.config_kP(0, P);
		rotateMotor.config_kI(0, I);
		rotateMotor.config_kD(0, D);
	}
	
	public double getRAngle(){
		return rotAngle;
	}
	
	public int getRawEncoderValue(){
		return rotateMotor.getSensorCollection().getAnalogInRaw();
	}
	
	public void goToAngle(){
		double current = getCurrentAngle();
		
		if(targetAngle>360){
			targetAngle-=360;
		}else if(targetAngle<0){
			targetAngle+=360;
		}
		
		double opposite = 180+targetAngle;
		
		if(opposite>360){
		 	opposite-=360;
		}else if(opposite<0){
			 opposite+=360;
		}
		
		if(Math.abs(targetAngle-current)>Math.abs(opposite-current)){
			reverseDir=true;
			targetAngle=opposite;
		}else{
			reverseDir=false;
		}
		
		double target = targetAngle*stepPerDegree+EHome;
		
		if(target>EMax){
			target-=EMax;
			target+=EMin;
		}
		//System.out.println(target);
		//SmartDashboard.putNumber(name + "'s target angle'", target);
		rotateMotor.set(ControlMode.Position,target);
	}
	
	public void drive(){
		if(reverseDir){
			driveMotor.set(-targetSpeed);
		}else{
			driveMotor.set(targetSpeed);
		}
	}
	
	public void rawRotate(double speed){
		rotateMotor.set(speed);
	}
}
