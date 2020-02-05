/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

import frc.team3373.util.MathUtil;
import frc.team3373.SwerveWheel;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
   * int FRdriveMotorID = 2; int FRrotateMotorID = 1; int FREncHome = 367; // Zero
   * values (value when wheel is turned to default zero- bolt hole facing //
   * front.) int FREncMin = 11; int FREncMax = 870;
   * 
   * int BLdriveMotorID = 4; int BLrotateMotorID = 3; int BLEncHome = 264; int
   * BLEncMin = 12; int BLEncMax = 902;
   * 
   * int FLdriveMotorID = 8; int FLrotateMotorID = 7; int FLEncHome = 426; int
   * FLEncMin = 11; int FLEncMax = 903;
   * 
   * int BRdriveMotorID = 6; int BRrotateMotorID = 5; int BREncHome = 102; int
   * BREncMin = 11; int BREncMax = 904;
   */

  private SuperJoystick driver;

  SwerveWheel FLWheel;
  SwerveWheel BLWheel;
  SwerveWheel FRWheel;
  SwerveWheel BRWheel;

  SwerveWheel[] wheels;

  SwerveControl swerve;
  SuperAHRS ahrs;

  double target = 0.0;

  int index=0;

  double min = Double.MAX_VALUE;
  double max = Double.MIN_VALUE;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    driver = new SuperJoystick(0);
    ahrs = SuperAHRS.GetInstance();
    
    swerve = SwerveControl.GetInstance();
    swerve.setDriveSpeed(0.25);
    

    /* double rotAngle = Math.toDegrees(Math.atan((Constants.robotWidth / 2) / (Constants.robotLength / 2)));

    FLWheel = new SwerveWheel("FrontLeft", Constants.FLRotateMotorID, Constants.FLDriveMotorID, Constants.FLEncMin,
        Constants.FLEncMax, Constants.FLEncHome, Constants.relativeEncoderRatio, 270 - rotAngle);
    FRWheel = new SwerveWheel("FrontRight", Constants.FRRotateMotorID, Constants.FRDriveMotorID, Constants.FREncMin,
        Constants.FREncMax, Constants.FREncHome, Constants.relativeEncoderRatio, rotAngle + 90);
    BLWheel = new SwerveWheel("BackLeft", Constants.BLRotateMotorID, Constants.BLDriveMotorID, Constants.BLEncMin,
        Constants.BLEncMax, Constants.BLEncHome, Constants.relativeEncoderRatio, rotAngle + 270);
    BRWheel = new SwerveWheel("BackRight", Constants.BRRotateMotorID, Constants.BRDriveMotorID, Constants.BREncMin,
        Constants.BREncMax, Constants.BREncHome, Constants.relativeEncoderRatio, rotAngle);
    
    System.out.println(Constants.ROTATE_FL_PID);
    
    FLWheel.setPIDController(Constants.ROTATE_FL_PID);
    FRWheel.setPIDController(Constants.ROTATE_FR_PID);
    BLWheel.setPIDController(Constants.ROTATE_BL_PID);
    BRWheel.setPIDController(Constants.ROTATE_BR_PID);

    wheels = new SwerveWheel[] { FLWheel, BLWheel, BRWheel, FRWheel}; */



    /* SmartDashboard.putNumber("P Gain", 1);
	  SmartDashboard.putNumber("I Gain", 1e-6);
		SmartDashboard.putNumber("D Gain", 0);
		SmartDashboard.putNumber("I Zone", 0);
		SmartDashboard.putNumber("Feed Forward", 1.56e-4);
		SmartDashboard.putNumber("Max Output", 1);
		SmartDashboard.putNumber("Min Output", -1);
		SmartDashboard.putNumber("Set Rotations", 0);

    SmartDashboard.putNumber("Current target", 0); */
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*  for (SwerveWheel wheel : wheels) {
      SmartDashboard.putNumber(wheel.name + " Position", wheel.getRawRotation());
      double pos = wheel.getRawAnalogRotation();
      SmartDashboard.putNumber(wheel.name + " Analog Position", pos);
      //SmartDashboard.putNumber(wheel.name + " Analog Raw Position", pos / 0.00080566406);
    }
 */ 
    double orientationOffset = Math.toRadians(ahrs.getYaw());
    SmartDashboard.putNumber("orientationDegree", Math.toDegrees(orientationOffset));
    SmartDashboard.putNumber("orientationRadians", orientationOffset);
    SmartDashboard.putBoolean("isCalibrating", ahrs.isCalibrating());
    swerve.showPositions();
    swerve.getRotationalCorrection();

    /* if(driver.isBackPushed()){
      ahrs.reset();
    }
    driver.clearButtons(); */

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  public void teleopInit() {
    swerve.resetOrentation();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    /* try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } */
    if(driver.isAHeld() ){
      swerve.calculateSwerveControl(0, -1, 0);
    }else{
      swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4)*0.75);
    }

    if(driver.isRBHeld()){
      swerve.setDriveSpeed(.75);
    }else if(driver.isLBHeld()){
      swerve.setDriveSpeed(0.15);
    }else{
      swerve.setDriveSpeed(0.45);
    }

    /* if(driver.isStartPushed()){
      swerve.calibrateHome();
    } */

    if(driver.isXPushed()){
      swerve.recalculateWheelPosition();
    }

    if(driver.isBackPushed()){
      swerve.resetOrentation();
    }

    switch (driver.getPOV()){
      case 0:
        swerve.changeFront(SwerveControl.Side.NORTH);
        break;
      case 90:
        swerve.changeFront(SwerveControl.Side.EAST);
        break;
      case 180:
        swerve.changeFront(SwerveControl.Side.SOUTH);
        break;
      case 270:
        swerve.changeFront(SwerveControl.Side.WEST);
        break;
    }
    driver.clearButtons();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if(driver.isRBPushed()){
      target+=Constants.relativeEncoderRatio;
      SmartDashboard.putNumber("Current target", target);
      for(SwerveWheel wheel: wheels){
        wheel.setPosition(target);
      }
    }else if(driver.isLBPushed()){
      target-=Constants.relativeEncoderRatio;
      SmartDashboard.putNumber("Current target", target);
      for(SwerveWheel wheel: wheels){
        wheel.setPosition(target);
      }
    }

    if (driver.isAPushed()){
      for(SwerveWheel wheel: wheels){
        //wheel.rawRotate(0);
        wheel.setTargetAngle(0);
        wheel.goToAngle();
        //wheel.setPosition(0);
        target = 0;
      }
    }

    /* if (driver.isYPushed()){
      for(SwerveWheel wheel: wheels){
        //wheel.rawRotate(0.2);
        SmartDashboard.putNumber("Current target", Constants.relativeEncoderRatio*10);
        wheel.setPosition(Constants.relativeEncoderRatio*10);
      }
    } */


    if(driver.isBHeld()){
      double speed = driver.getRawAxis(1);
      SmartDashboard.putNumber("selected power", speed);
      for(SwerveWheel wheel: wheels){
        wheel.setSpeed(speed);
        wheel.drive();
      }
    }
    if(driver.isXPushed()){
      for(SwerveWheel wheel: wheels){
        wheel.setSpeed(0);
        wheel.drive();
      }
    }
    if(driver.isStartPushed()){
      for(SwerveWheel wheel: wheels){
        wheel.zeroRotation();
        target=0;
      }
    }

    if(driver.isBackPushed()){
      for(SwerveWheel wheel: wheels){
        wheel.calabrateWheel();
        wheel.rawRotate(0);
      }
    }
    if(driver.getPOV()!=-1){
      //index=driver.getPOV();
      switch (driver.getPOV()){
        case 0:
          /* SmartDashboard.putString("calabrating Swerve", wheels[0].name);
          testRange(wheels[0]);
          SmartDashboard.putNumber("Min", min);
          SmartDashboard.putNumber("Max", max); */

          //wheels[0].UIConfigPID();
          for(SwerveWheel wheel: wheels){
            target = Math.PI/2;
            wheel.setTargetAngle(target);
            wheel.goToAngle();
          }
          break;
        case 90:
          /* SmartDashboard.putString("calabrating Swerve", wheels[1].name);
          //wheels[1].UIConfigPID();
          testRange(wheels[1]);
          SmartDashboard.putNumber("Min", min);
          SmartDashboard.putNumber("Max", max); */
          for(SwerveWheel wheel: wheels){
            target = 0;
            wheel.setTargetAngle(target);
            wheel.goToAngle();
          }
          break;
        case 180:
          /* SmartDashboard.putString("calabrating Swerve", wheels[2].name);
          //wheels[2].UIConfigPID();
          testRange(wheels[2]);
          SmartDashboard.putNumber("Min", min);
          SmartDashboard.putNumber("Max", max); */
          for(SwerveWheel wheel: wheels){
            target = (3*Math.PI)/2;
            wheel.setTargetAngle(target);
            wheel.goToAngle();
          }
          break;
        case 270:
          /* SmartDashboard.putString("calabrating Swerve", wheels[3].name);
          //wheels[3].UIConfigPID();
          testRange(wheels[3]);
          SmartDashboard.putNumber("Min", min);
          SmartDashboard.putNumber("Max", max); */
          for(SwerveWheel wheel: wheels){
            target = Math.PI;
            wheel.setTargetAngle(target);
            wheel.goToAngle();
          }
          break;
      }
    } 

    driver.clearButtons();
    
  }

  public void testRange(SwerveWheel wheel){
    double val = wheel.getRawAnalogRotation();
    if(val>max)
      max=val;
    if(val<min)
      min=val; 
  }
  

}
