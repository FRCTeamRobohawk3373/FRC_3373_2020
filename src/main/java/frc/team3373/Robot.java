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
//import edu.wpi.first.wpilibj.SPI;

//import frc.team3373.util.MathUtil;
import frc.team3373.SwerveWheel;
import frc.team3373.SwerveControl.DriveMode;
import frc.team3373.SwerveControl.Side;

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
  private static final String kDriveAuto = "Test drive";
  private static final String kRelRotAuto = "Test RelRotate";
  private static final String kAbsRotAuto = "Test ABSRotate";
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

  int index = 0;

  double min = Double.MAX_VALUE;
  double max = Double.MIN_VALUE;

  int mode = 0;
  double endtime = System.currentTimeMillis();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("Test drive", kDriveAuto);
    m_chooser.addOption("Test RelRotate", kRelRotAuto);
    m_chooser.addOption("Test ABSRotate", kAbsRotAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    driver = new SuperJoystick(0);
    ahrs = SuperAHRS.getInstance();

    swerve = SwerveControl.getInstance();
    swerve.setDriveSpeed(0.25);
    swerve.changeControllerLimiter(0);

    /*
     * double rotAngle = Math.toDegrees(Math.atan((Constants.robotWidth / 2) /
     * (Constants.robotLength / 2)));
     * 
     * FLWheel = new SwerveWheel("FrontLeft", Constants.FLRotateMotorID,
     * Constants.FLDriveMotorID, Constants.FLEncMin, Constants.FLEncMax,
     * Constants.FLEncHome, Constants.relativeEncoderRatio, 270 - rotAngle); FRWheel
     * = new SwerveWheel("FrontRight", Constants.FRRotateMotorID,
     * Constants.FRDriveMotorID, Constants.FREncMin, Constants.FREncMax,
     * Constants.FREncHome, Constants.relativeEncoderRatio, rotAngle + 90); BLWheel
     * = new SwerveWheel("BackLeft", Constants.BLRotateMotorID,
     * Constants.BLDriveMotorID, Constants.BLEncMin, Constants.BLEncMax,
     * Constants.BLEncHome, Constants.relativeEncoderRatio, rotAngle + 270); BRWheel
     * = new SwerveWheel("BackRight", Constants.BRRotateMotorID,
     * Constants.BRDriveMotorID, Constants.BREncMin, Constants.BREncMax,
     * Constants.BREncHome, Constants.relativeEncoderRatio, rotAngle);
     * 
     * System.out.println(Constants.ROTATE_FL_PID);
     * 
     * FLWheel.setPIDController(Constants.ROTATE_FL_PID);
     * FRWheel.setPIDController(Constants.ROTATE_FR_PID);
     * BLWheel.setPIDController(Constants.ROTATE_BL_PID);
     * BRWheel.setPIDController(Constants.ROTATE_BR_PID);
     * 
     * wheels = new SwerveWheel[] { FLWheel, BLWheel, BRWheel, FRWheel};
     */

    /*
     * SmartDashboard.putNumber("P Gain", 1); SmartDashboard.putNumber("I Gain",
     * 1e-6); SmartDashboard.putNumber("D Gain", 0);
     * SmartDashboard.putNumber("I Zone", 0);
     * SmartDashboard.putNumber("Feed Forward", 1.56e-4);
     * SmartDashboard.putNumber("Max Output", 1);
     * SmartDashboard.putNumber("Min Output", -1);
     * SmartDashboard.putNumber("Set Rotations", 0);
     * 
     * SmartDashboard.putNumber("Current target", 0);
     */
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
    /*
     * for (SwerveWheel wheel : wheels) { SmartDashboard.putNumber(wheel.name +
     * " Position", wheel.getRawRotation()); double pos =
     * wheel.getRawAnalogRotation(); SmartDashboard.putNumber(wheel.name +
     * " Analog Position", pos); //SmartDashboard.putNumber(wheel.name +
     * " Analog Raw Position", pos / 0.00080566406); }
     */
    double orientationOffset = Math.toRadians(ahrs.getYaw());
    SmartDashboard.putNumber("orientationDegree", Math.toDegrees(orientationOffset));
    SmartDashboard.putNumber("orientationRadians", orientationOffset);
    SmartDashboard.putBoolean("isNavxCalibrating", ahrs.isCalibrating());
    swerve.showPositions();
    // swerve.getRotationalCorrection();

    /*
     * if(driver.isBackPushed()){ ahrs.reset(); } driver.clearButtons();
     */

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
    m_autoSelected = m_chooser.getSelected();
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDriveAuto:
      swerve.relativeMoveRobot(0, .15, 1);
      break;
    case kRelRotAuto:
      swerve.setDriveSpeed(0.45);
      swerve.relativeRotateRobot(90);
      break;
    case kAbsRotAuto:
      swerve.absoluteRotateRobot(270);
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }

    try {
      Thread.sleep(5000);
    } catch (InterruptedException e) {
      e.printStackTrace();
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
      e.printStackTrace();
    } */
    if(driver.isAHeld() ){
      if(driver.isAPushed()){
        endtime=System.currentTimeMillis()+4000;
        mode=1;
      }
      swerve.calculateSwerveControl(0, -1, 0);

      if(System.currentTimeMillis()>endtime){
        endtime=System.currentTimeMillis()+4000;
        mode++;
      }

      switch(mode) {
        case 1:
          swerve.changeFront(Side.NORTH);
          break;
        case 2:
          swerve.changeFront(Side.WEST);
          break;
        case 3:
          swerve.changeFront(Side.SOUTH);
          break;
        case 4:
          swerve.changeFront(Side.EAST);
          break;
        default:
          mode=1;
          break;
        
      }
    }else{
      swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4)*0.75);
    }
    
    if(driver.isBPushed()){
      swerve.changeControllerLimiter();
    }

    if(driver.isLBHeld()){
      swerve.setDriveSpeed(0.15);
    }else if(driver.isRBHeld()){
        swerve.setDriveSpeed(.75);
    }else{
      swerve.setDriveSpeed(0.45);
    }

    /* if(driver.isStartPushed()){
      swerve.calibrateHome();
    } */

    if(driver.isYPushed()){
      swerve.resetOrentation();
    }

    if(driver.getRawAxis(2)>0.8)
      swerve.setControlMode(DriveMode.FIELDCENTRIC);
    else if(driver.getRawAxis(3)>0.8)
      swerve.setControlMode(DriveMode.ROBOTCENTRIC);

    
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

    if(driver.isBackPushed()){
      swerve.recalculateWheelPosition();
    }

    driver.clearButtons();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if(driver.isStartPushed()){
      swerve.calibrateHome();
    }
    if(driver.isBackPushed()){
      swerve.calibrateMinMax();
    }
    driver.clearButtons();
  }
}
