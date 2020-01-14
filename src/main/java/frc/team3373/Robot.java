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

  int FRdriveMotorID = 2;
  int FRrotateMotorID = 1;
	int FREncHome = 367; // Zero values (value when wheel is turned to default zero- bolt hole facing front.)
	int FREncMin = 11;
	int FREncMax = 870;
	
	int BLdriveMotorID = 4;
	int BLrotateMotorID = 3;
	int BLEncHome = 264;
	int BLEncMin = 12;
	int BLEncMax = 902;
	
	int FLdriveMotorID = 8;
	int FLrotateMotorID = 7;
	int FLEncHome = 426;
	int FLEncMin = 11;
	int FLEncMax = 903;
	
	int BRdriveMotorID = 6;
	int BRrotateMotorID = 5;
	int BREncHome = 102;
	int BREncMin = 11;
  int BREncMax = 904;

  double robotWidth = 22.25; // TODO change robot dimensions to match this years robot
  double robotLength = 16.25;

  private SuperJoystick driver;

  SwerveControl swerve;
  SuperAHRS ahrs;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    driver = new SuperJoystick(0);
    ahrs = new SuperAHRS(SPI.Port.kMXP);
    swerve = new SwerveControl(FLrotateMotorID, FLdriveMotorID, FLEncMin, FLEncMax, FLEncHome, BLrotateMotorID,
        BLdriveMotorID, BLEncMin, BLEncMax, BLEncHome, FRrotateMotorID, FRdriveMotorID, FREncMin, FREncMax, FREncHome,
        BRrotateMotorID, BRdriveMotorID, BREncMin, BREncMax, BREncHome, ahrs, robotWidth, robotLength);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
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

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
