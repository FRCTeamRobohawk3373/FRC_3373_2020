/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.SuperJoystick;

public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  // Color sensor
  private ColorSensor myCs = new ColorSensor();

  // Joystick
  private SuperJoystick driver = new SuperJoystick(0);

  // Timer
  private double timeNow = Timer.getFPGATimestamp();
  private double timeDelta;
  private double timeWas;

  // private final ADIS16448_IMU m_imu = new ADIS16448_IMU();
  ManualInput mi;
  Climber climber;

  public Robot() {
    super(0.02);
  }

  @Override
  public void robotInit() {

    // Meta
    m_autoChooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_autoChooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_autoChooser);


    mi = new ManualInput();
    climber = new Climber(1, 4);

  }

  @Override
  public void robotPeriodic() {

    if (RobotState.isOperatorControl()) {

      double lSticky = -driver.getRawAxis(1);
      if (Math.abs(lSticky) > 0.05) {
        climber.onYStick(lSticky);
      } else if (driver.isDPadDownPushed()) {
        climber.gotoLowPosition();
      } else if (driver.isDPadLeftPushed() || driver.isDPadRightPushed()) {
        climber.gotoMiddlePosition();
      } else if (driver.isDPadUpPushed()) {
        climber.gotoHighPosition();
      } else if (driver.isYPushed()) {
        climber.climb();
      }
      climber.updateTeleOp();
    } else if (RobotState.isTest()) {

      double lSticky = -driver.getRawAxis(1);
      if (Math.abs(lSticky) > 0.05) {
        climber.onYStick(lSticky);
      } else if (driver.isDPadDownPushed() && !climber.isCalibrating()) {
        climber.gotoLowPosition();
      } else if ((driver.isDPadLeftPushed() || driver.isDPadRightPushed()) && !climber.isCalibrating()) {
        climber.gotoMiddlePosition();
      } else if (driver.isDPadUpPushed() && !climber.isCalibrating()) {
        climber.gotoHighPosition();
      } else if (driver.isYPushed() && !climber.isCalibrating()) {
        climber.climb();
      } else if (driver.isStartPushed()) {
        climber.onCalibrateButton();
      } else if (driver.isAPushed()) {
        climber.onAButton();
      } else if (driver.isBPushed()) {
        climber.onBButton();
      }
      climber.updateTestMode();
    }
    

    mi.displayOnShuffleboard();

    
    timeNow = Timer.getFPGATimestamp();
    timeDelta = timeNow - timeWas;
    SmartDashboard.putString("ms / Hz", Math.round(timeDelta * 1000) + " / " + Math.round(1 / timeDelta));
    timeWas = timeNow;


    driver.clearButtons();
    driver.clearDPad();

  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_autoChooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

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

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testPeriodic() {
  }
}
