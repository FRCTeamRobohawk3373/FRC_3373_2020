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
  ////private ColorSensor myCs = new ColorSensor();

  // Joystick
  private SuperJoystick shooter = new SuperJoystick(0);

  // Timer
  private double timeNow = Timer.getFPGATimestamp();
  private double timeDelta;
  private double timeWas;

  ////private final ADIS16448_IMU m_imu = new ADIS16448_IMU();
  ManualInput mi;
  Climber climber;

  public Robot() {
    super(0.02);// Frame rate = 50Hz
  }

  @Override
  public void robotInit() {

    // Meta
    m_autoChooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_autoChooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_autoChooser);

    mi = new ManualInput();
    climber = new Climber();
  }

  @Override
  public void robotPeriodic() {

    mi.displayOnShuffleboard();

    timeNow = Timer.getFPGATimestamp();
    timeDelta = timeNow - timeWas;
    SmartDashboard.putString("ms / Hz", Math.round(timeDelta * 1000) + " / " + Math.round(1 / timeDelta));
    timeWas = timeNow;

    shooter.clearButtons();
    shooter.clearDPad();
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

    //* Climber
    double lSticky = -shooter.getRawAxis(1);
    if (Math.abs(lSticky) > 0.05) {
      climber.yStickManual(lSticky);
    } else if (shooter.isDPadDownPushed()) {
      climber.gotoLowPosition();
    } else if (shooter.isDPadLeftPushed() || shooter.isDPadRightPushed()) {
      climber.gotoMiddlePosition();
    } else if (shooter.isDPadUpPushed()) {
      climber.gotoHighPosition();
    } else if (shooter.isYPushed()) {
      climber.initiateClimbMode();
    }
    climber.update();
  }

  @Override
  public void testPeriodic() {

    //* Climber
    double yStick1 = -shooter.getRawAxis(1);
    double yStick2 = -shooter.getRawAxis(5);

    if (climber.getCalibrating()) {
      if (shooter.isAPushed()) {
        climber.calibrateHeights();
      } else if (shooter.isBPushed()) {                  
        climber.calibrateInches();
      }
      // Secondary y-stick for calibrating the climber

      if (Math.abs(yStick1) > 0.05) {
        climber.yStick1Calibrate(yStick1);
      } 
      
      if (Math.abs(yStick2) > 0.05) {
        climber.yStick2Calibrate(yStick2);
      }
    } else {
      if (shooter.isDPadDownPushed()) {
        climber.gotoLowPosition();
      } else if (shooter.isDPadLeftPushed() || shooter.isDPadRightPushed()) {
        climber.gotoMiddlePosition();
      } else if (shooter.isDPadUpPushed()) {
        climber.gotoHighPosition();
      } else if (shooter.isYPushed()) {
        climber.initiateClimbMode();
      }

      if (Math.abs(yStick1) > 0.05) {
        climber.yStickManual(yStick1);
      } 
    }

    if (shooter.isStartPushed()) {
      climber.startCalibrateOptions();
    }
    
    climber.update();
    climber.displayOnShuffleboard();
  }

  public void disabledInit() {
    climber.disable();
  }
}
