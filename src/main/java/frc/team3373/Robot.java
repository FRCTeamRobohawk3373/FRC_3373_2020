/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.drivers.ADIS16448_IMU;


public class Robot extends TimedRobot {
  private final int HEADING_FRAMES = 10;

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
  
  // IMU
  private final ADIS16448_IMU m_imu = new ADIS16448_IMU();

  // Digital imput
  ManualInput mi;


  public Robot() {
    super(0.02);
  }

  @Override
  public void robotInit()  {

    // Meta
    m_autoChooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_autoChooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_autoChooser);
    m_imu.calibrate();

    mi = new ManualInput();

  }

  public double ezround(double val) {
    return Math.round(val);
  }
  public double ezround(double val, int precision) {
    double exp = Math.pow(10, precision);
    return Math.round(val*exp)/exp;
  }



  @Override
  public void robotPeriodic() {

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


    //for (int i = 0; i < digins.length; i ++) {
    //  SmartDashboard.putBoolean("DIGIN "+i, digins[i].get());
    //}

    SmartDashboard.putNumber("num", mi.getClockNumber());
    SmartDashboard.putBoolean("l", mi.leftSwitchOn());
    SmartDashboard.putBoolean("r", mi.rightSwitchOn());
    SmartDashboard.putBoolean("r-t", mi.rightSwitchToggle());


    
    timeNow = Timer.getFPGATimestamp();
    timeDelta = timeNow - timeWas;
    SmartDashboard.putString("ms / Hz", Math.round(timeDelta*1000)+" / "+Math.round(1/timeDelta));
    timeWas = timeNow;
    


  }
}
