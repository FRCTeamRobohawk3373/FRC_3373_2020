/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373;

import frc.team3373.drivers.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  


  private final ADIS16448_IMU m_imu = new ADIS16448_IMU();


  public Robot() {
    super(0.01);
  }

  @Override
  public void robotInit() {


    // Meta
    m_autoChooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_autoChooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_autoChooser);


    ///

    m_autoChooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_autoChooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_autoChooser);

    SmartDashboard.putBoolean("Green if calibrating", true);
    m_imu.calibrate();
    SmartDashboard.putBoolean("Green if calibrating", false);

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
    //SmartDashboard.putNumber("Angle X", ezround(m_imu.getAngleX()));
    //SmartDashboard.putNumber("Angle Y", ezround(m_imu.getAngleY()));
    SmartDashboard.putNumber("Angle Z", ezround(m_imu.getAngleZ(), 1));
    //SmartDashboard.putNumber("Rate X",  ezround(m_imu.getRateX(), 4));
    //SmartDashboard.putNumber("Rate Y",  ezround(m_imu.getRateY(), 4));
    //SmartDashboard.putNumber("Rate Z",  ezround(m_imu.getRateZ(), 4));
    SmartDashboard.putNumber("Accel X", ezround(m_imu.getAccelX(), 5));
    SmartDashboard.putNumber("Accel Y", ezround(m_imu.getAccelY(), 5));
    //SmartDashboard.putNumber("Accel Z", ezround(m_imu.getAccelZ(), 5));
    //SmartDashboard.putNumber("Mag X",   ezround(m_imu.getMagX(), 1));
    //SmartDashboard.putNumber("Mag Y",   ezround(m_imu.getMagY(), 1));
    //SmartDashboard.putNumber("Mag Z",   ezround(m_imu.getMagZ(), 1));
    //SmartDashboard.putNumber("Temperature", ezround(m_imu.getTemperature()));

    

  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_autoChooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
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

    // Show data on shuffleboard  
    SmartDashboard.putNumber("IR", myCs.getIR());
    SmartDashboard.putNumber("Proximity", myCs.getProximity());
    SmartDashboard.putString("Detected Color", myCs.getWheelColor().toString());

    SmartDashboard.putBoolean("Is Not Green", myCs.getWheelColor().toString()!="GREEN");

    if (driver.isAHeld()) {
      m_imu.reset();
    } else {

    }
    
    timeNow = Timer.getFPGATimestamp();
    timeDelta = timeNow - timeWas;
    SmartDashboard.putString("ms / Hz", Math.round(timeDelta*1000)+" / "+Math.round(1/timeDelta));
    timeWas = timeNow;
    


  }
}
