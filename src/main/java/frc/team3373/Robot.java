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

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  // Color 2
  private ColorSensor myCs = new ColorSensor();

  // Joystick
  private SuperJoystick driver = new SuperJoystick(0);

  @Override
  public void robotInit() {

    // Meta
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
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

    // update color sensor
    myCs.update();

    // Show data on shuffleboard  
    SmartDashboard.putNumber("IR", myCs.getIR());
    SmartDashboard.putNumber("Proximity", myCs.getProximity());
    SmartDashboard.putString("Detected Color", myCs.getWheelColor().toString());

    if (driver.isAHeld()) {
      System.out.println(myCs.getR()+" "+myCs.getG()+" "+myCs.getB());
    }
  }
}
