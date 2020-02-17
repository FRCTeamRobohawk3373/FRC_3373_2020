package frc.team3373;

import java.io.IOException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.Indexer.Motors;

class Testbench extends TimedRobot {
    private SuperJoystick driver;
    private Indexer indexer;
    double INTAKE_MOTOR_SPEED = 0.2; // TODO put in Constants.java
    double CONVEYOR_MOTOR_SPEED = 0.4;

    @Override
    public void robotInit() {
        try {
            Config.loadConfig();
        } catch (IOException e) {
            try {
                Config.loadDefaults();
            } catch (Exception f) {
                System.out.println("Fatal config error");
                f.printStackTrace();
            }
        }
        driver = new SuperJoystick(0);
        indexer = new Indexer(4, 0, 2, 3);
    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
        calibControls();
        indexer.update();
        SmartDashboard.putNumber("Preload Pos", indexer.getRelPos(Motors.PRELOAD));
        SmartDashboard.putNumber("Preload Abs", indexer.getAbsPos(Motors.PRELOAD));
        SmartDashboard.putNumber("Preload Deg", indexer.getRelDeg(Motors.PRELOAD));
        SmartDashboard.putNumber("Load Pos", indexer.getRelPos(Motors.LOAD));
        SmartDashboard.putNumber("Load Abs", indexer.getAbsPos(Motors.LOAD));
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        joystickControls();
        indexer.update();
        SmartDashboard.putNumber("Preload Pos", indexer.getRelPos(Motors.PRELOAD));
        SmartDashboard.putNumber("Preload Abs", indexer.getAbsPos(Motors.PRELOAD));
        SmartDashboard.putNumber("Preload Deg", indexer.getRelDeg(Motors.PRELOAD));
        SmartDashboard.putNumber("Load Pos", indexer.getRelPos(Motors.LOAD));
        SmartDashboard.putNumber("Load Abs", indexer.getAbsPos(Motors.LOAD));
    }

    private void joystickControls() {
        double intakePower = (1-driver.getRawAxis(2));
        SmartDashboard.putNumber("intakePower", intakePower*INTAKE_MOTOR_SPEED);
        indexer.rotate(Motors.INTAKE, intakePower*INTAKE_MOTOR_SPEED);
        //indexer.rotate(Motors.CONVEYOR, intakePower*CONVEYOR_MOTOR_SPEED);

        if (driver.isYPushed()) {
            indexer.toggleRunning(Motors.PRELOAD);
        }
        driver.clearButtons();
    }

    private void calibControls() {
        if (driver.isAPushed()) {
            indexer.zero();
        }
        // if (driver.isBPushed()) {
        // indexer.toggleControl();
        // }
        if (Math.abs(driver.getRawAxis(1)) > 0.05) {
            indexer.rotate(Motors.PRELOAD, Math.pow(driver.getRawAxis(1), 3) / 1.5);
        } else {
            indexer.rotate(Motors.PRELOAD, 0);
        }
        if (Math.abs(driver.getRawAxis(5)) > 0.05) {
            indexer.rotate(Motors.LOAD, Math.pow(driver.getRawAxis(5), 3) / 1.5);
        } else {
            indexer.rotate(Motors.LOAD, 0);
        }
        driver.clearButtons();
    }
}