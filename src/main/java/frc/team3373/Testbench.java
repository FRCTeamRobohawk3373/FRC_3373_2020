package frc.team3373;

import java.io.IOException;

import org.json.JSONException;

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
            System.out.println("Configs loaded");
        } catch (IOException | JSONException e) {
            try {
                Config.loadDefaults();
                System.out.println("Defaults loaded");
            } catch (Exception f) {
                System.out.println("Fatal config error");
                f.printStackTrace();
            }
        }
        driver = new SuperJoystick(0);
        indexer = new Indexer(4, 0, 2, 3, 7);

        SmartDashboard.putBoolean("Save Config", false);
        SmartDashboard.putBoolean("Restore Backup", false);
        SmartDashboard.putBoolean("Update Config", false);
        SmartDashboard.putBoolean("Restore Defaults", false);
    }

    @Override
    public void robotPeriodic() {
        if (SmartDashboard.getBoolean("Update Config", false)) {
            Config.updateValues();
            SmartDashboard.putBoolean("Update Config", false);
        } else if (SmartDashboard.getBoolean("Save Config", false)) {
            try {
                Config.saveConfig();
                SmartDashboard.putBoolean("Save Config", false);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else if (SmartDashboard.getBoolean("Restore Backup", false)) {
            try {
                Config.restoreBackup();
                SmartDashboard.putBoolean("Restore Backup", false);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else if (SmartDashboard.getBoolean("Restore Defaults", false)) {
            try {
                Config.loadDefaults();
                SmartDashboard.putBoolean("Restore Defaults", false);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            SmartDashboard.putBoolean("Save Config", false);
            SmartDashboard.putBoolean("Restore Backup", false);

        }

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
       
        indexer.getOccupied();
    }

    private void joystickControls() {
        //* double intakePower = (1 - driver.getRawAxis(2)*2);
        //* indexer.getMotor(Motors.INTAKE).set(intakePower * INTAKE_MOTOR_SPEED);
        // indexer.rotate(Motors.CONVEYOR, intakePower*CONVEYOR_MOTOR_SPEED);

        
        if (driver.isYPushed()) {
            indexer.toggleRunning(Motors.PRELOAD);
        }
        if (driver.isAPushed()) {
            indexer.shoot();
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