package frc.team3373;

import java.io.IOException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.Indexer.Motors;

class Testbench extends TimedRobot {
    private SuperJoystick driver;
    private Indexer indexer;

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
        indexer = new Indexer(2, 3);
    }

    @Override
    public void robotPeriodic() {
        indexer.update();
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        joystickControls();
        indexer.update();
        SmartDashboard.putNumber("Ball 3 Pos", indexer.getRelPos(Motors.PRELOAD));
        SmartDashboard.putNumber("Ball 4 Pos", indexer.getRelPos(Motors.LOAD));
        SmartDashboard.putNumber("Ball 3 Abs", indexer.getAbsPos(Motors.PRELOAD));
        SmartDashboard.putNumber("Ball 4 Abs", indexer.getAbsPos(Motors.LOAD));
    }

    private void joystickControls() {
        if (driver.isAPushed()) {
            indexer.zero();
        }
        if (driver.isBPushed()) {
            indexer.toggleControl();
        }
        if (driver.isYPushed()) {
            indexer.toggleRunning(Motors.PRELOAD);
        }
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