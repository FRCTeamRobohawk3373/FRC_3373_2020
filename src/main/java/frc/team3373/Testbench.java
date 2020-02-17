package frc.team3373;

import java.io.IOException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        joystickControls();
        SmartDashboard.putNumber("Ball 3 Pos", indexer.getRelBall3Pos());
        SmartDashboard.putNumber("Ball 4 Pos", indexer.getRelBall4Pos());
        SmartDashboard.putNumber("Ball 3 Abs", indexer.getAbsBall3Pos());
        SmartDashboard.putNumber("Ball 4 Abs", indexer.getAbsBall4Pos());
    }

    private void joystickControls() {
        if (driver.isAPushed()) {
            indexer.zero();
        }
        if (driver.isBPushed()) {
            indexer.toggleControl();
        }
        if (Math.abs(driver.getRawAxis(1)) > 0.05) {
            indexer.rotate3(Math.pow(driver.getRawAxis(1), 3) / 1.5);
        } else {
            indexer.rotate3(0);
        }
        if (Math.abs(driver.getRawAxis(5)) > 0.05) {
            indexer.rotate4(Math.pow(driver.getRawAxis(5), 3) / 1.5);
        } else {
            indexer.rotate4(0);
        }
        driver.clearButtons();
    }
}