package frc.team3373;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Testbench extends TimedRobot {
    private SuperJoystick driver;
    private Indexer indexer;

    @Override
    public void robotInit() {
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
        SmartDashboard.putNumber("Ball 4 Pos", indexer.getRelBall4Pos());
        SmartDashboard.putNumber("Ball 5 Pos", indexer.getBall5Pos());
    }

    private void joystickControls() {
        if (driver.isAPushed()) {
            indexer.zero();
        }
        if (driver.isBPushed()) {
            indexer.toggleControl();
        }
        if (Math.abs(driver.getRawAxis(0)) > 0.05) {
            indexer.rotate4(Math.pow(driver.getRawAxis(0), 3) / 2);
        } else {
            indexer.rotate4(0);
        }
        driver.clearButtons();
    }
}