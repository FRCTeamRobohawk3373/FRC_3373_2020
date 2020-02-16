package frc.team3373;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        System.out.println(indexer.getBall4Pos());
        System.out.println(indexer.getBall5Pos());
    }

    private void joystickControls() {
        if (driver.isAPushed()) {
            indexer.zero();
        }
        if (driver.isBPushed()) {
            indexer.toggleControl();
        }
        if (driver.getRawAxis(0) > 0.1) {
            indexer.rotate4(driver.getRawAxis(0));
        }
    }
}