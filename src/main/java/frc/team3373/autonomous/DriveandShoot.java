package frc.team3373.autonomous;

import frc.team3373.Indexer4;
import frc.team3373.Launcher;
import frc.team3373.SwerveControl;

public class DriveandShoot implements AutoScript {
    private SwerveControl swerve;
    private Launcher launcher;
    private Indexer4 indexer;

    @Override
    public void init() {
        swerve = SwerveControl.getInstance();
        launcher = Launcher.getInstance();
        indexer = Indexer4.getInstance();
    }

    @Override
    public void start() {
        swerve.relativeMoveRobot(0, 0.5, 5);
        launcher.setSpeedFromDistance(6 * 12);
        while (!launcher.isUpToSpeed()) {
            sleep(20);
        }
        indexer.unloadBall();
        sleep(1000);
        indexer.unloadBall();
        sleep(1000);
        indexer.unloadBall();
    }

    private void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {

        }
    }

    @Override
    public void stop() {
        swerve = null;
        launcher = null;
        indexer = null;
    }

}