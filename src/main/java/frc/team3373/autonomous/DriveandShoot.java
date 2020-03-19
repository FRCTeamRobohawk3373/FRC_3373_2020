package frc.team3373.autonomous;

import frc.team3373.AutonomousControl;
import frc.team3373.Indexer4;
import frc.team3373.Launcher;
import frc.team3373.SwerveControl;

public class DriveandShoot implements Runnable {
    private AutonomousControl autoControl;
    private SwerveControl swerve;
    private Launcher launcher;
    private Indexer4 indexer;
    private int state = 0;

    @Override
    public void run() {
        autoControl = AutonomousControl.getInstance();
        swerve = SwerveControl.getInstance();
        launcher = Launcher.getInstance();
        indexer = Indexer4.getInstance();
        swerve.resetOrentation();
        
        while (!Thread.interrupted()) {
            switch (state) {
                case 0: // Moving robot
                    autoControl.relativeMoveRobot(0, 0.3, 1.5);
                    state = 1;
                    break;
                case 1:
                    //if (autoControl.finishedMoving()) {
                    //    state = 3;
                    //}
            }
            /* swerve.relativeMoveRobot(0, 0.3, 1.5);
            launcher.setSpeed(0.4);
            launcher.setSpeedFromDistance(6 * 12);
            indexer.startShooting();
            while (!launcher.isUpToSpeed() && !exit) {
                sleep(20);
            }
            indexer.unloadBall();
            sleep(1000);
            indexer.unloadBall();
            sleep(1000);
            indexer.unloadBall();
            indexer.stopShooting();
            launcher.stop(); */
        }
    }
}