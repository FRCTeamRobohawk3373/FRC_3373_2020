/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373;

import java.io.IOException;

import org.json.JSONException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.SPI;

//import frc.team3373.util.MathUtil;
import frc.team3373.SwerveControl.DriveMode;
import frc.team3373.SwerveControl.Side;

import frc.team3373.Indexer.State;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private static final String kDriveAuto = "Test drive";
    private static final String kRelRotAuto = "Test RelRotate";
    private static final String kAbsRotAuto = "Test ABSRotate";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private SuperJoystick driver;
    private SuperJoystick shooter;

    private SwerveControl swerve;
    private SuperAHRS ahrs;
    private Launcher launcher;
    private Indexer4 indexer;
    private Climber climber;
    // TODO add ManualInput.java?
    
    private int calibrationMode = -1;

    // private double target = 0.0;

    // private int index = 0;

    // private double min = Double.MAX_VALUE;
    // private double max = Double.MIN_VALUE;

    // private int mode = 0;
    // private double endtime = System.currentTimeMillis();

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        m_chooser.addOption("Test drive", kDriveAuto);
        m_chooser.addOption("Test RelRotate", kRelRotAuto);
        m_chooser.addOption("Test ABSRotate", kAbsRotAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

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
        shooter = new SuperJoystick(1);

        launcher = Launcher.getInstance();

        ahrs = SuperAHRS.getInstance();
        indexer = Indexer4.getInstance();

        swerve = SwerveControl.getInstance();
        swerve.setDriveSpeed(0.25);
        swerve.changeControllerLimiter(0);

        SmartDashboard.putBoolean("Save Config", false);
        SmartDashboard.putBoolean("Restore Backup", false);
        SmartDashboard.putBoolean("Update Values", false);
        SmartDashboard.putBoolean("Restore Defaults", false);

        SmartDashboard.putNumber("Shoot Distance", 0);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        if (SmartDashboard.getBoolean("Update Values", false)) {
            System.out.println("Updating Values!");
            Config.updateValues();
            SmartDashboard.putBoolean("Update Values", false);
        } else if (SmartDashboard.getBoolean("Save Config", false)) {
            try {
                System.out.println("Saving Config!");
                Config.saveConfig();
                SmartDashboard.putBoolean("Save Config", false);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else if (SmartDashboard.getBoolean("Restore Backup", false)) {
            try {
                System.out.println("Restore Config!");
                Config.restoreBackup();
                SmartDashboard.putBoolean("Restore Backup", false);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else if (SmartDashboard.getBoolean("Restore Defaults", false)) {
            try {
                System.out.println("Restore Defaults!");
                Config.loadDefaults();
                SmartDashboard.putBoolean("Restore Defaults", false);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            SmartDashboard.putBoolean("Save Config", false);
            SmartDashboard.putBoolean("Restore Backup", false);

        }
        double orientationOffset = Math.toRadians(ahrs.getYaw());
        SmartDashboard.putNumber("orientationDegree", Math.toDegrees(orientationOffset));
        SmartDashboard.putNumber("orientationRadians", orientationOffset);
        SmartDashboard.putBoolean("isNavxCalibrating", ahrs.isCalibrating());
        swerve.showPositions();
        
        /*if (shooter.isStartPushed()) {
            indexer.setInitialBallStates(
                new State[] { State.OCCUPIED, State.OCCUPIED, State.OCCUPIED, State.OCCUPIED, State.OCCUPIED });
        }*/
        // swerve.getRotationalCorrection();

        /*
         * if(driver.isBackPushed()){ ahrs.reset(); } driver.clearButtons();
         */
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        m_autoSelected = m_chooser.getSelected();
        switch (m_autoSelected) {
        case kCustomAuto:
            // Put custom auto code here
            break;
        case kDriveAuto:
            swerve.relativeMoveRobot(0, .15, 1);
            break;
        case kRelRotAuto:
            swerve.setDriveSpeed(0.45);
            swerve.relativeRotateRobot(90);
            break;
        case kAbsRotAuto:
            swerve.absoluteRotateRobot(270);
            break;
        case kDefaultAuto:
        default:
            // Put default auto code here
            break;
        }

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void teleopInit() {
        swerve.resetOrentation();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        joystickControls();
        indexer.update();
        launcher.updateTeleOp();
        /*
         * try { Thread.sleep(1000); } catch (InterruptedException e) {
         * e.printStackTrace(); }
         */
    }

    private void joystickControls() {
        /*
         * ######################## Driver Controls ########################
         */

        swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4) * 0.75);

        if (driver.isBPushed()) {
            swerve.changeControllerLimiter();
        }

        if (driver.isLBHeld()) {
            swerve.setDriveSpeed(0.15);
        } else if (driver.isRBHeld()) {
            swerve.setDriveSpeed(.75);
        } else {
            swerve.setDriveSpeed(0.45);
        }

        /*
         * if(driver.isStartPushed()){ swerve.calibrateHome(); }
         */

        if (driver.isYPushed()) {
            swerve.resetOrentation();
        }

        if (driver.getRawAxis(2) > 0.8)
            swerve.setControlMode(DriveMode.FIELDCENTRIC);
        else if (driver.getRawAxis(3) > 0.8)
            swerve.setControlMode(DriveMode.ROBOTCENTRIC);

        switch (driver.getPOV()) {
        case 0:
            swerve.changeFront(SwerveControl.Side.NORTH);
            break;
        case 90:
            swerve.changeFront(SwerveControl.Side.EAST);
            break;
        case 180:
            swerve.changeFront(SwerveControl.Side.SOUTH);
            break;
        case 270:
            swerve.changeFront(SwerveControl.Side.WEST);
            break;
        }

        if (driver.isBackPushed()) {
            swerve.recalculateWheelPosition();
        }

        /*
         * ######################### Shooter Controls #########################
         */
        if (shooter.isXPushed()) {
            indexer.stopIntake();
        }
        if (shooter.isYPushed()) {
            indexer.startIntake();
        }
        if (shooter.isAHeld()) { // ? Change?
            indexer.unloadBall5();
        }
        if (shooter.getRawAxis(3) > 0.5) {
            indexer.startShooting();

            double launcher_inches = SmartDashboard.getNumber("Shoot Distance", 0);
            launcher.setSpeedFromDistance(launcher_inches);
        } else {
            launcher.stop();
            indexer.stopShooting();
        }
        /* if (shooter.isBackPushed()) {
            indexer.reverseConveyor();
        } */

        if (shooter.isLBPushed()) {
            launcher.bumpDownSpeed();
        }
        if (shooter.isRBPushed()) {
            launcher.bumpUpSpeed();
        }
        if (shooter.getRawAxis(2) > 0.5) {
            indexer.enterPanicMode();
        }
        if (shooter.isBackPushed()) {
            indexer.zeroMotors();
        }

        driver.clearButtons();
        driver.clearDPad();
        shooter.clearButtons();
        shooter.clearDPad();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

        if (driver.isStartPushed()) {
            calibrationMode = 0;
        }
        if(calibrationMode >= 0) {
            if (driver.isXPushed()){
                calibrationMode = -1;// Abort
                launcher.setFirstTime(true);// Rewind launcher for second calibration if necessary 
                SmartDashboard.putString("Calibrate", "None");
            }
            
            switch (calibrationMode) {
                case 0:
                    SmartDashboard.putString("Calibrate", "Menu");

                    if (driver.isDPadUpPushed()) calibrationMode = 1;// Climber
                    if (driver.isDPadRightPushed()) calibrationMode = 2;// Calibrate launcher
                    if (driver.isDPadDownPushed()) calibrationMode = 3; // Calibrate indexer
                    if (driver.isDPadLeftPushed()) calibrationMode = 4; // calabrate swerve

                    break;

                case 1:// Calibrate launcher
                    SmartDashboard.putString("Calibrate", "Climber");
                    /*if (driver.isAPushed()) {
                        climber.calibrateHeights();
                    }
                    climber.calibrateInches();*///! uncomment and fix first-time logic
                    break;

                case 2:
                    SmartDashboard.putString("Calibrate", "Launcher");

                    if (driver.isAPushed()) {
                        launcher.nextCalibrationStep();
                    }
                    launcher.calibrationMotorSpeed(driver.isLBPushed(), driver.isRBPushed(), driver.getRawAxis(2), driver.getRawAxis(3));
                    if (launcher.updateCalibration()) {
                        calibrationMode = -1;
                    }
                    break;
                    
                case 3: 
                    SmartDashboard.putString("Calibrate", "Indexer");

                    if (driver.isDPadUpPushed()) {
                        indexer.configTiming("indexer");
                    } else if (driver.isDPadDownPushed()) {
                        indexer.configTiming("conveyor");
                    } else if (driver.isDPadLeftPushed()) {
                        indexer.configTiming("preload");
                    } else if (driver.isDPadRightPushed()) {
                        indexer.configTiming("load");
                    } else if (driver.isAPushed()) {
                        indexer.configTiming("conveyor_center");
                    }
                    if (driver.getRawAxis(2) > 0.5) {
                        indexer.enterPanicMode();
                    }
                    break;
                    
                case 4:
                    SmartDashboard.putString("Calibrate", "Swerve");

                    if (driver.isYPushed()) {
                        swerve.recalculateWheelPosition();
                    }

                    if (driver.isAPushed()) {
                        swerve.calibrateHome();
                    }
                    if (driver.isBPushed()) {
                        swerve.calibrateMinMax();
                    }

                    swerve.showPositions();
                    
                    if(driver.isRBHeld()){
                        swerve.setDriveSpeed(0.15);
                        swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4) * 0.75);
                    }
                    break;
                
                default:
                    SmartDashboard.putString("Calibrate", "broken robot mode");
                    break; //Don Nothing if invalid mode
            }
        } else {
            SmartDashboard.putString("Calibrate", "None");

            //* Shooter controls
            if (Math.abs(shooter.getRawAxis(1)) > 0.05) {
                indexer.moveMotor("preload", Math.pow(shooter.getRawAxis(1), 3) * 0.3);
            } else {
                indexer.moveMotor("preload", 0);
            }
            if (Math.abs(shooter.getRawAxis(5)) > 0.05) {
                indexer.moveMotor("load", Math.pow(shooter.getRawAxis(5), 3) * 0.3);
            } else {
                indexer.moveMotor("load", 0);
            }        
            launcher.setSpeed(Math.pow(shooter.getRawAxis(3), 2) / 2);
            if (shooter.isLBPushed()) {
                launcher.bumpDownSpeed();
            }
            if (shooter.isRBPushed()) {
                launcher.bumpUpSpeed();
            }
            if (shooter.getRawAxis(2) > 0.5) {
                indexer.enterPanicMode();
            }
            launcher.updateTeleOp();
        }
        indexer.updateTest();

        // Reset joysticks
        driver.clearButtons();
        driver.clearDPad();
        shooter.clearButtons();
        shooter.clearDPad();

    }
}
