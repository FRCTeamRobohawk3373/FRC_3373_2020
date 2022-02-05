/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373;

import java.io.IOException;

import org.json.JSONException;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.SPI;
import frc.team3373.Indexer4.State;

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
    public final double L = 18;
    public final double W = 17;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private AutonomousControl autoControl;

    private SuperJoystick driver;
    private SuperJoystick shooter;

    private SuperAHRS ahrs;
    private Launcher launcher;
    private Indexer4 indexer;
    private Climber climber;
    private Vision vis;
    // TODO add ManualInput.java?
    
    private int calibrationMode = -1;

    boolean firstTimeC;

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
            //Config.loadDefaults();
            //Config.saveConfig();
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

        autoControl = AutonomousControl.getInstance();

        driver = new SuperJoystick(0);
        shooter = new SuperJoystick(1);

        launcher = Launcher.getInstance();

        climber = Climber.getInstance();//TODO uncomment

        ahrs = SuperAHRS.getInstance();
        indexer = Indexer4.getInstance();


        //swerve.recalculateWheelPosition();
        //swerve.resetOrentation();

        //indexer.setInitialBallStates(new State[]{State.AVAILABLE,State.OCCUPIED,State.OCCUPIED,State.OCCUPIED});

        vis = Vision.getInstance();

        firstTimeC = true;

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
                if(RobotState.isTest()){
                    System.out.println("Saving Config!");
                    Config.saveConfig();
                    SmartDashboard.putBoolean("Save Config", false);
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else if (SmartDashboard.getBoolean("Restore Backup", false)) {
            try {
                if(RobotState.isTest()){
                    System.out.println("Restore Config!");
                    Config.restoreBackup();
                    SmartDashboard.putBoolean("Restore Backup", false);
                }
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
        //swerve.showPositions();
        
        /*if (shooter.isStartPushed()) {
            indexer.setInitialBallStates(
                new State[] { State.OCCUPIED, State.OCCUPIED, State.OCCUPIED, State.OCCUPIED, State.OCCUPIED });
        }*/
        // swerve.getRotationalCorrection();

        /*
         * if(driver.isBackPushed()){ ahrs.reset(); } driver.clearButtons();
         */
    }

    @Override
    public void disabledPeriodic() {
        if(shooter.isStartPushed()){
            indexer.setInitialBallStates(new State[]{State.AVAILABLE,State.OCCUPIED,State.OCCUPIED,State.OCCUPIED}); 
            //indexer.setInitialBallStates(new State[]{State.AVAILABLE,State.AVAILABLE,State.AVAILABLE,State.AVAILABLE}); 
        }
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
        m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        //TODO Needs more testing with balls
        //indexer.setInitialBallStates(new State[]{State.AVAILABLE,State.AVAILABLE,State.AVAILABLE,State.AVAILABLE}); 
        System.out.println("Auto selected: " + m_autoSelected);
        indexer.startInit();
        autoControl.start(0);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        indexer.update();
        autoControl.update();
    }

    public void teleopInit() {
        //swerve.resetOrentation();
        indexer.startInit();
    }

    /**
     * This function is called periodically during operator control.
     */
    public void teleopPeriodic() {
        SwerveDrive.drive(driver.getRawAxis(1), driver.getRawAxis(0), driver.getRawAxis(4));
        indexer.update();
        launcher.updateTeleOp();
        climber.update();
        /*
         * try { Thread.sleep(1000); } catch (InterruptedException e) {
         * e.printStackTrace(); }
         */
    }



    @Override
    public void testInit() {
        climber.unlockSolenoids();
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

                    if (driver.isDPadUpPushed()){ 
                        calibrationMode = 1;
                        climber.unlockSolenoids();
                    }// Climber
                    if (driver.isDPadRightPushed()) calibrationMode = 2;// Calibrate launcher
                    if (driver.isDPadDownPushed()) calibrationMode = 3; // Calibrate indexer
                    if (driver.isDPadLeftPushed()) calibrationMode = 4; // calabrate swerve

                    break;

                case 1://! Calibrate Climber
                SmartDashboard.putString("Calibrate", "Climber");
               
                if (firstTimeC) {
                    firstTimeC = false;
                        climber.calibrateInches();
                    }
                    
                    if (climber.getCalibrating()) {// If calibrating (B pressed), control each motor individually
                        climber.calibrateControl(-driver.getRawAxis(1), -driver.getRawAxis(5));
                    } else {// If not calibrating, control both motors together
                        
                        climber.teleOpControl(-driver.getRawAxis(1), driver.getRawAxis(0));
                    }

                    if (driver.isBPushed()) {// Go into calibration mode
                        climber.calibrateInches();
                    }

                    climber.update();
                    climber.displayOnShuffleboard();
                    break;

                case 2://! Calibrate Launcher
                    SmartDashboard.putString("Calibrate", "Launcher");

                    if (driver.isAPushed()) {
                        launcher.nextCalibrationStep();
                    }
                    launcher.calibrationMotorSpeed(driver.isLBPushed(), driver.isRBPushed(), driver.getRawAxis(2), driver.getRawAxis(3));
                    if (launcher.updateCalibration()) {
                        calibrationMode = -1;
                    }
                    break;
                    
                case 3: //! Calibrate Indexer
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
