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

//import frc.team3373.util.MathUtil;
import frc.team3373.SwerveControl.DriveMode;
// import frc.team3373.SwerveControl.Side;

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
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private Climber climber;
    // TODO add ManualInput.java?
    
    boolean firstTimeC;

    private SuperJoystick shooter, driver;
    private int calibrationMode;

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

        climber = Climber.getInstance();

        driver = shooter = new SuperJoystick(0);

        firstTimeC = true;

        calibrationMode = 0;

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
     
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    public void teleopInit() {
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        joystickControls();
       
    }

    private void joystickControls() {
       /*
         * ######################### Shooter Controls #########################
         */
        
        //* Climber
        climber.teleOpControl(-shooter.getRawAxis(1), shooter.getRawAxis(2));
    
        if (shooter.isDPadDownPushed()) {
            climber.gotoLowPosition();
        } else if (shooter.isDPadLeftPushed() || shooter.isDPadRightPushed()) {
            climber.gotoMiddlePosition();
        } else if (shooter.isDPadUpPushed()) {
            climber.gotoHighPosition();
        } else if (shooter.isYPushed()) {
            climber.changeClimbMode();
        }

        climber.update();
        //*/

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

                case 1:// Calibrate Climber
                    SmartDashboard.putString("Calibrate", "Climber");
               
                    if (firstTimeC) {
                        firstTimeC = false;
                        //climber.changeClimbMode();
                        climber.calibrateInches();
                        //climber.startCalibrateOptions();
                    }
                    
                    if (climber.getCalibrating()) {// If calibrating (B pressed), control each motor individually
                        climber.calibrateControl(-driver.getRawAxis(1), -driver.getRawAxis(5));
                    } else {// If not calibrating, control both motors together
                        
                        climber.teleOpControl(-driver.getRawAxis(1), driver.getRawAxis(2));//!
                    }

                    if (driver.isBPushed()) {// Go into calibration mode
                        climber.calibrateInches();
                    }

                    climber.update();
                    climber.displayOnShuffleboard();
                    break;

                case 2:
                    SmartDashboard.putString("Calibrate", "Launcher");


                    break;
                    
                case 3: 
                    SmartDashboard.putString("Calibrate", "Indexer");

                    
                    break;
                    
                case 4:
                    SmartDashboard.putString("Calibrate", "Swerve");

                    break;
                
                default:
                    SmartDashboard.putString("Calibrate", "broken robot mode");
                    break; //Don Nothing if invalid mode
            }
        } else {
            SmartDashboard.putString("Calibrate", "None");
        }

        // Reset joysticks
        driver.clearButtons();
        driver.clearDPad();
        shooter.clearButtons();
        shooter.clearDPad();

    }
}
