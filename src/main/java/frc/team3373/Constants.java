package frc.team3373;
import frc.team3373.util.PIDConstant;

public class Constants{
    //* Robot Config
    //TODO change robot dimensions to match this years robot
    public static final double robotLength = 29.9375; //The larger dimension
    public static final double robotWidth = 29.875;


    public static final double wheelCircumference = 18.8495559215387594307759; //TODO measure this value
    //public static final double relativeEncoderRatio= 17.999954;
    public static final double relativeEncoderRatio= 17.9992371;

    public static final double ROTATIONAL_CORRECTION_FACTOR = 4; 



    public static final int FLDriveMotorID = 7;
    public static final int FLRotateMotorID = 8;

    public static final int FRDriveMotorID = 5;
    public static final int FRRotateMotorID = 6;

    public static final int BLDriveMotorID = 1;
    public static final int BLRotateMotorID = 2;

    public static final int BRDriveMotorID = 3;
    public static final int BRRotateMotorID = 4;

    public static final byte numberOfControlSegments = 8;

    //* Drive motors PID
    public static final PIDConstant DRIVE_FL_PID = new PIDConstant(5e-5, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant DRIVE_FR_PID = new PIDConstant(5e-5, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant DRIVE_BL_PID = new PIDConstant(5e-5, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant DRIVE_BR_PID = new PIDConstant(5e-5, 1e-6, 0, 1.56e-4, 0, -1, 1);

    //* Rotation motors PID
    public static final PIDConstant ROTATE_FL_PID = new PIDConstant(1, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant ROTATE_FR_PID = new PIDConstant(1, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant ROTATE_BL_PID = new PIDConstant(1, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant ROTATE_BR_PID = new PIDConstant(1, 1e-6, 0, 1.56e-4, 0, -1, 1);

    //* Encoder Positions 
    // Front left
    public static final double FLEncMin = 0.015625;
    public static final double FLEncMax = 3.3125;
    public static final double FLEncHome = 1.519531;

    // Back left
    public static final double BLEncMin = 0.4453125;
    public static final double BLEncMax = 3.2578125;
    public static final double BLEncHome = 2.406250;

    // Back right
    public static final double BREncMin = 0.44140625;
    public static final double BREncMax = 3.2890625;
    public static final double BREncHome = 2.406250;

    // Front right
    public static final double FREncMin = 0.45703125;
    public static final double FREncMax = 3.26953125;
    public static final double FREncHome = 1.402344;

    // Indexer constructor values
    public static final int INTAKE_INDEX = 4;
    public static final int CONVEYOR_INDEX = 0;
    public static final int PRELOAD_INDEX = 2;
    public static final int LOAD_INDEX = 3;
    public static final int BALL_SENSOR_INDEX = 7;

    //Climber values
    public static final double C_PP=0.5;
    public static final double C_PI=0;
    public static final double C_PD=0;

    public static final double C_WP=0.5;
    public static final double C_WI=0;
    public static final double C_WD=0;

    public static final int POLE_MOTOR_ID = 1;
    public static final int WINCH_MOTOR_ID = 4;
    public static final int PCM_ID = 10;
    public static final int POLE_SOLENOID_ID = 0;
    public static final int WINCH_SOLENOID_ID = 1;
    public static final int INCLINE_MOTOR_ID = -1;

    // Shooter constants
    public static final int SHOOTER_MOTOR_ID = 1;
    public static final double SHOOTER_BUMP_AMOUNT = 0.02;
    public static final double[][] SHOOT_TABLE = new double[][] {
        {24.000000, 0.071515},
        {36.000000, 0.087067},
        {48.000000, 0.102619},
        {60.000000, 0.118171},
        {72.000000, 0.135451},
        {84.000000, 0.152731},
        {96.000000, 0.166555},
        {108.000000, 0.180379},
        {120.000000, 0.197659},
        {132.000000, 0.213211},
    };
} 