package frc.team3373;
import frc.team3373.util.PIDConstant;

public class Constants{
    //* Robot Config
    public static final double robotLength = 29.9375; //TODO change robot dimensions to match this years robot
    public static final double robotWidth = 29.875;
    public static final double wheelCircumference = 18.8495559215387594307759; //TODO measure this value
    //public static final double relativeEncoderRatio= 17.999954;
    public static final double relativeEncoderRatio= 17.9992371;

    public static final int FLDriveMotorID = 7;
    public static final int FLRotateMotorID = 8;

    public static final int FRDriveMotorID = 5;
    public static final int FRRotateMotorID = 6;

    public static final int BLDriveMotorID = 1;
    public static final int BLRotateMotorID = 2;

    public static final int BRDriveMotorID = 3;
    public static final int BRRotateMotorID = 4;

    /* //* Drive motors PID
    // Front left
    public static final double driFLP=5e-5;
    public static final double driFLI=1e-6;
    public static final double driFLD=0;
    public static final double driFLF=1.56e-4;
    // Front right
    public static final double driFRP=5e-5;
    public static final double driFRI=1e-6;
    public static final double driFRD=0;
    public static final double driFRF=1.56e-4;
    // Back left
    public static final double driBLP=5e-5;
    public static final double driBLI=1e-6;
    public static final double driBLD=0;
    public static final double driBLF=1.56e-4;
    // Back right
    public static final double driBRP=5e-5;
    public static final double driBRI=1e-6;
    public static final double driBRD=0;
    public static final double driBRF=1.56e-4;

    //* Rotation motors PID
    // Front left
    public static final double rotFLP=1;
    public static final double rotFLI=1e-6;
    public static final double rotFLD=0;
    public static final double rotFLF=1.56e-4;
    // Front right
    public static final double rotFRP=1;
    public static final double rotFRI=1e-6;
    public static final double rotFRD=0;
    public static final double rotFRF=1.56e-4;
    // Back left
    public static final double rotBLP=1;
    public static final double rotBLI=1e-6;
    public static final double rotBLD=0;
    public static final double rotBLF=1.56e-4;
    // Back right
    public static final double rotBRP=1;
    public static final double rotBRI=1e-6;
    public static final double rotBRD=0;
    public static final double rotBRF=1.56e-4; */

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
    public static final double FLEncMin = 0.023438;
    public static final double FLEncMax = 3.316406;
    public static final double FLEncHome = 1.531250;

    // Front right
    public static final double FREncMin = 0.476563;
    public static final double FREncMax = 3.261719;
    public static final double FREncHome = 1.421875;

    // Back left
    public static final double BLEncMin = 0.472656;
    public static final double BLEncMax = 3.250000;
    public static final double BLEncHome = 2.414063;

    // Back right
    public static final double BREncMin = 0.460938;
    public static final double BREncMax = 3.281250;
    public static final double BREncHome = 2.402344;
} 