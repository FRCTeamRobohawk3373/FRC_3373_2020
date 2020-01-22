package frc.team3373;

public class Constants{
    //* Robot Config
    public static final double robotWidth = 22.25; //TODO change robot dimensions to match this years robot
    public static final double robotLength = 16.25;
    public static final double wheelCircumference = 18.8495559215387594307759; //TODO measure this value

    //* Drive motors
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

    //* Rotation motors
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
    public static final double rotBRF=1.56e-4;

    //* Encoder Positions 
    // Front left
    public static final double FLEncMin = 0.003;
    public static final double FLEncMax = 3.21;
    public static final double FLEncHome = 0.976;

    // Front right
    public static final double FREncMin = 0.003;
    public static final double FREncMax = 3.21;
    public static final double FREncHome = 0.976;

    // Back left
    public static final double BLEncMin = 0.003;
    public static final double BLEncMax = 3.21;
    public static final double BLEncHome = 0.976;

    // Back right
    public static final double BREncMin = 0.003;
    public static final double BREncMax = 3.21;
    public static final double BREncHome = 0.976;
} 