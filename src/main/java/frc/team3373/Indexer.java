package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
    private WPI_TalonSRX intake;
    private WPI_TalonSRX conveyor;
    private LoadingMotor preload;
    private LoadingMotor load;
    private DigitalInput distanceSensor;
    private boolean manual;

    public enum Motors {
        INTAKE, CONVEYOR, PRELOAD, LOAD
    }

    public Indexer(int intakeIndex, int conveyorIndex, int preloadIndex, int loadIndex) {
        preload = new LoadingMotor(preloadIndex);
        load = new LoadingMotor(loadIndex);
        preload.setNeutralMode(NeutralMode.Brake);
        load.setNeutralMode(NeutralMode.Brake);

        preload.configOpenloopRamp(0);

        distanceSensor = new DigitalInput(7);

        manual = false;

        SmartDashboard.putBoolean("Manual", manual);
    }

    // TODO Remove unneeded constructor
    public Indexer(int preloadIndex, int loadIndex) {
        this(0, 0, preloadIndex, loadIndex);
    }

    public void update() {
        SmartDashboard.putBoolean("Sensor", distanceSensor.get());
        SmartDashboard.putBoolean("Preload Running", preload.running);
        SmartDashboard.putBoolean("Preload Stopping", preload.stopping);

        preload.update();
        load.update();
    }

    public boolean toggleRunning(Motors motor) {
        LoadingMotor m = (LoadingMotor)getMotor(motor);
        if (!m.running) {
            m.run();
            return true;
        }
        m.stop();
        return false;
    }

    public boolean getManual() {
        return manual;
    }

    private WPI_TalonSRX getMotor(Motors motor) {
        switch (motor) {
            case INTAKE:
                return intake;
            case CONVEYOR:
                return conveyor;
            case PRELOAD:
                return preload;
            case LOAD:
                return load;
            default:
                return null;
        }
    }

    public int getAbsPos(Motors motor) {
        return ((LoadingMotor)getMotor(motor)).getAbsPos();
    }

    public int getRelPos(Motors motor) {
        return ((LoadingMotor)getMotor(motor)).getRelPos();
    }

    public boolean toggleControl() {
        manual = !manual;
        if (manual) {
            preload.setNeutralMode(NeutralMode.Coast);
            load.setNeutralMode(NeutralMode.Coast);
        }
        SmartDashboard.putBoolean("Manual", manual);
        return manual;
    }

    public boolean rotate(Motors motor, double speed) {
        if (manual) {
            getMotor(motor).set(speed);
            return true;
        }
        return false;
    }

    public void zero() {
        preload.getSensorCollection().setQuadraturePosition(0, 20);
        load.getSensorCollection().setQuadraturePosition(0, 20);
    }


    class LoadingMotor extends WPI_TalonSRX {
        private boolean running;
        private boolean stopping;
        private double speed;
        private int scale;

        public LoadingMotor(int index) {
            super(index);
            running = false;
            scale = (int)Config.getNumber("encoderScale", 1992);
        }

        private double run() {
            running = true;
            speed = Config.getNumber("indexerSpeed", -1);
            super.set(-1);
            return speed;
        }

        private void stop() {
            running = false;
            stopping = true;
            super.set(-0.4);
        }

        private void update() {
            if (stopping) {
                scale = (int)Config.getNumber("encoderScale", 1992);
                //int deadband = 100; ////(int)Config.getNumber("loaderDeadband", 20);
                if (getRelPos() > 1942 || getRelPos() < 50) {
                    super.set(0);
                    stopping = false;
                }
            }
        }

        private int getAbsPos() {
            return super.getSensorCollection().getQuadraturePosition();
        }

        private int getRelPos() {
            int relPos = getAbsPos() % scale;
            if (relPos < 0) {
                return scale + relPos;
            }
            return relPos;
        }
    }
}
