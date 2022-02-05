package frc.team3373;

import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDInterface;
import frc.team3373.util.PIDConstant;

public class WheelDrive {
    public CANSparkMax angleMotor;
    public CANSparkMax speedMotor;
    public CANPIDController pidController;
    public final double MAX_VOLTS = 4.95;
    public WheelDrive (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new CANSparkMax (angleMotor, null);
        this.speedMotor = new CANSparkMax (speedMotor, null);
        pidController = new CANPIDController(this.angleMotor);
    
        pidController.setOutputRange(-1, 1);
        pidController.setContinuous();
        pidController.enable ();
    }
    public void drive (double speed, double angle) {
        speedMotor.set (speed);
        double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }
    
        pidController.setSetpoint(setpoint);
    }
    
}