package frc.team3373.util;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class PIDConstant {

    private double kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput;

    public PIDConstant(double P, double I, double D, double FF, double IZone, double MINOutput,
            double MaxOutput) {
        this.kP=P;
        this.kI=I;
        this.kD=D;
        this.kFF=FF;
        this.kIz=IZone;
        this.kMinOutput=MINOutput;
        this.kMaxOutput=MaxOutput;
        
    }

    public void configPID(CANSparkMax motor) {
        CANPIDController pidcontrol = motor.getPIDController();
        pidcontrol.setP(kP);
        pidcontrol.setI(kI);
        pidcontrol.setD(kD);
        pidcontrol.setFF(kFF);

        pidcontrol.setIZone(kIz);
        pidcontrol.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void configPID(CANPIDController pidcontrol) {
        pidcontrol.setP(kP);
        pidcontrol.setI(kI);
        pidcontrol.setD(kD);
        pidcontrol.setFF(kFF);

        pidcontrol.setIZone(kIz);
        pidcontrol.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void configPID(WPI_TalonSRX pidcontrol, int slotID) {
        pidcontrol.config_kP(slotID, kP);
        pidcontrol.config_kI(slotID, kI);
        pidcontrol.config_kD(slotID, kD);
        pidcontrol.config_kF(slotID, kFF);

        pidcontrol.config_IntegralZone(slotID, (int) kIz);
        // pidcontrol.setOutputRange(kMinOutput, kMaxOutput);
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getIZone() {
        return kIz;
    }

    public double getFeedForward() {
        return kFF;
    }

    public double getMinOutput() {
        return kMinOutput;
    }

    public double getMaxOutput() {
        return kMaxOutput;
    }

    public String toString(){
        return "P="+kP+",I="+kI+",D="+kD+",FF="+kFF+",IZone="+kIz+",Range("+kMinOutput+","+kMaxOutput+")";
    }

}