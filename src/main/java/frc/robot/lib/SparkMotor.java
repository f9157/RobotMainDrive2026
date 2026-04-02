package frc.robot.lib;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SparkMotor implements PidMotor, MotorDiag, MotorFollow {

    SparkMax motor;

    RelativeEncoder encoder;

    SparkClosedLoopController controller;

    int canId;

    String name;

    public SparkMotor(int canId, String name, boolean brushed) {
        this.canId = canId;
        this.name = name;
        MotorType mType;
        if (brushed) {
            mType = MotorType.kBrushed;
        } else {
            mType = MotorType.kBrushless;
        }

        this.motor = new SparkMax(canId, mType);

        this.encoder = this.motor.getEncoder();

        this.controller = this.motor.getClosedLoopController();
    }

    public SparkMax getInner() {
        return this.motor;
    }

    public String getMotorType() {
        return "SparkMax";
    }

    public String getMotorName() {
        return this.name;
    }

    public int getDeviceId() {
        return this.canId;
    }

    public void setDutyOut(double ref) {
        this.motor.set(ref);
    }

    public double getDutyOut() {
        return this.motor.get();
    }

    public void setPosition(double ref) {
        this.controller.setSetpoint(ref, ControlType.kPosition);
    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public void setVelocity(double ref) {
        this.controller.setSetpoint(ref, ControlType.kVelocity);
    }

    public double getVelocity() {
        return this.encoder.getVelocity();
    }

    public double getCurrent() {
        return this.motor.getOutputCurrent();
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public void setEncoderPosition(double ref) {
       this.encoder.setPosition(ref); 
    }

    public void follow(int otherCanId, boolean invert) {
        
    }
}
