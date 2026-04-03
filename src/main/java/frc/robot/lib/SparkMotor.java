package frc.robot.lib;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    public void apply(PidValues values) {
        ClosedLoopConfig pidConfig = new ClosedLoopConfig().pid(values.kP, values.kI, values.kD);
        SparkMaxConfig config = new SparkMaxConfig();
        config.apply(pidConfig);
        this.motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(otherCanId);
        config.inverted(invert);
        this.motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}
