package frc.robot.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonFXSMotor implements PidMotor, MotorDiag, MotorFollow, SimMotor {

    public static class MotorType {
        private MotorArrangementValue arrangementValue;
        private DCMotor motor;

        private MotorType(MotorArrangementValue arrangement, DCMotor motor) {
            this.arrangementValue = arrangement;
            this.motor = motor;
        }

        public static MotorType getNeo() {
            return new MotorType(MotorArrangementValue.NEO_JST, DCMotor.getNEO(1));
        }

        public static MotorType getMinion() {
            return new MotorType(MotorArrangementValue.Minion_JST, DCMotor.getMinion(1));
        }
    }

    private int canId;

    private String name;

    private TalonFXS motor;

    private TalonFXSSimState sim;

    private MotorType type;

    private StatusSignal<AngularVelocity> velocity;

    private StatusSignal<Angle> position;

    private StatusSignal<Current> current;

    private MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    private MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);

    public TalonFXSMotor(int canId, String name, MotorType type) {
        this.canId = canId;
        this.name = name;
        this.motor = new TalonFXS(canId);
        this.position = this.motor.getPosition();
        this.velocity = this.motor.getVelocity();
        this.current = this.motor.getStatorCurrent();
        this.type = type;
        this.sim = this.motor.getSimState();

        TalonFXSConfiguration config = new TalonFXSConfiguration();

        config.Commutation.MotorArrangement = type.arrangementValue;
        this.motor.getConfigurator().apply(config);
    }

    public void apply(PidValues values) {
        SlotConfigs slot = new SlotConfigs();
        slot.kV = values.kV;
        slot.kP = values.kP;
        slot.kI = values.kI;
        slot.kD = values.kD;
        slot.kS = values.kS;
        MotionMagicConfigs mmConfig = new MotionMagicConfigs().withMotionMagicAcceleration(values.maxAccel)
                .withMotionMagicCruiseVelocity(values.maxVel);

        this.motor.getConfigurator().apply(slot);
        this.motor.getConfigurator().apply(mmConfig);
    }

    public TalonFXS getInner() {
        return this.motor;
    }

    public String getMotorName() {
        return this.name;
    }

    public void setDutyOut(double ref) {
        this.motor.set(ref);
    }

    public double getDutyOut() {
        return this.motor.get();
    }

    public void setPosition(double ref) {
        this.positionControl.Position = ref;
        this.motor.setControl(this.positionControl);
    }

    public double getPosition() {
        this.position.refresh();
        return this.position.getValueAsDouble();
    }

    public void setVelocity(double ref) {
        this.velocityControl.Velocity = ref;
        this.motor.setControl(this.velocityControl);
    }

    public double getVelocity() {
        this.velocity.refresh();
        return this.velocity.getValueAsDouble();
    }

    public double getCurrent() {
        this.current.refresh();
        return this.current.getValueAsDouble();
    }

    public String getMotorType() {
        return "TalonFXS";
    }

    public int getDeviceId() {
        return this.canId;
    }

    public void follow(int otherCanId, boolean invert) {

        MotorAlignmentValue mAlign;
        if (invert) {
            mAlign = MotorAlignmentValue.Opposed;
        } else {
            mAlign = MotorAlignmentValue.Aligned;
        }

        this.motor.setControl(new Follower(otherCanId, mAlign));
    }

    public void setEncoderPosition(double ref) {
        this.motor.setPosition(ref);
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public void feedSimulatedSensorData(double positionRotations, double velocityRotations) {
        this.sim.setRawRotorPosition(positionRotations);
        this.sim.setRotorVelocity(velocityRotations);
        this.sim.setSupplyVoltage(12);
    }

    public double getSimOutputVoltage() {
        return this.sim.getMotorVoltage();
    }

}
