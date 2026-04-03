package frc.robot.lib;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PidValues {
    double kP;
    double kI;
    double kD;
    double kV;
    double kS;
    double maxAccel;
    double maxVel;

    public PidValues(double kP, double kI, double kD, double kV, double kS, double maxAccel, double maxVel) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
    }
}