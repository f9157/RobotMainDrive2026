package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.TargetingConstants;

public final class AllianceZoneUtil {
    private AllianceZoneUtil() {
    }

    public static boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    public static boolean inAllianceZone(double robotX) {
        if (isBlueAlliance()) {
            return robotX <= TargetingConstants.blueAllianceZoneMaxXMeters;
        }

        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }

        return robotX >= TargetingConstants.redAllianceZoneMinXMeters;
    }
}
