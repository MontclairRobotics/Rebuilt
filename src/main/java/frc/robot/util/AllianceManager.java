package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class AllianceManager {
    private static Alliance alliance = Alliance.Blue;
    public static boolean allianceKnown = false;

    private AllianceManager() {}

    public static void update() {
        DriverStation.getAlliance().ifPresent(a -> {
            alliance = a;
            allianceKnown = true;
        });
    }

    public static Alliance getAlliance() {
        return alliance;
    }

    public static boolean isRed() {
        return alliance == Alliance.Red;
    }

    public static boolean isAllianceKnown() {
        return allianceKnown;
    }
}
