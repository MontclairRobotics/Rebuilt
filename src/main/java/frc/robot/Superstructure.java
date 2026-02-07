package frc.robot;

import com.google.flatbuffers.FlexBuffers.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldConstants;

public class Superstructure {

    public static Turret turret;
    public static Hood hood;
    public CommandScheduler commandScheduler;
    private final Modes zone = Modes.NO_ZONE;
    private final Map<zone, Command> zoneCommands;
    public final Trigger isRedAllianceTrigger = new Trigger (this::isRedAlliance);
    public final Trigger inScoringZoneTrigger = new Trigger(this::isInScoringZone);
    public final Trigger ixnFerryingLeftZoneTrigger = new Trigger(this::isInFerryLeftZone);
    public final Trigger inFerryingRightZoneTrigger = new Trigger(this::isInFerryRightZone);

    public enum Modes {
        SCORING,
        FERRYING_LEFT,
        FERRYING_RIGHT,
        NO_ZONE
    }


    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public boolean isInScoringZone(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        return (pos.getX() >= FieldConstants.LinesVertical.starting);
    }

    public boolean isInFerryLeftZone(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        return (pos.getY() >= FieldConstants.LinesHorizontal.center &&
                pos.getX() <= FieldConstants.LinesVertical.neutralZoneFar);
    }

    public boolean isInFerryRightZone(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        return (pos.getY() <= FieldConstants.LinesHorizontal.center &&
        pos.getX() <= FieldConstants.LinesVertical.neutralZoneFar);       
    }

    zoneCommands = Map.of(
        Modes.SCORING,
        Commands.sequence(
            turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()),
            hood.setAngleCommand(() -> hood.getAngleToHub)
        ),
        Modes.FERRYING_LEFT(
            turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_POINT)),
            hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_POINT, FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_HEIGHT))
        ),
        Modes.FERRYING_RIGHT(
            turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_POINT)),
            hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_POINT, FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_HEIGHT))
        ),
        Modes.NO_ZONE(
        );
    )
}

