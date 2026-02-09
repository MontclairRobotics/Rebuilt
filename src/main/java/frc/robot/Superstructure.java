package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseUtils;

public class Superstructure {

    public static Turret turret;
    public static Hood hood;
    public final Trigger isInScoringZoneTrigger = new Trigger(this::isInScoringZone);
    public final Trigger isInFerryingLeftZoneTrigger = new Trigger(this::isInFerryLeftZone);
    public final Trigger isInFerryingRightZoneTrigger = new Trigger(this::isInFerryRightZone);
    public final Trigger isApproachingTrenchTrigger = new Trigger(this:isApproachingTrench);

    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public Translation2d reflectPointByAlliance(Translation2d pos){
        if (isRedAlliance()){
            return PoseUtils.flipTranslationAlliance(pos);
        }
        else return pos;
    }

    public boolean isInScoringZone(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        return (reflectPointByAlliance(pos).getX() >= FieldConstants.LinesVertical.starting);
    }

    public boolean isInFerryLeftZone(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        return (reflectPointByAlliance(pos).getY() >= FieldConstants.LinesHorizontal.center &&
                reflectPointByAlliance(pos).getX() <= FieldConstants.LinesVertical.neutralZoneFar);
    }

    public boolean isInFerryRightZone(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        return (reflectPointByAlliance(pos).getY() <= FieldConstants.LinesHorizontal.center &&
        reflectPointByAlliance(pos).getX() <= FieldConstants.LinesVertical.neutralZoneFar);       
    }

    //public boolean isApproachingTrench(){
    //    Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
    //}

    Command scoringCommand = Commands.sequence(
        turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()),
        hood.setAngleCommand(() -> hood.getAngleToHub())
    );

    Command ferryingLeftCommand = Commands.sequence(
        turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_POINT)),
        hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_POINT, FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_HEIGHT))
    );

    Command ferryRightCommand = Commands.sequence(
        turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_POINT)),
        hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_POINT, FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_HEIGHT))
    );

    public Superstructure(){
    isInScoringZoneTrigger.onTrue(scoringCommand);
    isInFerryingLeftZoneTrigger.onTrue(ferryingLeftCommand);
    isInFerryingRightZoneTrigger.onTrue(ferryRightCommand);
    }
}

