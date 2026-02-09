package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.util.HubTracker;
import frc.robot.util.PoseUtils;

public class Superstructure {

    public static Turret turret;
    public static Hood hood;
    public final Trigger isInScoringZoneTrigger = new Trigger(this::shouldBeScoring);
    public final Trigger isInFerryingLeftZoneTrigger = new Trigger(this::isInFerryLeftZone);
    public final Trigger isInFerryingRightZoneTrigger = new Trigger(this::isInFerryRightZone);
    public final Trigger isApproachingTrenchTrigger = new Trigger(this::isApproachingTrench);

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
        return (reflectPointByAlliance(pos).getX() >= FieldConstants.LinesVertical.STARTING.in(Meters));
    }

    public boolean shouldBeScoring(){
        //Are we in the scoring zone and is the hub active
        return !isApproachingTrench()
        && isInScoringZone() 
        && HubTracker.isActive(DriverStation.getAlliance().get(), HubTracker.getCurrentShift().get());
    }

    public boolean isInFerryLeftZone(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        return !isApproachingTrench()
        && reflectPointByAlliance(pos).getY() >= FieldConstants.LinesHorizontal.CENTER.in(Meters) 
        && reflectPointByAlliance(pos).getX() <= FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters);
    }

    public boolean isInFerryRightZone(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();

        return !isApproachingTrench()
        && reflectPointByAlliance(pos).getY() <= FieldConstants.LinesHorizontal.CENTER.in(Meters) 
        && reflectPointByAlliance(pos).getX() <= FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters);       
    }

    public boolean isApproachingTrench(){
        return inTrenchZone() 
        && movingIntoObstacle();
    }

	// whether we are in the zone to apply trench lock
	private boolean inTrenchZone() {
        Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
        for (Translation2d[] zone : FieldConstants.Zones.TRENCH_ZONES) {
            if (robotPose.getX() >= zone[0].getX()
                    && robotPose.getX() <= zone[1].getX()
                    && robotPose.getY() >= zone[0].getY()
                    && robotPose.getY() <= zone[1].getY()) {
                return true;
            }
        }
        return false;
    }

	// whether we are in the zone to apply bump lock
	private boolean inBumpZone() {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		for (Translation2d[] zone : FieldConstants.Zones.BUMP_ZONES) {
            if (robotPose.getX() >= zone[0].getX()
                    && robotPose.getX() <= zone[1].getX()
                    && robotPose.getY() >= zone[0].getY()
                    && robotPose.getY() <= zone[1].getY()) {
                return true;
            }
        }
        return false;
	}

	// are we moving INTO the trench?
	private boolean movingIntoObstacle() {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		return
			(robotPose.getX() < FieldConstants.LinesVertical.ALLIANCE_ZONE.plus(FieldConstants.Hub.WIDTH.div(2)).plus(Constants.BUMPER_WIDTH).in(Meters)
			&& RobotContainer.drivetrain.getForwardVelocityFromController() > 0 )

			|| (robotPose.getX() > FieldConstants.LinesVertical.ALLIANCE_ZONE.in(Meters)
			&& robotPose.getX() < FieldConstants.LinesVertical.CENTER.in(Meters)
			&& RobotContainer.drivetrain.getForwardVelocityFromController() < 0)

			|| (robotPose.getX() < FieldConstants.LinesVertical.OPP_ALLIANCE_ZONE.in(Meters)
			&& robotPose.getX() > FieldConstants.LinesVertical.CENTER.in(Meters)
			&& RobotContainer.drivetrain.getForwardVelocityFromController() > 0)

			|| (robotPose.getX() > FieldConstants.LinesVertical.OPP_ALLIANCE_ZONE.minus(FieldConstants.Hub.WIDTH.div(2)).minus(Constants.BUMPER_WIDTH).in(Meters)
			&& RobotContainer.drivetrain.getForwardVelocityFromController() < 0);
	}

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

    Command stowCommand = Commands.sequence(
        turret.setFieldRelativeAngleCommand(TurretConstants.MIN_ANGLE),
        hood.setAngleCommand(HoodConstants.MIN_ANGLE)
    );

    public Superstructure(){
    isInScoringZoneTrigger.onTrue(scoringCommand);
    isInFerryingLeftZoneTrigger.onTrue(ferryingLeftCommand);
    isInFerryingRightZoneTrigger.onTrue(ferryRightCommand);
    isApproachingTrenchTrigger.onTrue(stowCommand);
    }
}