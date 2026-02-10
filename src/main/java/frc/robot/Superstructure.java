package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import frc.robot.constants.Constants;
import frc.robot.util.HubTracker;
import frc.robot.util.PoseUtils;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private Shooter shooter;

    public Superstructure(Shooter shooter){
    this.shooter = shooter;
    scoringModeTrigger.onTrue(shooter.scoringCommand());
    ferryLeftTrigger.onTrue(shooter.ferryingLeftCommand());
    ferryRightTrigger.onTrue(shooter.ferryRightCommand());
    trenchDangerZoneTrigger.onTrue(shooter.stowCommand());
    }

    public final Trigger scoringModeTrigger = new Trigger(this::shouldBeScoring);
    public final Trigger ferryLeftTrigger = new Trigger(this::shouldFerryLeft);
    public final Trigger ferryRightTrigger = new Trigger(this::shouldFerryRight);
    public final Trigger trenchDangerZoneTrigger = new Trigger(this::inTrenchDangerZone);

    @Override
    public void periodic(){
        Logger.recordOutput("Superstructure/shouldBeScoring", shouldBeScoring());
        Logger.recordOutput("Superstructure/shouldFerryLeft", shouldFerryLeft());
        Logger.recordOutput("Superstructure/shouldFerryRight", shouldFerryRight());
        Logger.recordOutput("Superstructure/inTrenchDangerZone", inTrenchDangerZone());
    };

    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        Logger.recordOutput("Superstructure/isRedAlliance",alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red);
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public boolean isInScoringZone(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        Logger.recordOutput("Superstructure/isInScoringZone", !inTrenchDangerZone()
        && (isRedAlliance() ?
        pos.getX() >= PoseUtils.flipTranslationAlliance(
            new Translation2d(FieldConstants.LinesVertical.STARTING.in(Meters), 0)).getX()
        : pos.getX() <= FieldConstants.LinesVertical.STARTING.in(Meters)));

        return !inTrenchDangerZone()
        && (isRedAlliance() ?
        pos.getX() >= PoseUtils.flipTranslationAlliance(
            new Translation2d(FieldConstants.LinesVertical.STARTING.in(Meters), 0)).getX()
        : pos.getX() <= FieldConstants.LinesVertical.STARTING.in(Meters));
    }

    public boolean shouldBeScoring(){
        Logger.recordOutput("Superstructure/shouldBeScoring", !inTrenchDangerZone()
        && isInScoringZone()
        && HubTracker.isActive(DriverStation.getAlliance().get(), HubTracker.getCurrentShift().get()));

        //Are we in the scoring zone and is the hub active
        return !inTrenchDangerZone()
        && isInScoringZone()
        && HubTracker.isActive(DriverStation.getAlliance().get(), HubTracker.getCurrentShift().get());
    }

    public boolean shouldFerryLeft(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        Logger.recordOutput("Superstructure/shouldFerryLeft", !inTrenchDangerZone()
        && (isRedAlliance()?
        (pos.getY() <= PoseUtils.flipTranslationAlliance(
            new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
        && pos.getX() >= PoseUtils.flipTranslationAlliance(
            new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters), 0)).getX())
        : (pos.getY() <= FieldConstants.LinesHorizontal.CENTER.in(Meters)
        && pos.getX() <= FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters))
        ));
        return !inTrenchDangerZone()
        && (isRedAlliance()?
        (pos.getY() <= PoseUtils.flipTranslationAlliance(
            new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
        && pos.getX() >= PoseUtils.flipTranslationAlliance(
            new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters), 0)).getX())
        : (pos.getY() <= FieldConstants.LinesHorizontal.CENTER.in(Meters)
        && pos.getX() <= FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters))
        );
    }

    public boolean shouldFerryRight(){
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();
        Logger.recordOutput("Superstructure/shouldFerryRight", !inTrenchDangerZone()
        && (isRedAlliance()?
        (pos.getY() >= PoseUtils.flipTranslationAlliance(
            new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
        && pos.getX() >= PoseUtils.flipTranslationAlliance(
            new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters), 0)).getX())
        : (pos.getY() >= FieldConstants.LinesHorizontal.CENTER.in(Meters)
        && pos.getX() <= FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters))));

        return !inTrenchDangerZone()
        && (isRedAlliance()?
        (pos.getY() >= PoseUtils.flipTranslationAlliance(
            new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
        && pos.getX() >= PoseUtils.flipTranslationAlliance(
            new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters), 0)).getX())
        : (pos.getY() >= FieldConstants.LinesHorizontal.CENTER.in(Meters)
        && pos.getX() <= FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR.in(Meters))
        );
    }

    public boolean inTrenchDangerZone(){
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
}
