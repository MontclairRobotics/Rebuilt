package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.Constants.CURRENT_MODE;
import static frc.robot.subsystems.shooter.aiming.Aiming.TargetLocation.FERRY_LEFT;
import static frc.robot.subsystems.shooter.aiming.Aiming.TargetLocation.FERRY_RIGHT;
import static frc.robot.subsystems.shooter.aiming.Aiming.TargetLocation.HUB;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceManager;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LeftTrench;
import frc.robot.util.FieldConstants.LinesVertical;
import frc.robot.util.HubTracker;
import frc.robot.util.PoseUtils;

public class Superstructure extends SubsystemBase {

	private Shooter shooter;

	public Superstructure(Shooter shooter) {
		this.shooter = shooter;
		if(CURRENT_MODE == Mode.SIM) {
			scoringModeTrigger.whileTrue(
				shooter.setSimParameters(
					() -> RobotContainer.aiming.calculateSimShot(
						HUB, shooter.withConstantVelocity, shooter.whileMoving)
				));
			ferryLeftTrigger.whileTrue(
				shooter.setSimParameters(
					() -> RobotContainer.aiming.calculateSimShot(
						FERRY_LEFT, shooter.withConstantVelocity, shooter.whileMoving)
				));
			ferryRightTrigger.whileTrue(
				shooter.setSimParameters(
					() -> RobotContainer.aiming.calculateSimShot(
						FERRY_RIGHT, shooter.withConstantVelocity, shooter.whileMoving)
				));
			shouldStowHoodTrigger.whileTrue(
				shooter.stowCommand());
		} else {
			scoringModeTrigger.whileTrue(
				shooter.setParameters(
					() -> RobotContainer.aiming.calculateShot(HUB, shooter.withConstantVelocity, shooter.whileMoving)
				));
			ferryLeftTrigger.whileTrue(
				shooter.setParameters(
					() -> RobotContainer.aiming.calculateShot(FERRY_LEFT, shooter.withConstantVelocity, shooter.whileMoving)
				));
			ferryRightTrigger.whileTrue(
				shooter.setParameters(
					() -> RobotContainer.aiming.calculateShot(FERRY_RIGHT, shooter.withConstantVelocity, shooter.whileMoving)
				));
			shouldStowHoodTrigger.whileTrue(
				shooter.stowCommand());
		}

	}

	public final Trigger scoringModeTrigger =
			new Trigger(() -> DriverStation.isTeleopEnabled() && shouldBeScoring());

	public final Trigger ferryLeftTrigger =
			new Trigger(() -> DriverStation.isTeleopEnabled() && shouldFerryLeft());

	public final Trigger ferryRightTrigger =
			new Trigger(() -> DriverStation.isTeleopEnabled() && shouldFerryRight());

	public final Trigger shouldStowHoodTrigger =
			new Trigger(() -> DriverStation.isTeleopEnabled() && shouldStowHood());

	@Override
	public void periodic() {
		Logger.recordOutput("Superstructure/isallianceknown", AllianceManager.isAllianceKnown());
		Logger.recordOutput("Superstructure/currentshiftempty", HubTracker.getCurrentShift().isEmpty());
		Logger.recordOutput("Superstructure/shouldBeScoring", shouldBeScoring());
		Logger.recordOutput("Superstructure/shouldFerryLeft", shouldFerryLeft());
		Logger.recordOutput("Superstructure/shouldFerryRight", shouldFerryRight());
		Logger.recordOutput("Superstructure/inTrenchDangerZone", shouldStowHood());
		updateTrenchZonesVeloBased();
	};

	public void updateTrenchZonesVeloBased(){
		//Updates width of zone based on robot velocity
		Distance dynamicTrenchDangerZoneWidth = Meters.of(0.15 + Math.abs(RobotContainer.drivetrain.getFieldRelativeVelocity().getX())* HoodConstants.HOOD_LOWER_TIME);
		FieldConstants.Zones.TRENCH_DANGER_ZONES = new Translation2d[][]{
			// near right trench
            new Translation2d[] {
                new Translation2d(LinesVertical.HUB_CENTER.minus(dynamicTrenchDangerZoneWidth), Meters.zero()),
                new Translation2d(LinesVertical.HUB_CENTER.plus(dynamicTrenchDangerZoneWidth), FieldConstants.RightTrench.WIDTH)
            },

			// near left trench
            new Translation2d[] {
                new Translation2d(LinesVertical.HUB_CENTER.minus(dynamicTrenchDangerZoneWidth), FieldConstants.FIELD_WIDTH.minus(LeftTrench.WIDTH)),
                new Translation2d(LinesVertical.HUB_CENTER.plus(dynamicTrenchDangerZoneWidth), FieldConstants.FIELD_WIDTH)
            },

			// far right trench
            new Translation2d[] {
                new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(dynamicTrenchDangerZoneWidth), Meters.zero()),
                new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(dynamicTrenchDangerZoneWidth), FieldConstants.RightTrench.WIDTH)
            },

			// far left trench
            new Translation2d[] {
                new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(dynamicTrenchDangerZoneWidth), FieldConstants.FIELD_WIDTH.minus(LeftTrench.WIDTH)),
                new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(dynamicTrenchDangerZoneWidth), FieldConstants.FIELD_WIDTH)
            }
        };
	}

    public boolean isRedAlliance() {
		return AllianceManager.isRed();
    }

    public static boolean isInScoringZone() {
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();

        return
			!shouldStowHood()
			&&
			(AllianceManager.isRed() ?
				pos.getX() >= PoseUtils.flipTranslationAlliance(new Translation2d(FieldConstants.LinesVertical.STARTING.in(Meters), 0)).getX()
        		:
				pos.getX() <= FieldConstants.LinesVertical.STARTING.in(Meters)
			);
    }

    public boolean shouldBeScoring() {
		//for now
		// if(!AllianceManager.isAllianceKnown() || (HubTracker.getCurrentShift().isEmpty())) return false;

		if(!AllianceManager.isAllianceKnown()) return false;
		return
			!shouldStowHood()
			&& isInScoringZone();
        //Are we in the scoring zone and is the hub active
        // return
		// 	!inTrenchDangerZone()
        // 	&& isInScoringZone()
        // 	&& HubTracker.isActive(DriverStation.getAlliance().get(), HubTracker.getCurrentShift().get());
	}

    public boolean shouldFerryLeft() {
		if(!AllianceManager.isAllianceKnown()) return false;
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();

        return
			!shouldStowHood()
        	&& (AllianceManager.isRed() ?
				(
					pos.getY() <= PoseUtils.flipTranslationAlliance(new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
					&& pos.getX() <= PoseUtils.flipTranslationAlliance(new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters), 0)).getX()
				)
				:
				(
					pos.getY() >= FieldConstants.LinesHorizontal.CENTER.in(Meters)
					&& pos.getX() >= FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters)
				)
        	);
    }

    public boolean shouldFerryRight() {
		if(!AllianceManager.isAllianceKnown()) return false;
        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();

        return
			!shouldStowHood()
       		&&
			(AllianceManager.isRed() ?
        		(
					pos.getY() >= PoseUtils.flipTranslationAlliance(new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
        			&& pos.getX() <= PoseUtils.flipTranslationAlliance(new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters), 0)).getX()
				)
        		:
				(
					pos.getY() <= FieldConstants.LinesHorizontal.CENTER.in(Meters)
        			&& pos.getX() >= FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters)
				)
        	);
    }

    public static boolean shouldStowHood() {
		if(!AllianceManager.isAllianceKnown()) return false;
        return inTrenchDangerZone();
    }


	// whether we are in the zone to apply trench lock
	public static boolean inTrenchZone() {
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

	public static boolean inTrenchDangerZone() {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
        for (Translation2d[] zone : FieldConstants.Zones.TRENCH_DANGER_ZONES) {
            if (robotPose.getX() >= zone[0].getX()
                    && robotPose.getX() <= zone[1].getX()
                    && robotPose.getY() >= zone[0].getY()
                    && robotPose.getY() <= zone[1].getY()) {
                return true;
            }
        }
        return false;
	}

	public double turretToTrenchDistance() {
		return Math.abs(FieldConstants.LinesVertical.HUB_CENTER.in(Meters) - shooter.getFieldRelativePosition().getX());
	}

	// whether we are in the zone to apply bump lock
	public static boolean inBumpZone() {
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
	public static boolean movingIntoObstacle() {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		boolean movingIntoObstacleOnBlue =
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

		if(AllianceManager.isRed()) return !movingIntoObstacleOnBlue;
		return movingIntoObstacleOnBlue;
	}
}
