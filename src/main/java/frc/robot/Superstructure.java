package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HoodConstants;
import frc.robot.util.AllianceManager;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LeftTrench;
import frc.robot.util.FieldConstants.LinesVertical;
import frc.robot.util.PoseUtils;

public class Superstructure extends SubsystemBase {

	private int logCounter;
	private int loopsPerLog;

	private boolean hasRecentlyResetTrenchZones;

	private static boolean inScoringZone = false;
	private static boolean inLeftFerryZone = false;
	private static boolean inRightFerryZone = false;
	private static boolean inTrenchZone = false;
	private static boolean inTrenchDangerZone = false;

	private Distance defaultWidth = Meters.of(0.8);
	private Translation2d[][] currentTrenchDangerZones;

	@Override
	public void periodic() {

		Translation2d turretPos = RobotContainer.turret.getFieldRelativePosition();
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		double velocityInput = RobotContainer.drivetrain.getForwardVelocityFromController();

		inScoringZone = computeInScoringZone(turretPos);
		inLeftFerryZone = computeInLeftFerryZone(turretPos);
		inRightFerryZone = computeInRightFerryZone(turretPos);
		inTrenchZone = computeInTrenchZone(robotPose);
		inTrenchDangerZone = computeInTrenchDangerZone(turretPos);

		// Trench danger zone update
		logCounter++;
		// Update danger zones only while logging?
		if(logCounter % loopsPerLog == 0 && inTrenchZone && movingIntoObstacle(robotPose, velocityInput)) {
			double fieldRelativeXVelocity = RobotContainer.drivetrain.getFieldRelativeVelocity().getX();
			updateTrenchZonesVeloBased(fieldRelativeXVelocity);
			// We just reset, why record we haven't? 
			// Maybe because in danger zone and moving into, therefore should update again soon?
			hasRecentlyResetTrenchZones = false;
		} else {
			if(!hasRecentlyResetTrenchZones) resetTrenchDangerZones();
			// This should be inside the conditional, so we update only if we actually reset?
			hasRecentlyResetTrenchZones = true;
			// Should recency go false with time?
		}

	};

	public Superstructure() {
		loopsPerLog = RobotContainer.SUPERSTRUCTURE_DEBUG ? 1 : 5;
		hasRecentlyResetTrenchZones = false;

		currentTrenchDangerZones = buildTrenchDangerZones(defaultWidth);
	}

	// Public accessors just return cached values
	public static boolean inScoringZone() { return inScoringZone; }
	public static boolean inLeftFerryZone() { return inLeftFerryZone; }
	public static boolean inRightFerryZone() { return inRightFerryZone; }
	public static boolean inTrenchZone() { return inTrenchZone; }
	public static boolean inTrenchDangerZone() { return inTrenchDangerZone; }

	/**
	 * builds a set of trench danger zones given a specific width
	 */
	private Translation2d[][] buildTrenchDangerZones(Distance width) {
		return new Translation2d[][]{
		// near right trench
		new Translation2d[] {
			new Translation2d(LinesVertical.HUB_CENTER.minus(width), Meters.zero()),
			new Translation2d(LinesVertical.HUB_CENTER.plus(width), FieldConstants.RightTrench.WIDTH)
		},

		// near left trench
		new Translation2d[] {
			new Translation2d(LinesVertical.HUB_CENTER.minus(width), FieldConstants.FIELD_WIDTH.minus(LeftTrench.WIDTH)),
			new Translation2d(LinesVertical.HUB_CENTER.plus(width), FieldConstants.FIELD_WIDTH)
		},

		// far right trench
		new Translation2d[] {
			new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(width), Meters.zero()),
			new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(width), FieldConstants.RightTrench.WIDTH)
		},

		// far left trench
		new Translation2d[] {
			new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(width), FieldConstants.FIELD_WIDTH.minus(LeftTrench.WIDTH)),
			new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(width), FieldConstants.FIELD_WIDTH)
		}
		};
	}

	private void resetTrenchDangerZones() {
		currentTrenchDangerZones = buildTrenchDangerZones(defaultWidth);
	}

	private void updateTrenchZonesVeloBased(double fieldRelativeXVelocity) {
		Distance dynamicWidth = Meters.of(defaultWidth.in(Meters)
			+ Math.abs(fieldRelativeXVelocity)
			* HoodConstants.HOOD_LOWER_TIME);

		currentTrenchDangerZones = buildTrenchDangerZones(dynamicWidth);
	}

	private boolean computeInScoringZone(Translation2d turretPos) {
		if(!AllianceManager.isAllianceKnown()) return false;
		if(AllianceManager.isRed()) {
			return turretPos.getX() >= PoseUtils.flipTranslationAlliance(new Translation2d(FieldConstants.LinesVertical.HUB_CENTER.in(Meters), 0)).getX();
		} else {
			return turretPos.getX() <= FieldConstants.LinesVertical.HUB_CENTER.in(Meters);
		}
	}

	private boolean computeInLeftFerryZone(Translation2d turretPos) {
		if(!AllianceManager.isAllianceKnown()) return false;
		if(AllianceManager.isRed()) {
			return turretPos.getY() <= PoseUtils.flipTranslationAlliance(new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
				&& turretPos.getX() <= PoseUtils.flipTranslationAlliance(new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters), 0)).getX();
		} else {
			return turretPos.getY() >= FieldConstants.LinesHorizontal.CENTER.in(Meters)
				&& turretPos.getX() >= FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters);
		}
    }

  	private boolean computeInRightFerryZone(Translation2d turretPos) {
		if(!AllianceManager.isAllianceKnown()) return false;
		if(AllianceManager.isRed()) {
			return turretPos.getY() >= PoseUtils.flipTranslationAlliance(new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
        		&& turretPos.getX() <= PoseUtils.flipTranslationAlliance(new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters), 0)).getX();
		} else {
			return turretPos.getY() <= FieldConstants.LinesHorizontal.CENTER.in(Meters)
        		&& turretPos.getX() >= FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters);
		}
    }

	private boolean computeInTrenchZone(Pose2d robotPose) {
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

	private boolean computeInTrenchDangerZone(Translation2d turretPos) {
		for (Translation2d[] zone : currentTrenchDangerZones) {
			if (
				turretPos.getX() >= zone[0].getX()
				&& turretPos.getX() <= zone[1].getX()
				&& turretPos.getY() >= zone[0].getY()
				&& turretPos.getY() <= zone[1].getY()
			) {
				return true;
			}
		}
		return false;
	}

	// are we moving INTO the trench?
	public boolean movingIntoObstacle(Pose2d robotPose, double velocityInput) {
		boolean movingIntoObstacleOnBlue =
		(robotPose.getX() < FieldConstants.LinesVertical.ALLIANCE_ZONE.plus(FieldConstants.Hub.WIDTH.div(2)).plus(Constants.BUMPER_WIDTH).in(Meters)
		&& velocityInput > 0 )

		|| (robotPose.getX() > FieldConstants.LinesVertical.ALLIANCE_ZONE.in(Meters)
		&& robotPose.getX() < FieldConstants.LinesVertical.CENTER.in(Meters)
		&& velocityInput < 0)

		|| (robotPose.getX() < FieldConstants.LinesVertical.OPP_ALLIANCE_ZONE.in(Meters)
		&& robotPose.getX() > FieldConstants.LinesVertical.CENTER.in(Meters)
		&& velocityInput > 0)

		|| (robotPose.getX() > FieldConstants.LinesVertical.OPP_ALLIANCE_ZONE.minus(FieldConstants.Hub.WIDTH.div(2)).minus(Constants.BUMPER_WIDTH).in(Meters)
		&& velocityInput < 0);

		if(AllianceManager.isRed()) return !movingIntoObstacleOnBlue;
		return movingIntoObstacleOnBlue;
	}
}
