package frc.robot.subsystems.shooter.aiming;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.TurretConstants.ORIGIN_TO_TURRET;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.aiming.AimingConstants.ShootingParameters;
import frc.robot.subsystems.shooter.aiming.AimingConstants.ShotSettings;
import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShootingParameters;
import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShotSettings;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseUtils;

public class Aiming {

	private static double loopCounter;
	private static final double LOOPS_PER_CALCULATION = 3;
	private static ShootingParameters cachedShot;
	private static SimShootingParameters cachedSimShot;

	public static void initializeTargetLocations() {
		TargetLocation.HUB.setLocation(PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION));
		TargetLocation.FERRY_LEFT.setLocation(PoseUtils.flipTranslationAlliance(FieldConstants.FerryWaypoints.LEFT_FERRYING_POINT));
		TargetLocation.FERRY_RIGHT.setLocation(PoseUtils.flipTranslationAlliance(FieldConstants.FerryWaypoints.RIGHT_FERRYING_POINT));
	}

	public static ShootingParameters calculateShot(TargetLocation target, boolean withConstantVelocity, boolean whileMoving) {

		loopCounter++;

		if(loopCounter % LOOPS_PER_CALCULATION == 0 || cachedShot == null) {

			Shooter.targetLocation = target;
			Translation2d targetLocation = target.getLocation();
			InterpolatingTreeMap<Double, ShotSettings> map;
			InterpolatingDoubleTreeMap tofMap;

			switch(target) {
				case HUB:
					map = AimingConstants.REAL_MAP;
					tofMap = AimingConstants.REAL_TOF_MAP;
					break;
				case FERRY_LEFT:
				case FERRY_RIGHT:
					map = AimingConstants.REAL_FERRY_MAP;
					tofMap = AimingConstants.REAL_FERRY_TOF_MAP;
					break;
				default:
					map = AimingConstants.REAL_MAP;
					tofMap = AimingConstants.REAL_TOF_MAP;
			}

			Angle robotRelativeTurretAngle;
			Angle hoodAngle;
			AngularVelocity flywheelVelocity;

			ChassisSpeeds fieldRelativeSpeeds = RobotContainer.drivetrain.getFieldRelativeSpeeds();
			Translation2d futureRobotPose = RobotContainer.drivetrain.getRobotPose().getTranslation()
				.plus(new Translation2d(
					fieldRelativeSpeeds.vxMetersPerSecond * AimingConstants.LATENCY,
					fieldRelativeSpeeds.vyMetersPerSecond * AimingConstants.LATENCY
				));

			double robotOmega = RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond;
			Rotation2d futureRobotHeading = RobotContainer.drivetrain.getWrappedHeading()
				.plus(new Rotation2d(robotOmega * AimingConstants.LATENCY));

			Translation2d rotatedOffset = ORIGIN_TO_TURRET.toTranslation2d().rotateBy(futureRobotHeading);
			Translation2d futureTurretPosition = futureRobotPose.plus(rotatedOffset);

			Translation2d displacementToTarget = targetLocation.minus(futureTurretPosition);
			double realDistanceToTarget = displacementToTarget.getNorm();

			Translation2d virtualTarget = targetLocation;
			double virtualDistance = realDistanceToTarget;
			double estimatedTOF = tofMap.get(realDistanceToTarget);

			if(whileMoving) {
				for(int i = 0; i < 3; i++) {
					Translation2d robotDisplacementDuringShot = new Translation2d(
						fieldRelativeSpeeds.vxMetersPerSecond * estimatedTOF,
						fieldRelativeSpeeds.vyMetersPerSecond * estimatedTOF
					);

					virtualTarget = targetLocation.minus(robotDisplacementDuringShot);
					virtualDistance = virtualTarget.minus(futureTurretPosition).getNorm();
					double newTOF = tofMap.get(virtualDistance);

					if (Math.abs(newTOF - estimatedTOF) < 0.02) break;
					estimatedTOF = newTOF;
				}
			}

			Translation2d aimingVector = virtualTarget.minus(futureTurretPosition);
			robotRelativeTurretAngle = Turret.toRobotRelativeAngle(Rotations.of(aimingVector.getAngle().getRotations()));
			ShotSettings finalShotSettings = map.get(virtualDistance);
			hoodAngle = finalShotSettings.angle();
			flywheelVelocity = finalShotSettings.flywheelVelocity();

			cachedShot = new ShootingParameters(robotRelativeTurretAngle, hoodAngle, flywheelVelocity, Timer.getFPGATimestamp() + AimingConstants.LATENCY);
		}

		return cachedShot;
	}

	public static SimShootingParameters calculateSimShot(TargetLocation target, boolean withConstantVelocity, boolean whileMoving) {

		loopCounter++;

		if(loopCounter % LOOPS_PER_CALCULATION == 0 || cachedSimShot == null) {

			Shooter.targetLocation = target;
			Translation2d targetLocation = target.getLocation();
			InterpolatingTreeMap<Double, SimShotSettings> map;
			InterpolatingDoubleTreeMap tofMap;

			switch(target) {
				case HUB:
					map = AimingConstants.SIM_MAP;
					tofMap = AimingConstants.SIM_TOF_MAP;
					break;
				case FERRY_LEFT:
				case FERRY_RIGHT:
					map = AimingConstants.SIM_FERRY_MAP;
					tofMap = AimingConstants.SIM_FERRY_TOF_MAP;
					break;
				default:
					map = AimingConstants.SIM_MAP;
					tofMap = AimingConstants.SIM_TOF_MAP;
			}

			Angle robotRelativeTurretAngle;
			Angle hoodAngle;
			LinearVelocity exitVelocity;

			ChassisSpeeds fieldRelativeSpeeds = RobotContainer.drivetrain.getFieldRelativeSpeeds();
			Translation2d futureRobotPose = RobotContainer.drivetrain.getRobotPose().getTranslation()
				.plus(new Translation2d(
					fieldRelativeSpeeds.vxMetersPerSecond * AimingConstants.LATENCY,
					fieldRelativeSpeeds.vyMetersPerSecond * AimingConstants.LATENCY
				));

			double robotOmega = RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond;
			Rotation2d futureRobotHeading = RobotContainer.drivetrain.getWrappedHeading()
				.plus(new Rotation2d(robotOmega * AimingConstants.LATENCY));

			Translation2d rotatedOffset = ORIGIN_TO_TURRET.toTranslation2d().rotateBy(futureRobotHeading);
			Translation2d futureTurretPosition = futureRobotPose.plus(rotatedOffset);

			Translation2d displacementToTarget = targetLocation.minus(futureTurretPosition);
			double realDistanceToTarget = displacementToTarget.getNorm();

			Translation2d virtualTarget = targetLocation;
			double virtualDistance = realDistanceToTarget;
			double estimatedTOF = tofMap.get(realDistanceToTarget);

			if(whileMoving) {
				for(int i = 0; i < 5; i++) {
					Translation2d robotDisplacementDuringShot = new Translation2d(
						fieldRelativeSpeeds.vxMetersPerSecond * estimatedTOF,
						fieldRelativeSpeeds.vyMetersPerSecond * estimatedTOF
					);

					virtualTarget = targetLocation.minus(robotDisplacementDuringShot);
					virtualDistance = virtualTarget.minus(futureTurretPosition).getNorm();
					double newTOF = tofMap.get(virtualDistance);

					if (Math.abs(newTOF - estimatedTOF) < 0.02) break;
					estimatedTOF = newTOF;
				}
			}

			Translation2d aimingVector = virtualTarget.minus(futureTurretPosition);
			robotRelativeTurretAngle = Turret.toRobotRelativeAngle(Rotations.of(aimingVector.getAngle().getRotations()));
			SimShotSettings finalShotSettings = map.get(virtualDistance);
			hoodAngle = finalShotSettings.angle();
			exitVelocity = finalShotSettings.exitVelocity();

			cachedSimShot = new SimShootingParameters(robotRelativeTurretAngle, hoodAngle, exitVelocity);

		}

		return cachedSimShot;
	}

	public enum TargetLocation {
		HUB(FieldConstants.Hub.HUB_LOCATION),
		FERRY_LEFT(FieldConstants.FerryWaypoints.LEFT_FERRYING_POINT),
		FERRY_RIGHT(FieldConstants.FerryWaypoints.RIGHT_FERRYING_POINT);

		private Translation2d location;

		TargetLocation(Translation2d location) {
			this.location = location;
		}

		public Translation2d getLocation() {
			return location;
		}

		public void setLocation(Translation2d location) {
			this.location = location;
		}
	}
}
