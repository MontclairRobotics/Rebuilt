package frc.robot.subsystems.shooter.aiming;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.aiming.AimingConstants.ShootingParameters;
import frc.robot.subsystems.shooter.aiming.AimingConstants.ShotSettings;
import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShootingParameters;
import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShotSettings;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseUtils;

public class Aiming {
	
	private final Turret turret;

	public Aiming() {
		this.turret = RobotContainer.turret;
		TargetLocation.HUB.setLocation(PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION));
	}

	public ShootingParameters calculateShot(TargetLocation target, boolean withConstantVelocity, boolean whileMoving) {

		Translation2d targetLocation = target.getLocation();
		InterpolatingTreeMap<Double, ShotSettings> map;

		switch(target) {
			case HUB:
				map = withConstantVelocity ? AimingConstants.REAL_CONSTANT_VELOCITY_MAP : AimingConstants.REAL_MAP;
				break;
			case FERRY_LEFT:
				map = withConstantVelocity ? AimingConstants.REAL_CONSTANT_VELOCITY_FERRY_MAP : AimingConstants.REAL_FERRY_MAP;
				break;
			case FERRY_RIGHT:
				map = withConstantVelocity ? AimingConstants.REAL_CONSTANT_VELOCITY_FERRY_MAP : AimingConstants.REAL_FERRY_MAP;
				break;	
			default: 
				map = AimingConstants.REAL_MAP;
		}

		Angle robotRelativeTurretAngle;
		Angle hoodAngle;
		AngularVelocity flywheelVelocity;

		Translation2d turretVelocity = turret.getFieldRelativeVelocity();
		Translation2d futureTurretPosition = turret.getFieldRelativePosition()
			.plus(turretVelocity.times(AimingConstants.LATENCY));

		Translation2d displacementToTarget = targetLocation.minus(futureTurretPosition);
		double realDistanceToTarget = displacementToTarget.getNorm();

		Translation2d virtualTarget = targetLocation;
		double virtualDistance = realDistanceToTarget;
		double estimatedTOF = map.get(realDistanceToTarget).timeOfFlight().in(Seconds);

		if(whileMoving) {
			for(int i=0; i<5; i++) {
				virtualTarget = targetLocation.minus(turretVelocity.times(estimatedTOF));
				virtualDistance = virtualTarget.minus(futureTurretPosition).getNorm();
				double newTOF = map.get(virtualDistance).timeOfFlight().in(Seconds);
	
				if(Math.abs(newTOF - estimatedTOF) < 0.02) break;
				estimatedTOF = newTOF;
			}
		}

		Translation2d aimingVector = virtualTarget.minus(futureTurretPosition);
		robotRelativeTurretAngle = turret.toRobotRelativeAngle(Rotations.of(aimingVector.getAngle().getRotations()));
		hoodAngle = map.get(virtualDistance).angle();
		flywheelVelocity = map.get(virtualDistance).flywheelVelocity();

		return new ShootingParameters(robotRelativeTurretAngle, hoodAngle, flywheelVelocity);
	}

	public SimShootingParameters calculateSimShot(TargetLocation target, boolean withConstantVelocity, boolean whileMoving) {

		Translation2d targetLocation = target.getLocation();
		InterpolatingTreeMap<Double, SimShotSettings> map;

		switch(target) {
			case HUB:
				map = withConstantVelocity ? AimingConstants.SIM_CONSTANT_VELOCITY_MAP : AimingConstants.SIM_MAP;
				break;
			case FERRY_LEFT:
				map = withConstantVelocity ? AimingConstants.SIM_CONSTANT_VELOCITY_FERRY_MAP : AimingConstants.SIM_FERRY_MAP;
				break;
			case FERRY_RIGHT:
				map = withConstantVelocity ? AimingConstants.SIM_CONSTANT_VELOCITY_FERRY_MAP : AimingConstants.SIM_FERRY_MAP;
				break;	
			default: 
				map = AimingConstants.SIM_MAP;
		}

		Angle robotRelativeTurretAngle;
		Angle hoodAngle;
		LinearVelocity exitVelocity;

		Translation2d turretVelocity = turret.getFieldRelativeVelocity();
		Translation2d futureTurretPosition = turret.getFieldRelativePosition()
			.plus(turretVelocity.times(AimingConstants.LATENCY));

		Translation2d displacementToTarget = targetLocation.minus(futureTurretPosition);
		double realDistanceToTarget = displacementToTarget.getNorm();

		Translation2d virtualTarget = targetLocation;
		double virtualDistance = realDistanceToTarget;
		double estimatedTOF = map.get(realDistanceToTarget).timeOfFlight().in(Seconds);

		if(whileMoving) {
			for(int i=0; i<5; i++) {
				virtualTarget = targetLocation.minus(turretVelocity.times(estimatedTOF));
				virtualDistance = virtualTarget.minus(futureTurretPosition).getNorm();
				double newTOF = map.get(virtualDistance).timeOfFlight().in(Seconds);
	
				if(Math.abs(newTOF - estimatedTOF) < 0.02) break;
				estimatedTOF = newTOF;
			}
		}

		Translation2d aimingVector = virtualTarget.minus(futureTurretPosition);
		robotRelativeTurretAngle = turret.toRobotRelativeAngle(Rotations.of(aimingVector.getAngle().getRotations()));
		hoodAngle = map.get(virtualDistance).angle();
		exitVelocity = map.get(virtualDistance).exitVelocity();

		return new SimShootingParameters(robotRelativeTurretAngle, hoodAngle, exitVelocity);
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
