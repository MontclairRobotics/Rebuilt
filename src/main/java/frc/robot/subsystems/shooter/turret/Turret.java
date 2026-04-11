package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.TurretConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.aiming.Aiming.TargetLocation;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseUtils;

public class Turret extends SubsystemBase {

	private final TurretIO io;
	private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
	private final TurretVisualization visualization = new TurretVisualization();

	/** for fudging the turret angle mid-match should it get off */
	private Angle fudgeFactor = Degrees.of(0);
	private final Angle step = Degrees.of(3);

	private int logCounter;
	private final int loopsPerLog;

	private boolean isSpinningAround;
	private boolean hasRecentlyConstrainedAngle;
	private Debouncer spinningAroundDebouncer = new Debouncer(1, DebounceType.kFalling);

	private Translation2d fieldRelativePosition = new Translation2d();
	private Translation2d fieldRelativeVelocity = new Translation2d();

    public Turret(TurretIO io) {
        this.io = io;
		loopsPerLog = RobotContainer.TURRET_DEBUG ? 1 : 5; // faster logging during debug mode, slower otherwise
	}

	@Override
	public void periodic() {
		logCounter++;

		fieldRelativePosition = computeFieldRelativePosition(RobotContainer.drivetrain.getRobotPose());
		fieldRelativeVelocity = computeFieldRelativeVelocity(
			RobotContainer.drivetrain.getRobotPose(),
			RobotContainer.drivetrain.getFieldRelativeSpeeds()
		);

		hasRecentlyConstrainedAngle = false;

		// needs to happen every loop
		io.updateInputs(inputs);
		inputs.fieldRelativeAngle = RobotContainer.turret.toFieldRelativeAngle(inputs.robotRelativeAngle);
		Logger.processInputs("Turret", inputs);

		isSpinningAround = spinningAroundDebouncer.calculate(hasRecentlyConstrainedAngle);

		if(logCounter % loopsPerLog == 0) {
			Logger.recordOutput("Turret/DistanceToHub", getDistanceToHub());
		}

		/** only visualize when in debug mode */
		if(RobotContainer.TURRET_DEBUG || RobotBase.isSimulation()) {
			visualization.update();
			visualization.log();
		}
	}

	public Translation2d getFieldRelativePosition() { return fieldRelativePosition; }
	public Translation2d getFieldRelativeVelocity() { return fieldRelativeVelocity; }
	public boolean isSpinningAround() { return isSpinningAround; }

	public boolean hasJoystickInput() {
		return Math.hypot(RobotContainer.operatorController.getLeftX(), RobotContainer.operatorController.getLeftY()) > 0.2;
	}

	public void stop() {
		io.stop();
	}

	public Angle calculateRobotRelativeAngleManualJoystickAim() {
		double y = RobotContainer.operatorController.getLeftX();
		double x = RobotContainer.operatorController.getLeftY();

		// deadband
		if (Math.hypot(x, y) < 0.2) {
			return getRobotRelativeAngle(); // hold last
		}

		double fieldAngleRad = Math.atan2(y, x);
		return toRobotRelativeAngle(Radians.of(fieldAngleRad));
	}

	public void increaseFudgeFactor() {
		fudgeFactor = fudgeFactor.plus(step);
		io.applyFudgeFactor(fudgeFactor);
	}

	public void decreaseFudgeFactor() {
		fudgeFactor = fudgeFactor.minus(step);
		io.applyFudgeFactor(fudgeFactor);
	}

	public Command increaseFudgeFactorCommand() {
		return Commands.runOnce(() -> increaseFudgeFactor());
	}

	public Command decreaseFudgeFactorCommand() {
		return Commands.runOnce(() -> decreaseFudgeFactor());
	}

	public Command lockForever() {
		return Commands.run(() -> io.stop(), this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
	}

	public boolean atSetpoint() {
		return inputs.isAtSetpoint;
	}

    /**
	 * Handles turret wrapping through the max and min angles
	 * @param angle the angle to be constrained to a possible turret angle, in rotations
	 * @return the new angle, constrained between our min and max angles
	 */
	public Angle constrainAngle(Angle angle) {

		while (angle.in(Rotations) > MAX_ANGLE.in(Rotations)) {
    		angle = angle.minus(Rotations.of(1));
			hasRecentlyConstrainedAngle = true;
		}

		while (angle.in(Rotations) < MIN_ANGLE.in(Rotations)) {
   			angle = angle.plus(Rotations.of(1));
			hasRecentlyConstrainedAngle = true;
		}

		return angle;
	}

    /**
	 * @param fieldRelativeAngle target field relative angle of the turret
	 * @return the corresponding target robot relative angle needed to achieve the stated field relative angle
	 */
	public Angle toRobotRelativeAngle(Angle fieldRelativeAngle) {
		return constrainAngle(fieldRelativeAngle
			.minus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
			.minus(ANGLE_OFFSET));
	}

	/**
	 * @param robotRelativeAngle target robot relative angle of the turret
	 * @return the corresponding field relative angle the turret would point at the specified robot relative angle
	 */
	public Angle toFieldRelativeAngle(Angle robotRelativeAngle) {
		return constrainAngle(robotRelativeAngle
			.plus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
			.plus(ANGLE_OFFSET));
	}

	public void setNeutralMode(NeutralModeValue value) {
		io.setNeutralMode(value);
	}

    public Angle getRobotRelativeAngle() {
        return inputs.robotRelativeAngle;
    }

	public Angle getFieldRelativeAngle() {
		return inputs.fieldRelativeAngle;
	}

	/**
	 *
	 * @return the field relative pose of the center of the turret
	 */
	public Translation2d computeFieldRelativePosition(Pose2d robotPose) {
		Rotation2d robotHeading = robotPose.getRotation();
		Translation2d fieldRelativeOffset = TURRET_OFFSET.rotateBy(robotHeading);
		return robotPose.getTranslation().plus(fieldRelativeOffset);
	}

	/**
	 *
	 * @return the field relative velocity of the center of the turret
	 */
	public Translation2d computeFieldRelativeVelocity(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
		Rotation2d robotHeading = robotPose.getRotation();
		Translation2d fieldRelativeOffset = TURRET_OFFSET.rotateBy(robotHeading);

		Rotation2d offsetAngle = fieldRelativeOffset.getAngle();
		Rotation2d tangentialDirection = offsetAngle.plus(Rotation2d.fromDegrees(90));

		double offsetMagnitude = fieldRelativeOffset.getNorm();
		double tangentialVelocityMagnitude = fieldSpeeds.omegaRadiansPerSecond * offsetMagnitude;
		Translation2d tangentialVelocity = new Translation2d(tangentialVelocityMagnitude, tangentialDirection);

		return new Translation2d(
			fieldSpeeds.vxMetersPerSecond + tangentialVelocity.getX(),
			fieldSpeeds.vyMetersPerSecond + tangentialVelocity.getY()
		);
	}

	/**
	 *
	 * @return the distance from the center of the turret to the center of the hub
	 */
	public Distance getDistanceToHub() {
		Translation2d hublocation = PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION);
      	return Meters.of(fieldRelativePosition.getDistance(hublocation));
	}

	/**
	 *
	 * @param location the point to calculate the distance to
	 * @return the distance from the center of the turret to the specified location
	 */
	public Distance getDistanceToPoint(Translation2d location) {
      	return Meters.of(fieldRelativePosition.getDistance(location));
  	}

	/**
	 * @return the field relative angle to align the turret to in order to point at the hub
	 */
	public Angle getAngleToHub() {
		Translation2d hublocation = PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION);
		Translation2d robotToHub = hublocation.minus(fieldRelativePosition);
		return robotToHub.getAngle().getMeasure();
	}

	/**
	 *
	 * @param point the point to calculate the angle towards
	 * @return the field relative angle to align the turret to in order to point at the specific location
	 */
	public Angle getAngleToPoint(Translation2d point) {
		Translation2d turretToPoint = point.minus(fieldRelativePosition);
		return turretToPoint.getAngle().getMeasure();
	}

	/**
	 *
	 * @param target the target to aim at
	 * @return the robot relative velocity the turret should maintain in order to have a field relative velocity of zero
	 */
	public AngularVelocity calculateTargetVelocity(TargetLocation target) {
    	Translation2d r = target.getLocation().minus(fieldRelativePosition);
    	double distance = r.getNorm();
    	Translation2d rHat = r.div(distance); // unit vector towards target
    	Translation2d rHatPerp = new Translation2d(-rHat.getY(), rHat.getX()); // perpendicular unit vector
    	double tangentialVelocity = fieldRelativeVelocity.dot(rHatPerp);
    	double omega = ((tangentialVelocity / distance) + RobotContainer.drivetrain.getAngularVelocity().in(RadiansPerSecond));

    	return RadiansPerSecond.of(-omega);
	}

	public void setRobotRelativeAngle(Angle angle, AngularVelocity velocity) {
		if(hasJoystickInput()) {
			angle = calculateRobotRelativeAngleManualJoystickAim();
			velocity = RotationsPerSecond.zero();
		}
		io.setRobotRelativeAngle(angle, velocity);
	}

	public void setRobotRelativeAngle(Supplier<Angle> angleSupplier, Supplier<AngularVelocity> angularVelocitySupplier) {
		setRobotRelativeAngle(angleSupplier.get(), angularVelocitySupplier.get());
	}

	public Command stopCommand() {
		return Commands.runOnce(() -> io.stop());
	}

	public Command setVoltageCommand(double voltage) {
		return Commands.runOnce(() -> io.setVoltage(voltage));
	}

	public Command setRobotRelativeAngleCommand(Supplier<Angle> angleSupplier, Supplier<AngularVelocity> velocitySupplier) {
		return Commands.run(() -> setRobotRelativeAngle(angleSupplier.get(), velocitySupplier.get()));
	}
}
