package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.TurretConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseUtils;
import frc.robot.util.Tunable;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * The TurretIOHardware class interfaces with the TalonFX motor controller and CANCoders to manage
 * turret movement and sensor readings.
 */
public class Turret extends SubsystemBase {

	public TurretIO io;
	private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
	private TurretVisualization visualization;

	private ProfiledPIDController pidController;

	@SuppressWarnings("unused")
	public Turret(TurretIO io) {
		this.io = io;
		visualization = new TurretVisualization();

		pidController = new ProfiledPIDController(
			kP, kI, kD,
			new Constraints(
				MAX_VELOCITY.in(RotationsPerSecond),
				MAX_ACCELERATION.in(RotationsPerSecondPerSecond)
			)
		);

		pidController.disableContinuousInput();
		pidController.setTolerance(
			ANGLE_TOLERANCE.in(Rotations),
			ANGULAR_VELOCITY_TOLERANCE.in(RotationsPerSecond)
		);

		Tunable kpTunable = new Tunable("turret kP", kP, (value) -> pidController.setP(value));
		Tunable kiTunable = new Tunable("turret kI", kP, (value) -> pidController.setI(value));
		Tunable kdTunable = new Tunable("turret kD", kP, (value) -> pidController.setD(value));
	}

	/**
	 * Handles turret wrapping through the max and min angles
	 * @param angle the angle to be constrained to a possible turret angle, in rotations
	 * @return the new angle, constrained between our min and max angles
	 */
	public Angle constrainAngle(Angle angle) {
		if(angle.in(Rotations) > MAX_ANGLE.in(Rotations)) {
			return angle.minus(Rotations.of(1));
		} else if (angle.in(Rotations) < MIN_ANGLE.in(Rotations)) {
			return angle.plus(Rotations.of(1));
		} else {
			return angle;
		}
	}

	/**
	 * @param fieldRelativeAngle target field relative angle of the turret
	 * @return the corresponding target robot relative angle needed to achieve the stated field relative angle
	 */
	public Angle toRobotRelativeAngle(Angle fieldRelativeAngle) {
		return fieldRelativeAngle
			.minus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
			.minus(OFFSET);
	}

	/**
	 * @param robotRelativeAngle target robot relative angle of the turret
	 * @return the corresponding field relative angle the turret would point at the specified robot relative angle
	 */
	public Angle toFieldRelativeAngle(Angle robotRelativeAngle) {
		return robotRelativeAngle
			.plus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
			.plus(OFFSET);
	}

	/**
	 * @return the target turret velocity for a given robot angular velocity, which is just that angular velocity
	 */
	public AngularVelocity targetTurretVelocity() {
		return RadiansPerSecond.of(RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond);
	}

	/**
	 * @return the field relative angle to align the turret to in order to point at the hub
	 */
	public Angle getAngleToHub() {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		Translation2d hublocation = PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION);
		Translation2d robotToHub = hublocation.minus(robotPose.getTranslation());
		Logger.recordOutput("Turret/getAngleToHub", robotToHub.getAngle().getRotations());
		return robotToHub.getAngle().getMeasure();
	}

	public Angle getAngleToPoint(Translation2d point) {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		Translation2d robotToPoint = point.minus(robotPose.getTranslation());
		Logger.recordOutput("Turret/getAngletoPoint", robotToPoint.getAngle().getRotations());
		return robotToPoint.getAngle().getMeasure();
	}

	public boolean atGoal() {
		return pidController.atGoal();
	}

	/**
	 * Uses PID Control to set voltage to the turret to achieve a certain robot relative angle
	 * @param angle the robot relative angle to align to
	 */
	public void setRobotRelativeAngle(Angle angle) {
		pidController.setGoal(
			new TrapezoidProfile.State(
				constrainAngle(angle).in(Rotations),
				targetTurretVelocity().in(RotationsPerSecond)
			)
		);

		Logger.recordOutput("Turret/targetRobotRelativeAngleRotations", angle.in(Rotations));

		io.setVoltage(pidController.calculate(io.getRobotRelativeAngle().in(Rotations)));
	}

	public void setFieldRelativeAngle(Angle angle) {;
		setRobotRelativeAngle(toRobotRelativeAngle(angle));
	}

	public Command stopCommand() {
		return Commands.runOnce(() -> io.stop());
	}

	public Command setFieldRelativeAngleCommand(Angle angle) {
		return Commands.run(() -> {
			setFieldRelativeAngle(angle);
		}).until(this::atGoal);
	}

	public Command setFieldRelativeAngleCommand(Supplier<Angle> angleSupplier) {
		return Commands.run(() -> {
			setFieldRelativeAngle(angleSupplier.get());
			Logger.recordOutput("Turret/targetFieldRelativeAngleRotations", angleSupplier.get().in(Rotations));
		});
	}

	public Translation2d getFieldRelativePosition() {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		Rotation2d robotHeading = robotPose.getRotation();
		Translation2d fieldRelativeOffset = TURRET_OFFSET.rotateBy(robotHeading);
		return robotPose.getTranslation().plus(fieldRelativeOffset);
	}


	public Translation2d getFieldRelativeVelocity() {
		ChassisSpeeds robotSpeeds = RobotContainer.drivetrain.getState().Speeds;
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		Rotation2d robotHeading = robotPose.getRotation();
		Translation2d fieldRelativeOffset = TURRET_OFFSET.rotateBy(robotHeading);

		Rotation2d offsetAngle = fieldRelativeOffset.getAngle();
		Rotation2d tangentialDirection = offsetAngle.plus(Rotation2d.fromDegrees(90));

		double offsetMagnitude = fieldRelativeOffset.getNorm();
		double tangentialVelocityMagnitude = robotSpeeds.omegaRadiansPerSecond * offsetMagnitude;
		Translation2d tangentialVelocity = new Translation2d(tangentialVelocityMagnitude, tangentialDirection);

		return new Translation2d(
			robotSpeeds.vxMetersPerSecond + tangentialVelocity.getX(),
			robotSpeeds.vyMetersPerSecond + tangentialVelocity.getY()
		);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Turret", inputs);
		visualization.update();
		visualization.log();
	}
}
