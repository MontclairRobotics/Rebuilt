package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.TurretConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;
import frc.robot.util.Tunable;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The TurretIOHardware class interfaces with the TalonFX motor controller and CANCoders to manage
 * turret movement and sensor readings.
 */
public class Turret extends SubsystemBase {

	public TurretIO io;
	private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
	private TurretVisualization turretVisualization;

	private ProfiledPIDController pidController;

	@SuppressWarnings("unused")
	public Turret(TurretIO io) {
		this.io = io;
		turretVisualization = new TurretVisualization();

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
	public double constrainAngle(double angle) {
		if(angle > MAX_ANGLE.in(Rotations)) {
			return angle - 1;
		} else if (angle < MIN_ANGLE.in(Rotations)) {
			return angle + 1;
		} else {
			return angle;
		}
	}

	/**
	 * @param fieldRelativeAngle target field relative angle of the turret
	 * @return the corresponding target robot relative angle needed to achieve the stated field relative angle
	 */
	public double toRobotRelativeAngle(double fieldRelativeAngle) {
		return fieldRelativeAngle - RobotContainer.drivetrain.getWrappedHeading().getRotations() - OFFSET.in(Rotations);
	}

	/**
	 * @param robotRelativeAngle target robot relative angle of the turret
	 * @return the corresponding field relative angle the turret would point at the specified robot relative angle
	 */
	public double toFieldRelativeAngle(double robotRelativeAngle) {
		return robotRelativeAngle + RobotContainer.drivetrain.getWrappedHeading().getRotations() + OFFSET.in(Rotations);
	}

	/**
	 * @return the target turret velocity for a given robot angular velocity, which is just that angular velocity
	 */
	public double targetTurretVelocity() {
		return RadiansPerSecond.of(
			RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond
		).in(RotationsPerSecond);
	}

	/**
	 * @return the field relative angle to align the turret to in order to point at the hub
	 */
	public double getAngleToHub() {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		Translation2d hublocation = PoseUtils.flipTranslationAlliance(HUB_LOCATION);
		Translation2d robotToHub = hublocation.minus(robotPose.getTranslation());
		return robotToHub.getAngle().getRotations();
	}

	public boolean atGoal() {
		return pidController.atGoal();
	}

	/**
	 * Uses PID Control to set voltage to the turret to achieve a certain robot relative angle
	 * @param angle the robot relative angle to align to
	 */
	public void setRobotRelativeAngle(double angle) {
		pidController.setGoal(
			new TrapezoidProfile.State(
				constrainAngle(angle),
				targetTurretVelocity()
			)
		);

		io.setVoltage(pidController.calculate(io.getRobotRelativeAngle()));
	}

	public void setFieldRelativeAngle(double angle) {
		double robotRelativeAngleSetpoint = toRobotRelativeAngle(angle);
		setRobotRelativeAngle(robotRelativeAngleSetpoint);
	}

	public Command stopCommand() {
		return Commands.runOnce(() -> io.stop());
	}

	public Command setFieldRelativeAngleCommand(double angle) {
		return Commands.run(() -> {
			setFieldRelativeAngle(angle);
		}).until(this::atGoal);
	}

	public Command setFieldRelativeAngleCommand(DoubleSupplier angleSupplier) {
		return Commands.run(() -> {
			setFieldRelativeAngle(angleSupplier.getAsDouble());
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
		turretVisualization.update();
		turretVisualization.log();
	}
}
