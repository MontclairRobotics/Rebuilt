package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.TurretConstants.SLOT0_CONFIGS;
import static frc.robot.constants.TurretConstants.MOTION_MAGIC_CONFIGS;
import static frc.robot.constants.HoodConstants.kD;
import static frc.robot.constants.HoodConstants.kS;
import static frc.robot.constants.TurretConstants.ANGLE_OFFSET;
import static frc.robot.constants.TurretConstants.MAX_ANGLE;
import static frc.robot.constants.TurretConstants.MAX_VELOCITY_AT_SETPOINT;
import static frc.robot.constants.TurretConstants.MIN_ANGLE;
import static frc.robot.constants.TurretConstants.TURRET_OFFSET;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseUtils;
import frc.robot.util.tunables.Tunable;

public class Turret extends SubsystemBase {

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
	private final TurretVisualization visualization = new TurretVisualization();

	public double tunnedAngleDegrees = 0;

	private final Tunable kPTunable = new Tunable("Turret/kP", SLOT0_CONFIGS.kP, (value) -> SLOT0_CONFIGS.withKP(value));
	private final Tunable kDTunable = new Tunable("Turret/kD", kD, (value) -> SLOT0_CONFIGS.withKD(value));
	private final Tunable kSTunable = new Tunable("Turret/kS", kS, (value) -> SLOT0_CONFIGS.withKS(value));

	private final Tunable tunableMotionMagicCruiseVelocity = new Tunable("Turret/Motion Magic Cruise Velocity", MOTION_MAGIC_CONFIGS.MotionMagicCruiseVelocity, (value) -> MOTION_MAGIC_CONFIGS.withMotionMagicCruiseVelocity(value));
	private final Tunable tunableMotionMagicAcceleration = new Tunable("Turret/Motion Magic Acceleration", MOTION_MAGIC_CONFIGS.MotionMagicAcceleration, (value) -> MOTION_MAGIC_CONFIGS.withMotionMagicAcceleration(value));
	private final Tunable tunableMotionMagicJerk = new Tunable("Turret/Motion Magic Jerk", MOTION_MAGIC_CONFIGS.MotionMagicJerk, (value) -> MOTION_MAGIC_CONFIGS.withMotionMagicJerk(value));
	private final Tunable tunableMaxVelocityAtSetpoint = new Tunable("Turret/Max Velocity At Setpoint", MAX_VELOCITY_AT_SETPOINT.in(RotationsPerSecond), (value) -> MAX_VELOCITY_AT_SETPOINT = AngularVelocity.ofBaseUnits(value, RotationsPerSecond));

	public final Tunable tunableRobotRelativeTurretAngleDegrees = new Tunable("Turret/Tunable Robot Relative Angle", 0, (value) -> tunnedAngleDegrees = value);

    public Turret(TurretIO io) {
        this.io = io;
    }

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Turret", inputs);
		visualization.update();
		visualization.log();
		// updateTunables();
	}

    /**
	 * Handles turret wrapping through the max and min angles
	 * @param angle the angle to be constrained to a possible turret angle, in rotations
	 * @return the new angle, constrained between our min and max angles
	 */
	public static Angle constrainAngle(Angle angle) {
		while (angle.in(Rotations) > MAX_ANGLE.in(Rotations)) {
    		angle = angle.minus(Rotations.of(1));
		}
		while (angle.in(Rotations) < MIN_ANGLE.in(Rotations)) {
   			angle = angle.plus(Rotations.of(1));
		}
		return angle;
	}

    /**
	 * @param fieldRelativeAngle target field relative angle of the turret
	 * @return the corresponding target robot relative angle needed to achieve the stated field relative angle
	 */
	public static Angle toRobotRelativeAngle(Angle fieldRelativeAngle) {
		return constrainAngle(fieldRelativeAngle
			.minus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
			.minus(ANGLE_OFFSET));
	}

	/**
	 * @param robotRelativeAngle target robot relative angle of the turret
	 * @return the corresponding field relative angle the turret would point at the specified robot relative angle
	 */
	public static Angle toFieldRelativeAngle(Angle robotRelativeAngle) {
		return constrainAngle(robotRelativeAngle
			.plus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
			.plus(ANGLE_OFFSET));
	}

	public void setNeutralMode(NeutralModeValue value) {
		io.setNeutralMode(value);
	}

	// public void updateTunables() {
	// 	if(tunableKP.hasChanged(hashCode())
    //             || tunableKD.hasChanged(hashCode())
    //             || tunableKS.hasChanged(hashCode())) {
    //         io.setGains(tunableKP.get(), tunableKD.get(), tunableKS.get());
    //     }

	// 	if(tunableMotionMagicCruiseVelocity.hasChanged(hashCode())
	// 			|| tunableMotionMagicAcceleration.hasChanged(hashCode())
	// 			|| tunableMotionMagicJerk.hasChanged(hashCode())) {
	// 		io.setMotionMagic(
	// 			tunableMotionMagicCruiseVelocity.get(),
	// 			tunableMotionMagicAcceleration.get(),
	// 			tunableMotionMagicJerk.get()
	// 		);
	// 	}

	// 	if(tunableMaxVelocityAtSetpoint.hasChanged(hashCode())) MAX_VELOCITY_AT_SETPOINT = RotationsPerSecond.of(tunableMaxVelocityAtSetpoint.get());
	// }

	public void applyJoystickInput() {
		double input = MathUtil.copyDirectionPow(MathUtil.applyDeadband(RobotContainer.driverController.getRightY(), 0.1), 1.5);
		double voltage = input * RobotController.getBatteryVoltage();
		io.setVoltage(voltage);
	}

    public Angle getRobotRelativeAngle() {
        return inputs.robotRelativeAngle;
    }

	public Angle getFieldRelativeAngle() {
		return inputs.fieldRelativeAngle;
	}

	public Translation2d getFieldRelativePosition() {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		Rotation2d robotHeading = robotPose.getRotation();
		Translation2d fieldRelativeOffset = TURRET_OFFSET.rotateBy(robotHeading);
		return robotPose.getTranslation().plus(fieldRelativeOffset);
	}

	public Translation2d getFieldRelativeVelocity() {
		ChassisSpeeds fieldSpeeds = RobotContainer.drivetrain.getFieldRelativeSpeeds();
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
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

	public Distance getDistanceToHub() {
		Translation2d hublocation = PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION);
      	return Meters.of(getFieldRelativePosition().getDistance(hublocation));
	}

	public Distance getDistanceToPoint(Translation2d location) {
      	return Meters.of(getFieldRelativePosition().getDistance(location));
  	}

	/**
	 * @return the field relative angle to align the turret to in order to point at the hub
	 */
	public Angle getAngleToHub() {
		Translation2d hublocation = PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION);
		Translation2d robotToHub = hublocation.minus(getFieldRelativePosition());
		Logger.recordOutput("Turret/getAngleToHub", robotToHub.getAngle().getRotations());
		return robotToHub.getAngle().getMeasure();
	}

	public Angle getAngleToPoint(Translation2d point) {
		Translation2d robotPose = getFieldRelativePosition();
		Translation2d robotToPoint = point.minus(robotPose);
		Logger.recordOutput("Turret/getAngletoPoint", robotToPoint.getAngle().getRotations());
		return robotToPoint.getAngle().getMeasure();
	}

	public boolean atSetpoint() {
		return io.isAtSetpoint();
	}

	public void setRobotRelativeAngle(Angle angle) {
		io.setRobotRelativeAngle(angle);
	}

	public void setRobotRelativeAngle(Supplier<Angle> angleSupplier) {
		io.setRobotRelativeAngle(angleSupplier.get());
	}

	public Command joystickControlCommand() {
		return Commands.run(() -> applyJoystickInput(), this);
	}

	public Command stopCommand() {
		return Commands.runOnce(() -> io.stop());
	}

	public Command setRobotRelativeAngleCommand(Supplier<Angle> angleSupplier) {
		return Commands.run(() -> setRobotRelativeAngle(angleSupplier.get()));
	}
}
