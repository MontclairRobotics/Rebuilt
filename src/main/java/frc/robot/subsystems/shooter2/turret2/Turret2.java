// package frc.robot.subsystems.shooter2.turret2;

// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.Rotations;
// import static frc.robot.constants.TurretConstants.*;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation2d;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import frc.robot.constants.TurretConstants;
// import frc.robot.util.FieldConstants;
// import frc.robot.util.PoseUtils;

// import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;

// /**
//  * The TurretIOHardware class interfaces with the TalonFX motor controller and CANCoders to manage
//  * turret movement and sensor readings.
//  */
// public class Turret2 extends SubsystemBase {

// 	public final TurretIO2 io;
// 	private final TurretIOInputsAutoLogged2 inputs = new TurretIOInputsAutoLogged2();
// 	private final TurretVisualization2 visualization;

// 	private PIDController pidController;

// 	@SuppressWarnings("unused")
// 	public Turret2(TurretIO2 io) {
// 		this.io = io;
// 		visualization = new TurretVisualization2();

// 		pidController = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
// 		pidController.setTolerance(TurretConstants.ANGLE_TOLERANCE.in(Rotations));
// 	}

// 	/**
// 	 * Handles turret wrapping through the max and min angles
// 	 * @param angle the angle to be constrained to a possible turret angle, in rotations
// 	 * @return the new angle, constrained between our min and max angles
// 	 */
// 	public Angle constrainAngle(Angle angle) {
// 		if(angle.in(Rotations) > MAX_ANGLE.in(Rotations)) {
// 			return angle.minus(Rotations.of(1));
// 		} else if (angle.in(Rotations) < MIN_ANGLE.in(Rotations)) {
// 			return angle.plus(Rotations.of(1));
// 		} else {
// 			return angle;
// 		}
// 	}

// 	/**
// 	 * @param fieldRelativeAngle target field relative angle of the turret
// 	 * @return the corresponding target robot relative angle needed to achieve the stated field relative angle
// 	 */
// 	public Angle toRobotRelativeAngle(Angle fieldRelativeAngle) {
// 		return constrainAngle(fieldRelativeAngle
// 			.minus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
// 			.minus(ANGLE_OFFSET));
// 	}

// 	/**
// 	 * @param robotRelativeAngle target robot relative angle of the turret
// 	 * @return the corresponding field relative angle the turret would point at the specified robot relative angle
// 	 */
// 	public Angle toFieldRelativeAngle(Angle robotRelativeAngle) {
// 		return constrainAngle(robotRelativeAngle
// 			.plus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
// 			.plus(ANGLE_OFFSET));
// 	}

// 	public Angle getRobotRelativeAngle() {
// 		return io.getRobotRelativeAngle();
// 	}

// 	public Angle getFieldRelativeAngle() {
// 		return io.getFieldRelativeAngle();
// 	}

// 	/**
// 	 * @return the target turret velocity for a given robot angular velocity, which is just that angular velocity
// 	 */
// 	public AngularVelocity calculateTargetVelocity() {
// 		// Logger.recordOutput("Turret/Calculated Target Velocity", RadiansPerSecond.of(RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond).in(RotationsPerSecond));
// 		return RadiansPerSecond.of(RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond);
// 	}

// 	public double getDistanceToHub() {
// 	  Translation2d hublocation = PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION);
//       return getFieldRelativePosition().getDistance(hublocation);
//   	}
// 	public double getDistanceToPoint(Translation2d location) {
//       return getFieldRelativePosition().getDistance(location);
//   	}

// 	/**
// 	 * @return the field relative angle to align the turret to in order to point at the hub
// 	 */
// 	public Angle getAngleToHub() {
// 		Translation2d hublocation = PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION);
// 		Translation2d robotToHub = hublocation.minus(getFieldRelativePosition());
// 		Logger.recordOutput("Turret/getAngleToHub", robotToHub.getAngle().getRotations());
// 		return robotToHub.getAngle().getMeasure();
// 	}

// 	public Angle getAngleToPoint(Translation2d point) {
// 		Translation2d robotPose = getFieldRelativePosition();
// 		// Translation2d location = PoseUtils.flipTranslationAlliance(point);
// 		Translation2d robotToPoint = point.minus(robotPose);
// 		Logger.recordOutput("Turret/getAngletoPoint", robotToPoint.getAngle().getRotations());
// 		return robotToPoint.getAngle().getMeasure();
// 	}

// 	public boolean atSetpointForShooting() {
// 		return Math.abs(pidController.getSetpoint() - getRobotRelativeAngle().in(Rotations)) < TurretConstants.ANGLE_TOLERANCE.in(Rotations);
// 	}

// 	/**
// 	 * Uses PID Control to set voltage to the turret to achieve a certain robot relative angle
// 	 * @param angle the robot relative angle to align to
// 	 */
// 	public void setRobotRelativeAngle(Angle angle) {
// 		// Logger.recordOutput("Turret/Target Robot Relative Anlge", angle);
// 		pidController.setSetpoint(constrainAngle(angle).in(Rotations));
// 		io.setVoltage(pidController.calculate(io.getRobotRelativeAngle().in(Rotations)));
// 	}

// 	public void setRobotRelativeAngle(Supplier<Angle> angleSupplier) {
// 		setRobotRelativeAngle(angleSupplier.get());
// 	}

// 	public void setFieldRelativeAngle(Angle angle) {
// 		setRobotRelativeAngle(toRobotRelativeAngle(angle));
// 	}

// 	public Command stopCommand() {
// 		return Commands.runOnce(() -> io.stop());
// 	}

// 	public Command setRobotRelativeAngleCommand(Supplier<Angle> angleSupplier) {
// 		return Commands.run(() -> setRobotRelativeAngle(angleSupplier.get()));
// 	}

// 	public Translation2d getFieldRelativePosition() {
// 		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
// 		Rotation2d robotHeading = robotPose.getRotation();
// 		Translation2d fieldRelativeOffset = TURRET_OFFSET.rotateBy(robotHeading);
// 		return robotPose.getTranslation().plus(fieldRelativeOffset);
// 	}


// 	public Translation2d getFieldRelativeVelocity() {
// 		ChassisSpeeds fieldSpeeds = RobotContainer.drivetrain.getFieldRelativeSpeeds();
// 		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
// 		Rotation2d robotHeading = robotPose.getRotation();
// 		Translation2d fieldRelativeOffset = TURRET_OFFSET.rotateBy(robotHeading);

// 		Rotation2d offsetAngle = fieldRelativeOffset.getAngle();
// 		Rotation2d tangentialDirection = offsetAngle.plus(Rotation2d.fromDegrees(90));

// 		double offsetMagnitude = fieldRelativeOffset.getNorm();
// 		double tangentialVelocityMagnitude = fieldSpeeds.omegaRadiansPerSecond * offsetMagnitude;
// 		Translation2d tangentialVelocity = new Translation2d(tangentialVelocityMagnitude, tangentialDirection);

// 		return new Translation2d(
// 			fieldSpeeds.vxMetersPerSecond + tangentialVelocity.getX(),
// 			fieldSpeeds.vyMetersPerSecond + tangentialVelocity.getY()
// 		);
// 	}

// 	@Override
// 	public void periodic() {
// 		io.updateInputs(inputs);
// 		Logger.processInputs("Turret", inputs);
// 		Logger.recordOutput("Intake pose", new Pose3d(0.2664714, 0, 0.1711706, Rotation3d.kZero));
// 		Logger.recordOutput("Turret distance to hub", getDistanceToHub());
// 		Logger.recordOutput("Turret/Robot Relative Setpoint", constrainAngle(Rotations.of(pidController.getSetpoint())).in(Rotations));
// 		Logger.recordOutput("Turret/Robot Relative Angle", getRobotRelativeAngle().in(Rotations));
// 		Logger.recordOutput("Turret/My Recorded Error", Math.abs(getRobotRelativeAngle().in(Rotations) - pidController.getSetpoint()));
// 		Logger.recordOutput("Turret/Robot Relative Angle to Hub", toRobotRelativeAngle(getAngleToHub()).in(Rotations));
// 		Logger.recordOutput("Turret/At Setpoint Real", RobotContainer.turret.atSetpointForShooting());
// 		Logger.recordOutput("Turret/PID Controller Setpoint", pidController.getSetpoint());
// 		Logger.recordOutput("Turret/Tolerance", pidController.getErrorTolerance());
// 		Logger.recordOutput("Turret/PID Controller Error", pidController.getError());

// 		visualization.update();
// 		visualization.log();
// 	}
// }
