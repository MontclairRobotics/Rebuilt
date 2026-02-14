package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import static frc.robot.constants.DriveConstants.MIN_VELOCITY_FOR_TRENCH_AND_BUMP_LOCKS;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.AllianceManager;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LeftTrench;
import frc.robot.util.tunables.TunablePIDController;

public class JoystickDriveCommand extends Command {

	private final CommandSwerveDrivetrain drivetrain;

	// inputs from controller
	private final DoubleSupplier xVelocitySupplier; // forwards velocity input
	private final DoubleSupplier yVelocitySupplier; // strafe velocity input
	private final DoubleSupplier omegaVelocitySupplier; // angular velocity input

	// @AutoLogOutput
	// // private final Trigger shouldTrenchLockTrigger = new Trigger(this::shouldTrenchLock)
	// 	.and(() -> DriverStation.isEnabled()) // resets value when disabled
	// 	.debounce(0.1);

	// @AutoLogOutput
	// // private final Trigger shouldBumpLockTrigger = new Trigger(this::shouldBumpLock)
	// 	.and(() -> DriverStation.isEnabled()) // resets value when disabled
	// 	.debounce(0.1);

	private final TunablePIDController thetaController =
		new TunablePIDController(DriveConstants.ROTATION_CONSTANTS);

	private final TunablePIDController trenchYController =
		new TunablePIDController(DriveConstants.TRENCH_TRANSLATION_CONSTANTS);

	private DriveMode currentDriveMode = DriveMode.NORMAL;

	public JoystickDriveCommand() {
		this.drivetrain = RobotContainer.drivetrain;
		this.xVelocitySupplier = () -> drivetrain.getForwardVelocityFromController();
		this.yVelocitySupplier = () -> drivetrain.getStrafeVelocityFromController();
		this.omegaVelocitySupplier = () -> drivetrain.getOmegaVelocityFromController();

		// shouldTrenchLockTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK))
		// 	.onFalse(updateDriveMode(DriveMode.NORMAL).onlyIf(() -> !RobotContainer.driverController.L1().getAsBoolean()))
		// 	.onFalse(updateDriveMode(DriveMode.SNAKE).onlyIf(() -> RobotContainer.driverController.L1().getAsBoolean()));
		// shouldBumpLockTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK))
		// .onFalse(updateDriveMode(DriveMode.NORMAL).onlyIf(() -> !RobotContainer.driverController.L1().getAsBoolean()))
		// .onFalse(updateDriveMode(DriveMode.SNAKE).onlyIf(() -> RobotContainer.driverController.L1().getAsBoolean()));
		// RobotContainer.driverController.L1().onTrue(updateDriveMode(DriveMode.SNAKE)).onFalse(updateDriveMode(DriveMode.NORMAL));

		addRequirements(drivetrain);
	}

	// private boolean shouldTrenchLock() {
	// 	return Superstructure.inTrenchZone() && drivingBiasedForwards() && Superstructure.movingIntoObstacle();
	// }

	// private boolean shouldBumpLock() {
	// 	return Superstructure.inBumpZone() && drivingBiasedForwards() && Superstructure.movingIntoObstacle() && Math.abs(omegaVelocitySupplier.getAsDouble()) < 1;
	// }

	// if we are moving forwards a little bit and not significantly moving sideways
	private boolean drivingBiasedForwards() {
		return Math.abs(xVelocitySupplier.getAsDouble()) > MIN_VELOCITY_FOR_TRENCH_AND_BUMP_LOCKS.in(MetersPerSecond) && Math.abs(yVelocitySupplier.getAsDouble()) < Math.abs(xVelocitySupplier.getAsDouble());
	}

	// gets the distance to the midline of the trench, used for PID calculations
	private Distance getTrenchY() {
		Pose2d robotPose = drivetrain.getRobotPose();
		Distance trenchY;
		// are we in the left side of the field
		if(robotPose.getMeasureY().gte(FieldConstants.FIELD_WIDTH.div(2))) {
			trenchY = FieldConstants.LinesHorizontal.LEFT_TRENCH_OPEN_END.plus(LeftTrench.OPENING_WIDTH.div(2.0));
		} else {
			trenchY =  FieldConstants.RightTrench.OPENING_WIDTH.div(2.0);
		}

		return trenchY;
	}

	// how much to adjust the PID based on how close we are to the center of the trench (x value)
	private double getTrenchYAdjustFactor() {
		Pose2d robotPose = drivetrain.getRobotPose();
		if(robotPose.getX() < FieldConstants.LinesVertical.CENTER.in(Meters)) {
			return MathUtil.clamp(0.2 + (1 - (Math.abs(robotPose.getX() - FieldConstants.Hub.INNER_CENTER_POINT.getX())
				/ FieldConstants.Zones.TRENCH_ZONE_EXTENSION.in(Meters))), 0, 1);
		} else {
			return MathUtil.clamp(0.2 + (1 - (Math.abs(robotPose.getX() - FieldConstants.Hub.OPP_TOP_CENTER_POINT.getX())
				/ FieldConstants.Zones.TRENCH_ZONE_EXTENSION.in(Meters))), 0, 1);
		}
	}

	private Rotation2d getTrenchLockAngle() {
		if (Math.abs(drivetrain.getWrappedHeading().getDegrees()) < 90) {
            return Rotation2d.kZero;
        } else {
            return Rotation2d.k180deg;
        }
	}

	private Rotation2d getBumpLockAngle() {
		for (int i = -135; i < 180; i += 90) {
            if (Math.abs(MathUtil.inputModulus(drivetrain.getWrappedHeading().getDegrees() - i, -180, 180)) <= 45) {
                return Rotation2d.fromDegrees(i);
            }
        }
        return Rotation2d.kZero;
	}

	private Command updateDriveMode(DriveMode driveMode) {
		return Commands.runOnce(() -> currentDriveMode = driveMode);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		switch (currentDriveMode) {
			case NORMAL:
				drivetrain.drive(
					xVelocitySupplier.getAsDouble(),
					yVelocitySupplier.getAsDouble(),
					omegaVelocitySupplier.getAsDouble(),
					drivetrain.fieldRelative,
					true
				);

				break;

			case TRENCH_LOCK:
				double pidContribution = AllianceManager.isRed() ?
					-(getTrenchYAdjustFactor() * trenchYController.calculate(drivetrain.getRobotPose().getY(), getTrenchY().in(Meters)))
					:
					(getTrenchYAdjustFactor() * trenchYController.calculate(drivetrain.getRobotPose().getY(), getTrenchY().in(Meters)));
				double yVelocity =
				(yVelocitySupplier.getAsDouble()) / 1.3 + pidContribution;
				double rotVelocityTrenchLock = thetaController.calculate(
					drivetrain.getWrappedHeading().getRadians(), getTrenchLockAngle().getRadians()
				);
				drivetrain.drive(
					xVelocitySupplier.getAsDouble(),
					yVelocity,
					rotVelocityTrenchLock,
					drivetrain.fieldRelative,
					true
				);

				break;

			case BUMP_LOCK:
				double rotVelocityBumpLock = thetaController.calculate(
					drivetrain.getWrappedHeading().getRadians(), getBumpLockAngle().getRadians()
				);
				drivetrain.drive(
					xVelocitySupplier.getAsDouble(),
					yVelocitySupplier.getAsDouble(),
					rotVelocityBumpLock,
					drivetrain.fieldRelative,
					true
				);

				break;

			case SNAKE:
				Rotation2d targetAngle = drivetrain.getSnakeDriveAngle();
				drivetrain.setFieldRelativeAngle(targetAngle);
				drivetrain.alignToAngleFieldRelative(false);

				break;

		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}

	private enum DriveMode {
        NORMAL,
        TRENCH_LOCK,
        BUMP_LOCK,
		SNAKE
    }

}
