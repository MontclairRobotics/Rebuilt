package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import static frc.robot.constants.DriveConstants.MIN_VELOCITY_FOR_TRENCH_AND_BUMP_LOCKS;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LeftTrench;
import frc.robot.util.TunableControls.TunablePIDController;

public class JoystickDrive extends Command {

	private final CommandSwerveDrivetrain drivetrain;

	// inputs from controller
	private final DoubleSupplier xVelocitySupplier; // forwards velocity input
	private final DoubleSupplier yVelocitySupplier; // strafe velocity input
	private final DoubleSupplier omegaVelocitySupplier; // angular velocity input

	@AutoLogOutput
	private final Trigger shouldTrenchLockTrigger = new Trigger(this::shouldTrenchLock).
		and(() -> DriverStation.isEnabled())
		.debounce(0.1);

	@AutoLogOutput
	private final Trigger shouldSoftTrenchLockTrigger = new Trigger(this::shouldSoftTrenchLock)
		.and(shouldTrenchLockTrigger.negate())
		.and(() -> DriverStation.isEnabled())
		.debounce(0.1);

	@AutoLogOutput
	private final Trigger shouldBumpLockTrigger = new Trigger(this::shouldBumpLock)
		.and(() -> DriverStation.isEnabled())
		.debounce(0.1);

	private final TunablePIDController trenchYController =
		new TunablePIDController(DriveConstants.TRENCH_TRANSLATION_CONSTANTS);
	private final TunablePIDController thetaController =
		new TunablePIDController(DriveConstants.ROTATION_CONSTANTS);

	private DriveMode currentDriveMode = DriveMode.NORMAL;
	public JoystickDrive() {
		this.drivetrain = RobotContainer.drivetrain;
		this.xVelocitySupplier = () -> drivetrain.getForwardVelocityFromController();
		this.yVelocitySupplier = () -> drivetrain.getStrafeVelocityFromController();
		this.omegaVelocitySupplier = () -> drivetrain.getOmegaVelocityFromController();

		shouldTrenchLockTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
		shouldBumpLockTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
		shouldSoftTrenchLockTrigger.onTrue(updateDriveMode(DriveMode.SOFT_TRENCH_LOCK));

		// upon becoming false, update drive mode to normal
		shouldTrenchLockTrigger.or(shouldBumpLockTrigger).or(shouldSoftTrenchLockTrigger)
			.onFalse(updateDriveMode(DriveMode.NORMAL));

		addRequirements(drivetrain);
	}

	private boolean inTrenchZone() {
        Pose2d robotPose = drivetrain.getRobotPose();
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

	private boolean inSoftTrenchZone() {
		Pose2d robotPose = drivetrain.getRobotPose();
        for (Translation2d[] zone : FieldConstants.Zones.SOFT_TRENCH_ZONES) {
            if (robotPose.getX() >= zone[0].getX()
                    && robotPose.getX() <= zone[1].getX()
                    && robotPose.getY() >= zone[0].getY()
                    && robotPose.getY() <= zone[1].getY()) {
                return true;
            }
        }
        return false;
	}

	private boolean inBumpZone() {
		Pose2d robotPose = drivetrain.getRobotPose();
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

	private boolean shouldTrenchLock() {
		return inTrenchZone() && drivingBiasedForwards();
	}

	private boolean shouldSoftTrenchLock() {
		return inSoftTrenchZone() && !inTrenchZone() && drivingBiasedForwards();
	}

	private boolean shouldBumpLock() {
		return inBumpZone() && drivingBiasedForwards() && Math.abs(omegaVelocitySupplier.getAsDouble()) < 1;

	}

	private boolean drivingBiasedForwards() {
		// must be driving more forwards than sideways and have non-negligible velocity input
		return Math.abs(xVelocitySupplier.getAsDouble()) > MIN_VELOCITY_FOR_TRENCH_AND_BUMP_LOCKS.in(MetersPerSecond);
		// 	&& yVelocitySupplier.getAsDouble() > MIN_VELOCITY_FOR_TRENCH_AND_BUMP_LOCKS.in(MetersPerSecond)
		// 	&& xVelocitySupplier.getAsDouble() > MIN_VELOCITY_FOR_TRENCH_AND_BUMP_LOCKS.in(MetersPerSecond);
	}

	private Distance getTrenchY() {
		Pose2d robotPose = drivetrain.getRobotPose();
		// are we in the left side of the field
		if(robotPose.getMeasureY().gte(FieldConstants.FIELD_WIDTH.div(2))) {
			return FieldConstants.LinesHorizontal.LEFT_TRENCH_OPEN_END.plus(LeftTrench.OPENING_WIDTH.div(2.0));
		} else {
			return FieldConstants.RightTrench.OPENING_WIDTH.div(2.0);
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
				double yVelocityTrenchLock = yVelocitySupplier.getAsDouble();
				double rotVelocityTrenchLock = thetaController.calculate(
					drivetrain.getWrappedHeading().getRadians(), getTrenchLockAngle().getRadians()
				);
				drivetrain.drive(
					xVelocitySupplier.getAsDouble(),
					yVelocityTrenchLock,
					rotVelocityTrenchLock,
					drivetrain.fieldRelative,
					true
				);

				break;

			case SOFT_TRENCH_LOCK:

				double yVelocitySoftTrenchLock = + yVelocitySupplier.getAsDouble();
				double rotVelocitySoftTrenchLock = thetaController.calculate(
					drivetrain.getWrappedHeading().getRadians(), getTrenchLockAngle().getRadians()
				);
				drivetrain.drive(
					xVelocitySupplier.getAsDouble(),
					yVelocitySoftTrenchLock,
					rotVelocitySoftTrenchLock,
					drivetrain.fieldRelative,
					true
				);

				break;

			case BUMP_LOCK:
				double rotVelocityToDiagonal = thetaController.calculate(
					drivetrain.getWrappedHeading().getRadians(), getBumpLockAngle().getRadians()
				);
				drivetrain.drive(
					xVelocitySupplier.getAsDouble(),
					yVelocitySupplier.getAsDouble(),
					rotVelocityToDiagonal,
					drivetrain.fieldRelative,
					true
				);

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
		SOFT_TRENCH_LOCK,
        BUMP_LOCK
    }

}
