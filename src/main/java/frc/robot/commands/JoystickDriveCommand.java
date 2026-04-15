package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain.ConfigurationMode;

public class JoystickDriveCommand extends Command {

	private final CommandSwerveDrivetrain drivetrain;

	private final DoubleSupplier xVelocitySupplier; // forwards velocity input, away from driver
	private final DoubleSupplier yVelocitySupplier; // strafe velocity input, left/right relative to driver
	private final DoubleSupplier omegaVelocitySupplier; // angular velocity input

	private DriveMode currentDriveMode = DriveMode.NORMAL;

	public JoystickDriveCommand() {
		this.drivetrain = RobotContainer.drivetrain;
		this.xVelocitySupplier = () -> drivetrain.getForwardVelocityFromController();
		this.yVelocitySupplier = () -> drivetrain.getStrafeVelocityFromController();
		this.omegaVelocitySupplier = () -> drivetrain.getOmegaVelocityFromController();

		RobotContainer.xModeTrigger
			.onTrue(updateDriveMode(DriveMode.XMODE))
			.onFalse(updateDriveMode(DriveMode.NORMAL));

		RobotContainer.turboTrigger
			.onTrue(updateDriveMode(DriveMode.TURBO))
			.onFalse(updateDriveMode(DriveMode.NORMAL));

		RobotContainer.precisionTrigger
			.onTrue(updateDriveMode(DriveMode.PRECISION))
			.onFalse(updateDriveMode(DriveMode.NORMAL));

		addRequirements(drivetrain);
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

				drivetrain.isLimitingAcceleration = false;
				drivetrain.swapConfigurationModeTo(ConfigurationMode.NORMAL);
				drivetrain.drive(
					xVelocitySupplier.getAsDouble(),
					yVelocitySupplier.getAsDouble(),
					omegaVelocitySupplier.getAsDouble(),
					drivetrain.fieldRelative
				);

				break;

			case TURBO:

				drivetrain.isLimitingAcceleration = false;
				drivetrain.swapConfigurationModeTo(ConfigurationMode.TURBO);
				drivetrain.drive(
					xVelocitySupplier.getAsDouble(),
					yVelocitySupplier.getAsDouble(),
					omegaVelocitySupplier.getAsDouble(),
					drivetrain.fieldRelative
				);

				break;

			case PRECISION:

				drivetrain.isLimitingAcceleration = true;
				drivetrain.swapConfigurationModeTo(ConfigurationMode.PRECISION);
				drivetrain.drive(
					xVelocitySupplier.getAsDouble(),
					yVelocitySupplier.getAsDouble(),
					omegaVelocitySupplier.getAsDouble(),
					drivetrain.fieldRelative
				);

				break;

			case XMODE:

				drivetrain.isLimitingAcceleration = false;
				drivetrain.swapConfigurationModeTo(ConfigurationMode.NORMAL);
				drivetrain.enableXMode();

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
		PRECISION,
		TURBO,
		XMODE
    }

}
