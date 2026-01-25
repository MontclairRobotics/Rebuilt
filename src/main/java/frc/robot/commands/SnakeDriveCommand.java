package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class SnakeDriveCommand extends Command {

	private CommandSwerveDrivetrain drive;

	public SnakeDriveCommand(CommandSwerveDrivetrain drive) {
		this.drive = drive;
		addRequirements(drive);
	}

	@Override
	public void execute() {
		Rotation2d targetAngle = drive.getSnakeDriveAngle();
		drive.setFieldRelativeAngle(targetAngle);
		drive.alignToAngleFieldRelative(false);
	}
}
