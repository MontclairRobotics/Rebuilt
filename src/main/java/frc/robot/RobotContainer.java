// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.JoystickDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.FieldConstants;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;

public class RobotContainer {

	// Controllers
	public static CommandPS5Controller driverController = new CommandPS5Controller(0);
	public static CommandPS5Controller operatorController = new CommandPS5Controller(1);

	private SwerveDriveSimulation driveSimulation = null;
	private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));

	public static final boolean debugMode = true;

	// Subsystems
	public static Vision vision;
	public static CommandSwerveDrivetrain drivetrain;
	public static Flywheel flywheel;
	public static Turret turret;
	public static Hood hood;

	public RobotContainer() {

		switch (Constants.CURRENT_MODE) {
		case REAL:
			flywheel = new Flywheel(new FlywheelIOTalonFX());
			drivetrain = TunerConstants.createDrivetrain();
			turret = new Turret(new TurretIOTalonFX());
			hood = new Hood(new HoodIOTalonFX());
			vision =
				new Vision(
					drivetrain::addVisionMeasurement,
					new VisionIOLimelight(camera0Name, () -> drivetrain.odometryHeading),
					new VisionIOLimelight(camera1Name, () -> drivetrain.odometryHeading));

			break;

		case SIM:
			flywheel = new Flywheel(new FlywheelIOTalonFX());
			drivetrain = TunerConstants.createDrivetrain();
			driveSimulation = drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive;
			turret = new Turret(new TurretIOSim());
			hood = new Hood(new HoodIOSim());
			// vision =
			// 	new Vision(
			// 		drivetrain::addVisionMeasurement,
			// 		new VisionIOPhotonVisionSim(
			// 			"camera0Name", robotToCamera0, drivetrain::getRobotPose),
			// 		new VisionIOPhotonVisionSim(
			// 			"camera1Name", robotToCamera1, drivetrain::getRobotPose));

			//TODO: Fix vision simulation! (It is causing loop overruns and memory issues with advantage kit logging)

			break;

		default:
			vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
		}

		drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));

		configureBindings();

    	drivetrain.registerTelemetry(logger::telemeterize);

		for(Translation2d[] arr: FieldConstants.Zones.BUMP_ZONES) {
			for(Translation2d i : arr)
				System.out.println(i);
		}
	}

	private void configureBindings() {

		// hood.setDefaultCommand(hood.joystickCommand());
		drivetrain.setDefaultCommand(new JoystickDrive());
		// driverController.cross().whileTrue(turret.setPositiveVoltageCommand());
		// driverController.square().whileTrue(turret.setNegativeVoltageCommand());
		// driverController.L1().onTrue(turret.setRobotRelativeAngleCommand(0.25));
		// driverController.L1().whileTrue(new SnakeDriveCommand(drivetrain));
		driverController.R2().whileTrue(turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()));
		driverController.R1().whileTrue(hood.setAngleCommand(() -> hood.getAngleToHub()));

		driverController.triangle().onTrue(hood.setAngleCommand(HoodConstants.MAX_ANGLE));
		driverController.cross().onTrue(hood.setAngleCommand(HoodConstants.MIN_ANGLE));

		// driverController.triangle()
		// 	.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)), false));
		// driverController.square()
		// 	.onTrue(drivetrain.alignToAngleFieldRelativeCommand((Rotation2d.fromDegrees(90)), false));
		// driverController.cross()
		// 	.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(180)), false));
		// driverController.circle()
		// 	.onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(-90), false));

		// zeros gyro
		driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}

	/**
	 * Resets the simulation.
	 *
	 * <p>Borrowed from
	 * https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/RobotContainer.java#L394.
	 */
	public void resetSimulation() {
		if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;
		drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));
		SimulatedArena.getInstance().resetFieldForAuto();
	}

	/** Updates Simulated Arena; to be called from Robot.simulationPeriodic() */
	public void displaySimFieldToAdvantageScope() {
		if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

		SimulatedArena.getInstance().simulationPeriodic();
		// The pose by maplesim, including collisions with the field.
		// See https://www.chiefdelphi.com/t/simulated-robot-goes-through-walls-with-maplesim/508663.
		Logger.recordOutput(
			"FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
	}
}
