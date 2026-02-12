// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.spindexer.SpindexerIOSim;
import frc.robot.subsystems.shooter.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.FuelSim;
import frc.robot.util.PoseUtils;

public class RobotContainer {

	// Controllers
	public static CommandPS5Controller driverController = new CommandPS5Controller(0);
	public static CommandPS5Controller operatorController = new CommandPS5Controller(1);

	// Subsystems
	public static Vision vision;
	public static CommandSwerveDrivetrain drivetrain;
	public static Flywheel flywheel;
	public static Turret turret;
	public static Hood hood;
	public static Shooter shooter;
	public static Spindexer spindexer;
	public static Superstructure superstructure;

	private SwerveDriveSimulation driveSimulation;
	private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));
	public FuelSim fuelSim = new FuelSim("fuel");

	public RobotContainer() {
			fuelSim.registerRobot(Constants.BUMPER_WIDTH,Constants.BUMPER_WIDTH, Inches.of(6),()->drivetrain.getRobotPose(),()->drivetrain.getState().Speeds);
			fuelSim.registerIntake(Inches.of(15), Inches.of(22), Inches.of(-15), Inches.of(15));
			fuelSim.spawnStartingFuel();
		switch (Constants.CURRENT_MODE) {
		case REAL:
			flywheel = new Flywheel(new FlywheelIOTalonFX());
			drivetrain = TunerConstants.createDrivetrain();
			turret = new Turret(new TurretIOTalonFX());
			hood = new Hood(new HoodIOTalonFX());
			spindexer = new Spindexer(new SpindexerIOTalonFX());
			shooter = new Shooter(hood, flywheel, turret, spindexer);
			superstructure = new Superstructure(shooter);
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
			spindexer = new Spindexer(new SpindexerIOSim());
			shooter = new Shooter(hood, flywheel, turret, spindexer);
			superstructure = new Superstructure(shooter);
			fuelSim.enableAirResistance();
			fuelSim.start();
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
	}

	private void configureBindings() {


		drivetrain.setDefaultCommand(new JoystickDriveCommand());
		driverController.R2().whileTrue(turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub())).onFalse(turret.stopCommand());
		driverController.R1().whileTrue(hood.setAngleCommand(() -> hood.getAngleToHub())).onFalse(hood.stopCommand());
		driverController.circle().whileTrue(Commands.runOnce(()->fuelSim.launchFuel(MetersPerSecond.of(8), (hood.getAngle().times(-1)).plus(Degrees.of(90)), turret.getFieldRelativeAngle(),TurretConstants.ORIGIN_TO_TURRET.getMeasureZ())));
		// driverController.triangle().onTrue(hood.setAngleCommand(HoodConstants.MAX_ANGLE));
		// driverController.cross().onTrue(hood.setAngleCommand(HoodConstants.MIN_ANGLE));

		driverController.triangle()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)), false));
		driverController.square()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand((Rotation2d.fromDegrees(90)), false));
		driverController.cross()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(180)), false));
		// driverController.circle()
		// 	.onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(-90), false));
		driverController.cross().onTrue(hood.setAngleCommand(HoodConstants.MAX_ANGLE));
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
