// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.intake.pivot.PivotIOSim;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.RollersIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.aiming.Aiming;
import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShootingParameters;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.spindexer.indexer.Indexer;
import frc.robot.subsystems.shooter.spindexer.indexer.IndexerIOSim;
import frc.robot.subsystems.shooter.spindexer.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.shooter.spindexer.serializer.Serializer;
import frc.robot.subsystems.shooter.spindexer.serializer.SerializerIOSim;
import frc.robot.subsystems.shooter.spindexer.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;
import frc.robot.util.sim.FuelSim;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;


public class RobotContainer {

	// Controllers
	public static CommandPS5Controller driverController = new CommandPS5Controller(0);
	public static CommandPS5Controller operatorController = new CommandPS5Controller(1);

	// Subsystems
	public static Vision vision;
	public static CommandSwerveDrivetrain drivetrain;

	public static Shooter shooter;
	public static Turret turret;
	public static Flywheel flywheel;
	public static Hood hood;
	public static Serializer serializer;
	public static Indexer indexer;
	public static Spindexer spindexer;

	public static Pivot pivot;
	public static Rollers rollers;
	public static Intake intake;

	public static Superstructure superstructure;
	public static Aiming aiming;

	public static SimShootingParameters simShootingParameters = new SimShootingParameters(Degrees.zero(), Degrees.zero(), MetersPerSecond.zero());

	private SwerveDriveSimulation driveSimulation;
	private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));
	public static FuelSim fuelSim = new FuelSim("fuel");

	private boolean withConstantVelocity = false;
	private boolean whileMoving = true;

	public RobotContainer() {
		System.out.println("Constants.CURRENT_MODE: " + Constants.CURRENT_MODE);
		switch (Constants.CURRENT_MODE) {
			case REAL:
				drivetrain = TunerConstants.createDrivetrain();
				flywheel = new Flywheel(new FlywheelIOTalonFX());
				turret = new Turret(new TurretIOTalonFX());
				hood = new Hood(new HoodIOTalonFX());
				serializer = new Serializer(new SerializerIOTalonFX());
				indexer = new Indexer(new IndexerIOTalonFX());
				spindexer = new Spindexer(serializer, indexer);
				// shooter = new Shooter(
				// 	hood, flywheel, turret, spindexer,
				// 	withConstantVelocity, whileMoving
				// );

				// pivot = new Pivot(new PivotIOTalonFX());
				// rollers = new Rollers(new RollersIOTalonFX());
				// intake = new Intake(pivot, rollers);
				// superstructure = new Superstructure(shooter);
				// aiming = new Aiming(turret);
				// vision =
				// 	new Vision(
				// 		drivetrain::addVisionMeasurement,
				// 		new VisionIOLimelight(camera0Name, () -> drivetrain.odometryHeading),
				// 		new VisionIOLimelight(camera1Name, () -> drivetrain.odometryHeading));

				break;

			case SIM:
				drivetrain = TunerConstants.createDrivetrain();
				driveSimulation = drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive;
				flywheel = new Flywheel(new FlywheelIOSim());
				turret = new Turret(new TurretIOSim());
				hood = new Hood(new HoodIOSim());
				serializer = new Serializer(new SerializerIOSim());
				indexer = new Indexer(new IndexerIOSim());
				spindexer = new Spindexer(serializer, indexer);
				shooter = new Shooter(
					hood, flywheel, turret, spindexer,
					withConstantVelocity, whileMoving
				);
				pivot = new Pivot(new PivotIOSim());
				rollers = new Rollers(new RollersIOSim());
				intake = new Intake(pivot, rollers);
				superstructure = new Superstructure(shooter);
				fuelSim.enableAirResistance();
				fuelSim.start();
				aiming = new Aiming(turret);

				fuelSim.registerRobot(
					Constants.BUMPER_WIDTH,
					Constants.BUMPER_WIDTH,
					Inches.of(6),
					() -> drivetrain.getRobotPose(),
					() -> drivetrain.getFieldRelativeSpeeds()
				);
				fuelSim.registerIntake(
					Inches.of(15),
					Inches.of(22),
					Inches.of(-15),
					Inches.of(15),
					shooter::shouldIntake,
					shooter::addBall
				);
				fuelSim.spawnStartingFuel();

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
		// drivetrain.setDefaultCommand(new JoystickDriveCommand(false));
		driverController.circle().whileTrue(spindexer.spinUpCommand()).onFalse(spindexer.spinDownCommand());
		// hood.setDefaultCommand(hood.joystickControlCommand());
		// turret.setDefaultCommand(turret.joystickControlCommand());
		flywheel.setDefaultCommand(flywheel.joystickControlCommand());
		// driverController.L2().whileTrue(spindexer.spinCommand()).onFalse(spindexer.stopCommand());

		// driverController.circle()
		// 	.whileTrue(hood.setAngleCommand(() -> Degrees.of(hood.tunableHoodAngle.get())))
		// 	.onFalse(hood.stopCommand());

		// driverController.square()
		// 	.whileTrue(turret.setRobotRelativeAngleCommand(() -> Degrees.of(hood.tunableHoodAngle.get())))
		// 	.onFalse(turret.stopCommand());

		// driverController.triangle()
		// 	.whileTrue(flywheel.setVelocityCommand(() -> RotationsPerSecond.of(flywheel.tuningFlywheelSpeed.get())))
		// 	.onFalse(flywheel.stopCommand());

		//.times(1).minus(Rotations.of(drivetrain.getWrappedHeading().getRotations()))
		// driverController.circle().whileTrue(Commands.runOnce(() -> fuelSim.launchFuel(MetersPerSecond.of(launchSpeed),Degrees.of(90-hoodAngle),turret.getAngleToHub(),TurretConstants.ORIGIN_TO_TURRET.getMeasureZ())));
		// driverController.triangle()
		// 	.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)), false));
		// driverController.square()
		// 	.onTrue(drivetrain.alignToAngleFieldRelativeCommand((Rotation2d.fromDegrees(90)), false));
		// driverController.cross()
		// 	.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(180)), false));
		// driverController.circle()
		// 	.onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(-90), false));
		// driverController.cross().onTrue(hood.setAngleCommand(HoodConstants.MAX_ANGLE));
		driverController.PS().whileTrue(Commands.runOnce(() -> fuelSim.clearFuel()));
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
		Logger.recordOutput("ferry1",new Translation2d(2,7));
		// Logger.recordOutput("ferry2",new Translation2d(2,1));
		Logger.recordOutput("ferrydistance", turret.getDistanceToPoint(new Translation2d(2,7)));
	}
}
