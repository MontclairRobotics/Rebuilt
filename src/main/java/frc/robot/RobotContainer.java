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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickDriveCommand;
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
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.spindexer.indexer.Indexer;
import frc.robot.subsystems.shooter.spindexer.indexer.IndexerIOSim;
import frc.robot.subsystems.shooter.spindexer.serializer.Serializer;
import frc.robot.subsystems.shooter.spindexer.serializer.SerializerIOSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;
import frc.robot.util.sim.FuelSim;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;


import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShootingParameters;

public class RobotContainer {

	// private final SendableChooser<Command> autoChooser;

	// Controllers
	public static CommandPS5Controller controller = new CommandPS5Controller(0);

	// Subsystems
	public static Vision vision;
	public static CommandSwerveDrivetrain drivetrain;

	// shooter
	public static Shooter shooter;
	public static Turret turret;
	public static Flywheel flywheel;
	public static Hood hood;

	public static Spindexer spindexer;
	public static Serializer serializer;
	public static Indexer indexer;

	// intake
	public static Pivot pivot;
	public static Rollers rollers;
	public static Intake intake;

	public static Superstructure superstructure;
	public static Aiming aiming;

	public static Auto auto;

	public static SimShootingParameters simShootingParameters = new SimShootingParameters(Degrees.zero(), Degrees.zero(), MetersPerSecond.zero());

	private SwerveDriveSimulation driveSimulation;
	private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));
	public static FuelSim fuelSim = new FuelSim("fuel");

	private boolean useConstantVelocityMap = false;
	private boolean shootWhileMoving = false;

	// debug, set to true to increase logging, set to false to increase performance and reduce loop overruns
	public static boolean VISION_DEBUG = false;
	public static boolean TURRET_DEBUG = false;
	public static boolean FLYWHEEL_DEBUG = false;
	public static boolean HOOD_DEBUG  = false;
	public static boolean INDEXER_DEBUG = false;
	public static boolean SERIALIZER_DEBUG = false;
	public static boolean ROLLERS_DEBUG = false;
	public static boolean PIVOT_DEBUG = false;
	public static boolean DRIVETRAIN_DEBUG = false;
	public static boolean SUPERSTRUCTURE_DEBUG = false;

	public double turretFudge = 0;

	public static boolean shouldShootAuto = false;

	public static Trigger shootTrigger = controller.circle().or(() -> shouldShootAuto);

	public RobotContainer() {

		switch (Constants.CURRENT_MODE) {
			case REAL:
				drivetrain = TunerConstants.createDrivetrain();

				hood = new Hood(new HoodIOSim());
				flywheel = new Flywheel(new FlywheelIOSim());
				turret = new Turret(new TurretIOTalonFX());

				serializer = new Serializer(new SerializerIOSim());
				indexer = new Indexer(new IndexerIOSim());
				spindexer = new Spindexer(serializer, indexer);

				shooter = new Shooter(
					hood, flywheel, turret, spindexer,
					useConstantVelocityMap, shootWhileMoving
				);

				superstructure = new Superstructure(shooter);
				aiming = new Aiming(turret);

				pivot = new Pivot(new PivotIOSim());
				rollers = new Rollers(new RollersIOSim());
				intake = new Intake(pivot, rollers);

				break;

			case SIM:
				drivetrain = TunerConstants.createDrivetrain();
				driveSimulation = drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive;

				hood = new Hood(new HoodIOSim());
				flywheel = new Flywheel(new FlywheelIOSim());
				turret = new Turret(new TurretIOSim());

				serializer = new Serializer(new SerializerIOSim());
				indexer = new Indexer(new IndexerIOSim());
				spindexer = new Spindexer(serializer, indexer);

				shooter = new Shooter(
					hood, flywheel, turret, spindexer,
					useConstantVelocityMap, shootWhileMoving
				);

				superstructure = new Superstructure(shooter);
				aiming = new Aiming(turret);

				pivot = new Pivot(new PivotIOSim());
				rollers = new Rollers(new RollersIOSim());
				intake = new Intake(pivot, rollers);

				fuelSim.enableAirResistance();
				fuelSim.start();

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

				auto = new Auto();

				break;

				default:
					vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
		}

		configureBindings();

    	drivetrain.registerTelemetry(logger::telemeterize);
	}


	private void configureBindings() {

		drivetrain.setDefaultCommand(new JoystickDriveCommand(false));
		controller.touchpad().onTrue(drivetrain.zeroGyroCommand());

	}

	public Command getAutonomousCommand() {
		return Commands.none();
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
