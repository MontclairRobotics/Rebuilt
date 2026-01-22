// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  private final Vision vision;

  // Controllers
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);

  private SwerveDriveSimulation driveSimulation = null;
  private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));

  // Subsystems
  public static CommandSwerveDrivetrain drivetrain;
  public static Flywheel flywheel;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        drivetrain = TunerConstants.createDrivetrain();

        break;

      case SIM:
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        drivetrain = TunerConstants.createDrivetrain();
        driveSimulation = drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive;

        break;

      default:
    }

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());
    drivetrain.registerTelemetry(logger::telemeterize);
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
    if (Constants.currentMode != Constants.Mode.SIM) return;
    drivetrain.resetPose(new Pose2d(3, 2, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /** Updates Simulated Arena; to be called from Robot.simulationPeriodic() */
  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    // The pose by maplesim, including collisions with the field.
    // See https://www.chiefdelphi.com/t/simulated-robot-goes-through-walls-with-maplesim/508663.
    Logger.recordOutput(
        "FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
  }
}
