// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

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

        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "camera0Name", robotToCamera0, drivetrain::getRobotPose),
                new VisionIOPhotonVisionSim(
                    "camera1Name", robotToCamera1, drivetrain::getRobotPose));

        break;

      default:
        vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    }
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());
    drivetrain.registerTelemetry(logger::telemeterize);
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
