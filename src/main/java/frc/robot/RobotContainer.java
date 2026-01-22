// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {

  private final Vision vision;

  // Controllers
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Subsystems
  public static CommandSwerveDrivetrain drivetrain;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision =
        	new Vision(
        		drivetrain::addVisionMeasurement,
        		new VisionIOLimelight(camera0Name, () -> drivetrain.odometryHeading),
        		new VisionIOLimelight(camera1Name, () -> drivetrain.odometryHeading));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision =
        	new Vision(
        		drivetrain::addVisionMeasurement,
        		new VisionIOPhotonVisionSim("camera0Name", robotToCamera0, drivetrain::getRobotPose),
        		new VisionIOPhotonVisionSim("camera1Name", robotToCamera1, drivetrain::getRobotPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision =
        	new Vision(
        		drivetrain::addVisionMeasurement,
        		new VisionIO() {},
        		new VisionIO() {});
        break;
    }

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
