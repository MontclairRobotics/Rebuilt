// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;

public class RobotContainer {

  // Controllers
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Subsystems
  public static CommandSwerveDrivetrain drivetrain;
  public static Turret turret;

  public RobotContainer() {
    TurretIO turretInput = new TurretIOSim();
    turret = new Turret(turretInput);
    configureBindings();
  }

  private void configureBindings() {
    operatorController.povRight().onTrue(turret.setPositiveVoltageCommand());
    operatorController.povLeft().onTrue(turret.setPositiveVoltageCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
