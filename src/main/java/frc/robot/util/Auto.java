package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {
  public final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
  
  SmartDashboard.putData("Auto Chooser", autoChooser);
  
  NetworkTableInstance instAuto = NetworkTableInstance.getDefault();
  NetworkTable auto = instAuto.getTable("Auto");
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
