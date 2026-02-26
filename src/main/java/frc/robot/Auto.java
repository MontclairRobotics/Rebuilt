package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auto extends SubsystemBase {
  private char currentPos;
  private double timeToEmptyFuel = 6.0; //TODO: get the actual value
  private double maxFuel = 24.0; //TODO: get the actual value
  private ShuffleboardTab autoTab;
  private GenericEntry autoString;
  private GenericEntry error;
  private GenericEntry alliance;

  private Command autoCommand;

  public Auto() {
    autoTab = Shuffleboard.getTab("Auto");
    autoString = autoTab.add("Auto string","Enter auto string").withSize(2,1).getEntry();
    error = autoTab.add("Error", "No errors").withSize(2,1).withPosition(0,1).getEntry();
    alliance = autoTab.add("Alliance", "Unkown alliance").withSize(2,1).withPosition(0,2).getEntry();
  }

  public boolean isAutoValid(String autoString) {

    if(this.autoString.getString("").equals("Enter auto string")) {
      error.setString("Enter a string into Auto string");
      return false;
    }

    if(autoString.length() < 3) {
      error.setString("Auto length is too short");
      return false;
    }

    if(autoString.length() % 2 == 0) {
      error.setString("One of the paths has an invalid length");
      return false;
    }

    String currentAutoString = autoString.substring(0,3);
    String currentPos = autoString.substring(1,2);
    try {
      AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentAutoString));
    } catch(Exception e) {
      error.setString("Path " + currentAutoString + " doesn\'t exist");
      return false;
    }

    for(int i = 3; i < autoString.length(); i += 2) {
      currentAutoString = currentPos + autoString.substring(i, i + 2);
      try {
        AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentAutoString));
      } catch(Exception e) {
        error.setString("Path " + currentAutoString + " doesn\'t exist");
        return false;
      }
      currentPos = autoString.substring(i, i + 1);
    }

    error.setString("No errors");
    return true;
  }

  public Command buildAuto(String autoString) {
    SequentialCommandGroup autoCommand = new SequentialCommandGroup();

    if (!isAutoValid(autoString)) {
      try {
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("LL0"));
        return autoCommand;
      } catch(Exception e) {

      }
    }

    SequentialCommandGroup followPathCommands = new SequentialCommandGroup();

    currentPos = autoString.charAt(1);
    try {
      followPathCommands.addCommands(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3)))
      );
    } catch(Exception e) {
      error.setString("Path issue");
    }

    for (int i = 3; i < autoString.length(); i += 2) {
      try {
        followPathCommands.addCommands(
          AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentPos + autoString.substring(i, i + 2)))
        );
      } catch(Exception e) {
        error.setString("Path issue");
      }
      currentPos = autoString.charAt(i);
      currentPos = autoString.charAt(i);
    }

    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      autoCommand.addCommands(
        Commands.parallel(
          Commands.runOnce(
            () -> {try {
              RobotContainer.drivetrain.resetPose(
                new Pose2d(PathPlannerPath.fromPathFile(autoString.substring(0,3)).getPoint(0).position,
                PathPlannerPath.fromPathFile(autoString.substring(0,3)).getInitialHeading())
              );
            } catch (FileVersionException | IOException | ParseException e) {
              e.printStackTrace();
            }}
          ),
          followPathCommands,
          RobotContainer.rollers.intakeCommand()
        )
      );
    } else {
      autoCommand.addCommands(
        Commands.parallel(
          Commands.runOnce(
            () -> {try {
              RobotContainer.drivetrain.resetPose(
                new Pose2d(PathPlannerPath.fromPathFile(autoString.substring(0,3)).flipPath().getPoint(0).position,
                PathPlannerPath.fromPathFile(autoString.substring(0,3)).flipPath().getInitialHeading())
              );
            } catch (FileVersionException | IOException | ParseException e) {
              e.printStackTrace();
            }}
          ),
          followPathCommands,
          RobotContainer.rollers.intakeCommand()
        )
      );
    }

    return autoCommand;
  }

  public void createAuto(String autoString) {
    autoCommand = buildAuto(autoString);
  }

  public Command getAutoCommand() {
    return autoCommand;
  }


  public void periodic() {
    alliance.setString(DriverStation.getAlliance().get().toString());
    createAuto(autoString.getString(""));
  }
}
