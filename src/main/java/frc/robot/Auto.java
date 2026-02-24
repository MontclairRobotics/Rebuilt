//TODO: Send why there is an error

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  // private NetworkTable autoTable;
  // private StringTopic autoTopic;
  // private StringPublisher autoPub;
  // private BooleanTopic pushAutoTopic = autoTable.getBooleanTopic("Push Auto");
  // private StringEntry autoPathEntry = autoTopic.getEntry("");
  // private BooleanEntry pushAutoEntry = pushAutoTopic.getEntry(false);

  private Command autoCommand;

  public Auto() {
    autoTab = Shuffleboard.getTab("Auto");
    autoString = autoTab.add("Auto string","Enter auto string").getEntry();
    error = autoTab.add("Error", "No errors").getEntry();
  }

  public boolean isAutoValid(String autoString) {
    if(autoString == "Enter auto string") {
      error.setString("Enter a string into Auto String");
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
    currentPos = autoString.charAt(1);
    try {
      if(currentPos != 'D' || autoString.charAt(2) == '0' && currentPos != 'O' || autoString.charAt(2) == '0') {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          autoCommand.addCommands(
            //RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            Commands.race(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3)))
              //RobotContainer.rollers.intakeCommand()
            )
          );
        } else {
          autoCommand.addCommands(
            // RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            Commands.race(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3)).flipPath())
              //RobotContainer.rollers.intakeCommand()
            )
          );
        }

        if(autoString.charAt(1) == 'L' || autoString.charAt(1) == 'R' && autoString.substring(0,2) != "LR" && autoString.substring(0,2) != "RL") {
          autoCommand.addCommands(
            //RobotContainer.pivot.goToAngleCommand(PivotConstants.MAX_ANGLE),
            //RobotContainer.shooter.indexAndShootCommand(() -> Units.RadiansPerSecond.of(0.0)).withTimeout(timeToEmptyFuel)
          );
        }
      } else {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          autoCommand.addCommands(
            // RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            Commands.race(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3)))
              //Commands.sequence(
                // Commands.waitUntil(() -> Superstructure.isInScoringZone()),
                // RobotContainer.pivot.goToAngleCommand(PivotConstants.MAX_ANGLE)
                //RobotContainer.shooter.whileScoring() //IDK how they have implemented scoring while dirving so this is a placeholder
              //),
              // RobotContainer.rollers.intakeCommand()
            )
          );
        } else {
          autoCommand.addCommands(
           // RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            Commands.race(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3)).flipPath())
              /*Commands.sequence(
                // Commands.waitUntil(() -> Superstructure.isInScoringZone()),
                RobotContainer.pivot.goToAngleCommand(PivotConstants.MAX_ANGLE)
                // RobotContainer.shooter.whileScoring().until(() -> !Superstructure.isInScoringZone()) IDK how they have implemented scoring while dirving so this is a placeholder
              ),
              RobotContainer.rollers.intakeCommand()*/
            )
          );
        }
      }
    } catch (Exception e) {
      error.setString("Unknown error");
    }

    for (int i = 3; i < autoString.length(); i += 2) {
      try {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          autoCommand.addCommands(
            // RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentPos + autoString.substring(i, i + 2)))
          );
        } else {
          autoCommand.addCommands(
            // RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentPos + autoString.substring(i, i + 2)).flipPath())
          );
        }

        currentPos = autoString.charAt(i);
        if(autoString.charAt(i) == 'L' || autoString.charAt(i) == 'R') {
          autoCommand.addCommands(
            // RobotContainer.pivot.goToAngleCommand(PivotConstants.MAX_ANGLE),
            // RobotContainer.shooter.indexAndShootCommand(() -> Units.RadiansPerSecond.of(0.0)).withTimeout(timeToEmptyFuel)
          );
        }
        currentPos = autoString.charAt(i);
      } catch (Exception e) {
        error.setString("Unknown issue");
      }
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
    createAuto(autoString.getString(""));
    // createAuto("LL0");
    //testPub.set(stringEnt.get());
    // if(DriverStation.isDisabled()) {
      // String autoString = autoPathEntry.get("");
      // if(pushAutoEntry.get()) {
        // createAuto(autoString);
      // }
    // }
  }
}
