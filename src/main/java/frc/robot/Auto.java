package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auto extends SubsystemBase {
  private String[] pathNames =
      new String[] {
        "LD0", "LD1", "LD2", "LL0", "LL1", "LL2", "RO0", "RO1", "RO2", "RO3", "RR0", "RR1", "RR2", "LR0", "RL0"
      };
  private char currentPos;
  private double timeToEmptyFuel = 6.0; //TODO: get the actual value
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable autoTable = inst.getTable("Auto");
  private StringTopic autoTopic = autoTable.getStringTopic("Auto String");
  private BooleanTopic pushAutoTopic = autoTable.getBooleanTopic("Push Auto");
  private StringEntry autoPathEntry = autoTopic.getEntry("");
  private BooleanEntry pushAutoEntry = pushAutoTopic.getEntry(false);
  private Command autoCommand;

  public Auto() {
    //Ex auto: LL0L1D0
  }

  public int calculateMaxScore(String autoString) {
    int estimatedScore = 0;
    if(! isAutoValid(autoString)) {
      return - 1;
    }

    if(autoString.substring(0,2) != "LR" && autoString.substring(0,2) != "RL") {
      estimatedScore += 15; //Assume we collect as much fuel as we can score at for every path + get all fuel in. Change 15 to the actual max fuel storage number
    }

    for(int i = 3; i < autoString.length(); i += 2) {
      if(autoString.substring(i,i + 2) != "LR" && autoString.substring(i,i + 2) != "RL") {
        estimatedScore += 15; //Assume we collect as much fuel as we can score at for every path + get all fuel in. Change 15 to the actual max fuel storage number
      }
    }

    return estimatedScore;
  }

  public boolean isAutoValid(String autoString) {
    if (autoString.length() < 5 || autoString.length() % 2 == 0) {
      return false;
    }

    String currentAutoString = autoString.substring(0,3);
    boolean doesPathExist = false;
    for(int i = 0; i < autoString.length(); i++) {
      if(currentAutoString == pathNames[i]) {
        doesPathExist = true;
      }
    }

    if(! doesPathExist) {
      return false;
    }

    for(int i = 3; i < autoString.length(); i += 2) {
      currentAutoString = currentAutoString.charAt(1) + autoString.substring(i, i + 2);
      doesPathExist = false;
      for(int j = 0; j < pathNames.length; j++) {
        if(currentAutoString == pathNames[j]) {
          doesPathExist = true;
        }
      }

      if(! doesPathExist) {
        return false;
      }
    }

    return true;
  }

  //TODO: Make it work for both alliance
  //TODO: Account for ferrying
  public Command buildAuto(String autoString) {
    if (!isAutoValid(autoString)) {
      return RobotContainer.shooter.indexAndShootCommand(() -> Units.RadiansPerSecond.of(0.0)).withTimeout(timeToEmptyFuel * (8.0 / 24.0)); //TODO: Get max amount of fuel
    }

    SequentialCommandGroup autoCommand = new SequentialCommandGroup();

    currentPos = autoString.charAt(1);
    try {
      if(currentPos != 'D' || autoString.charAt(2) == '0' && currentPos != 'O' || autoString.charAt(2) == '0') {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          autoCommand.addCommands(
            //No pivot, so these commands and objects are standins and don't actually exist
            // RobotContainer.pivot.stowCommand(),
            Commands.race(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3)))
                // RobotContainer.rollers.intakeCommand() //IDK how they have implemented scoring while dirving so this is a placeholder
            )
          );
        } else {
          autoCommand.addCommands(
          // RobotContainer.pivot.stowCommand(),
            Commands.race(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3)).flipPath())
              // RobotContainer.rollers.intakeCommand() //IDK how they have implemented scoring while dirving so this is a placeholder
            )
          );
        }

        if(autoString.charAt(1) == 'L' || autoString.charAt(1) == 'R') {
          autoCommand.addCommands(
            // RobotContainer.pivot.unstowCommand(),
            RobotContainer.shooter.indexAndShootCommand(() -> Units.RadiansPerSecond.of(0.0)).withTimeout(timeToEmptyFuel)
          );
        }
      } else {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          autoCommand.addCommands(
            //RobotContainer.pivot.stowCOmmand(),
            Commands.race(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3))),
              Commands.sequence(
                Commands.waitUntil(() -> Superstructure.isInScoringZone())
                // RobotContainer.pivot.unstowCommand(),
                //RobotContainer.shooter.whileScoring() //IDK how they have implemented scoring while dirving so this is a placeholder
              )
              // RobotContainer.rollers.intakeCommand()
            )
          );
        } else {
          autoCommand.addCommands(
              //RobotContainer.pivot.stowCOmmand(),
              Commands.race(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3)).flipPath()),
                Commands.sequence(
                  Commands.waitUntil(() -> Superstructure.isInScoringZone())
                  // RobotContainer.pivot.unstowCommand(),
                  //RobotContainer.shooter.whileScoring() //IDK how they have implemented scoring while dirving so this is a placeholder
                )
                // RobotContainer.rollers.intakeCommand()
              )
            );
        }
      }
    } catch (Exception e) {

    }

    for (int i = 3; i < autoString.length(); i += 2) {
      try {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          autoCommand.addCommands(
            //RobotContainer.pivot.stowCommand(),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(i - 1, i + 2)))
          );
        } else {
          autoCommand.addCommands(
            //RobotContainer.pivot.stowCommand(),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(i - 1, i + 2)).flipPath())
          );
        }

        if(autoString.charAt(i) == 'L' || autoString.charAt(i) == 'R') {
          autoCommand.addCommands(
            //Robot.pivot.unstowCommand(),
            RobotContainer.shooter.indexAndShootCommand(() -> Units.RadiansPerSecond.of(0.0)).withTimeout(timeToEmptyFuel)
          );
        }
        currentPos = autoString.charAt(i);
      } catch (Exception e) {

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
    if(DriverStation.isDisabled()) {
      String autoString = autoPathEntry.get("");
      SmartDashboard.putNumber("Score", calculateMaxScore(autoString));

      createAuto(autoString);
    }
  }
}
