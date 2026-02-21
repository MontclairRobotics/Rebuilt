//TODO: Send why there is an error

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auto extends SubsystemBase {
  private char currentPos;
  private double timeToEmptyFuel = 6.0; //TODO: get the actual value
  private double maxFuel = 24.0; //TODO: get the actual value
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable auto = inst.getTable("Auto");
  private StringTopic testTopic = auto.getStringTopic("Auto String");
  private StringPublisher testPub = testTopic.publish();
  StringEntry stringEnt = testTopic.getEntry("");

  // private NetworkTable autoTable;
  // private StringTopic autoTopic;
  // private StringPublisher autoPub;
  // private BooleanTopic pushAutoTopic = autoTable.getBooleanTopic("Push Auto");
  // private StringEntry autoPathEntry = autoTopic.getEntry("");
  // private BooleanEntry pushAutoEntry = pushAutoTopic.getEntry(false);

  private Command autoCommand;
  private StringPublisher errorPub;

  public Auto() {
    // stringEnt.set("");
  }

  public boolean isAutoValid(String autoString) {
    if(autoString.length() < 3) {
      // errorPub.set("Auto length is too short");
    }

    if(autoString.length() % 2 == 0) {
      // errorPub.set("One of the paths has too many or few characters");
      return false;
    }

    String currentAutoString = autoString.substring(0,3);
    try {
      AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentAutoString));
    } catch(Exception e) {
      // errorPub.set("Path " + currentAutoString + " doesn\'t exist");
    }

    for(int i = 3; i < autoString.length(); i += 2) {
      try {
        AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentAutoString));
      } catch(Exception e) {
        // errorPub.set("Path " + currentAutoString + "doesn\'t exist");
      }
    }

    return true;
  }

  public Command buildAuto(String autoString) {

    //RR0R1
    SequentialCommandGroup autoCommand = new SequentialCommandGroup();

    if (!isAutoValid(autoString)) {
      try {
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("LL0"));
        return autoCommand;
      } catch(Exception e) {

      }

    }

    //Valid

    currentPos = autoString.charAt(1);
    //'R'

    try {
      if(currentPos != 'D' || autoString.charAt(2) == '0' && currentPos != 'O' || autoString.charAt(2) == '0') {
        //Runs
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          autoCommand.addCommands(
            //RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            Commands.race(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3)))
              //Follow path: RR0
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
      // errorPub.set("Unknown error");
    }

    for (int i = 3; i < autoString.length(); i += 2) {
      try {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          autoCommand.addCommands(
            // RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            //i = 3
            //
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
        // errorPub.set("Unknown issue");
      }
    }

    return autoCommand;
  }

  public void createAuto(String autoString) {
    autoCommand = buildAuto(autoString);
  }

  public Command getAutoCommand(String str) {
    return buildAuto(str);
  }


  public void periodic() {
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
