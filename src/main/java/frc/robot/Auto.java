package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

public class Auto extends SubsystemBase {
  private String[] pathNames =
      new String[] {
        "LD0", "LD1", "LD2", "LL0", "LL1", "LL2", "RO0", "RO1", "RO2", "RO3", "RR0", "RR1", "RR2", "LR0", "RL0"
      };
  private char currentPos;
  private String autoString = "";
  private ArrayList<PathPlannerPath> pathList = new ArrayList<PathPlannerPath>();
  private ArrayList<Pose2d> posesList = new ArrayList<Pose2d>();
  private ArrayList<PathPoint> pathPointsList = new ArrayList<PathPoint>();

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

  //Doesn't work for both alliances and doesn't account for ferrying paths
  public Command buildAuto(String autoString) {
    if (!isAutoValid(autoString)) {
      return null; //TODO: defalt
    }

    SequentialCommandGroup autoCommand = new SequentialCommandGroup();

    currentPos = autoString.charAt(1);
    try {
      if(currentPos != 'D' || autoString.charAt(2) == '0' && currentPos != 'O' || autoString.charAt(2) == '0') {
        autoCommand.addCommands(
          RobotContainer.pivot.stowCommand(),
          Commands.race(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0, 3))),
            RobotContainer.rollers.intakeCommand() //IDK how they have implemented scoring while dirving so this is a placeholder
          )
        );

        if(autoString.charAt(1) == 'L' || autoString.charAt(1) == 'R') {
          autoCommand.addCommands(
            RobotContainer.pivot.unstowCommand(),
            RobotContainer.shooter.indexAndShootCommand(() -> Units.RadiansPerSecond.of(0.0)).withTimeout(6.0) //Change 6 secs to how much time it takes to shoot all fuel plus change the flywheel speed
          );
        }
      } else {
        autoCommand.addCommands(
          RobotContainer.pivot.unstowCommand(),
          Commands.race(PathPlannerPath.fromPathFile(autoString.substring(0, 3)),
            Commands.sequence(
              Commands.waitUntil(/*our x position is < a certain amount*/),
              RobotContainer.shooter.whileScoring() //IDK how they have implemented scoring while dirving so this is a placeholder
            ),
            RobotContainer.rollers.intakeCommand()
          )
        );
      }
    } catch (Exception e) {

    }

    for (int i = 3; i < autoString.length(); i += 2) {
      try {
        autoCommand.addCommands(
          RobotContainer.pivot.stowCommand(),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(i - 1, i + 2)))
        );

        if(autoString.charAt(i) == 'L' || autoString.charAt(i) == 'R') {
          autoCommand.addCommands(
            Robot.pivot.unstowCommand(),
            RobotContainer.shooter.indexAndShootCommand(() -> Units.RadiansPerSecond.of(0.0)).withTimeout(6.0) //Change 6 secs to how much time it takes to shoot all fuel plus change the flywheel speed
          );
        }
        currentPos = autoString.charAt(i);
      } catch (Exception e) {

      }
    }

    return autoCommand;
  }
}
