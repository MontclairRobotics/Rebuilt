package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auto extends SubsystemBase{
    private String[] pathNames = new String[] {"D0", "D1", "D2", "L0", "L1", "L2", "R0", "L0", "O0", "O1", "O2", "O3", "R0", "R1", "R2"};
    private char[] pos = new char[] {'L', 'R', 'C', 'O', 'D'};
    private char currentPos;
    private String autoString = "";
    private ArrayList<PathPlannerPath> pathList = new ArrayList<PathPlannerPath>();
    private ArrayList<Pose2d> posesList = new ArrayList<Pose2d>();
    private ArrayList<PathPoint> pathPointsList = new ArrayList<PathPoint>();

    private Command currentCommand = Commands.none();

    public Auto() {

    }

    public boolean isAutoValid(String autoString) {
        if(autoString.length() < 5 || autoString.length() % 2 == 0) {
            return false;
        }

        currentPos = autoString.charAt(0);
        for(int i = 2; i < autoString.length(); i += 2) {
            if(autoString.charAt(i) != currentPos && autoString.charAt(i) != 'C') {
                return false;
            }
        }

        String currentPath;
        boolean pathExists;
        for(int i = 2; i < autoString.length(); i += 2) {
            currentPath = autoString.substring(i, i + 3);
            pathExists = false;
            for(int j = 0; j < pathNames.length; j++) {
                if(pathNames[j] == currentPath) {
                    pathExists = true;
                }
            }

            if(! pathExists) {
                return false;
            }
        }

        return true;
    }

    public Command buildAuto(String autoString) {
        if(! isAutoValid(autoString)) {
            return null;
        }

        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        currentPos = autoString.charAt(1);
        try {
            autoCommand.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(autoString.substring(0,3))));
        } catch(Exception e) {

        }

        for(int i = 3; i < autoString.length(); i += 2) {
            try {
                autoCommand.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentPos + autoString.substring(i, i + 2))));
                currentPos = autoString.charAt(i + 1);
            } catch(Exception e) {

            }
        }

        return autoCommand;
    }
}
