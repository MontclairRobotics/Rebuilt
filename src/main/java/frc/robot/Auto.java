package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;
import frc.robot.util.AllianceManager;
import frc.robot.util.PoseUtils;

public class Auto extends SubsystemBase {
	private char currentPos;
	private double timeToEmptyFuel = 6.0; //TODO: get the actual value
	private ShuffleboardTab autoTab;
	private GenericEntry autoString;
	private GenericEntry error;
	private GenericEntry alliance;
	private GenericEntry currPos;
	private Field2d field = new Field2d();
	private ArrayList<PathPlannerPath> allPaths = new  ArrayList<PathPlannerPath>();
	private ArrayList<PathPlannerPath> prevPaths = new ArrayList<PathPlannerPath>();
	private Command autoCommand;
	private int maxObjs;

	NetworkTableInstance inst = NetworkTableInstance.getDefault();
	NetworkTable auto = inst.getTable("Auto");

	StringTopic autoTopic = auto.getStringTopic("Auto String");
	StringEntry stringEnt = autoTopic.getEntry("");

	StringTopic feedbackTopic = auto.getStringTopic("Auto Feedback");
	StringPublisher feedbackPub = feedbackTopic.publish();
	StringSubscriber stringSub = autoTopic.subscribe("");

	public Auto() {
		autoTab = Shuffleboard.getTab("Auto");
		autoString = autoTab.add("Auto string", "").withSize(2,1).getEntry();
		error = autoTab.add("Error", "No errors").withSize(2,1).withPosition(0,1).getEntry();
		alliance = autoTab.add("Alliance", "Unkown alliance").withSize(2,1).withPosition(0,2).getEntry();
		currPos = autoTab.add("current position", "").getEntry();
		SmartDashboard.putData("Field", field);
	}

	public static void drawAuto(String auto) {
		int maxObjs = 0;
		Field2d field = new Field2d();
		for(int i = 0; i <= maxObjs; i++) {
		field.getObject("obj" + i).setPoses(new Pose2d());
		}
		maxObjs = 0;

		if(AllianceManager.getAlliance() == DriverStation.Alliance.Blue) {
		try {
			for(int i = 0; i < PathPlannerAuto.getPathGroupFromAutoFile(auto).size(); i++) {
			field.getObject("obj" + i).setPoses(PathPlannerAuto.getPathGroupFromAutoFile(auto).get(i).getPathPoses());
			if(i > maxObjs) {
				maxObjs = i;
			}
			}
		} catch (IOException | ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		}

		if(AllianceManager.getAlliance() == DriverStation.Alliance.Red) {
		try {
			for(int i = 0; i < PathPlannerAuto.getPathGroupFromAutoFile(auto).size(); i++) {
			field.getObject("obj" + i).setPoses(PathPlannerAuto.getPathGroupFromAutoFile(auto).get(i).flipPath().getPathPoses());
			if(i > maxObjs) {
				maxObjs = i;
			}
			}
		} catch (IOException | ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		}
		SmartDashboard.putData("Static field",field);
	}

	public void drawPaths() {
		if(!prevPaths.equals(allPaths)) {
			for(int i = 0; i <= maxObjs; i++) {
				field.getObject("obj" + i).setPoses(new Pose2d());
			}
			maxObjs = 0;
		}

		if(AllianceManager.getAlliance() == DriverStation.Alliance.Blue) {
			for(int i = 0; i < allPaths.size(); i++) {
				field.getObject("obj" + i).setPoses(allPaths.get(i).getPathPoses().toArray(new Pose2d[0]));
				if(i > maxObjs) {
					maxObjs = i;
				}
			}
		}

		if(AllianceManager.getAlliance() == DriverStation.Alliance.Red) {
			for(int i = 0; i < allPaths.size(); i++) {
				field.getObject("obj" + i).setPoses(allPaths.get(i).flipPath().getPathPoses().toArray(new Pose2d[0]));
				if(i > maxObjs) {
					maxObjs = i;
				}
			}
		}

		prevPaths = allPaths;
	}

	public boolean isAutoValid(String autoString) {

		if(this.autoString.getString("").equals("")) {
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

		allPaths = new ArrayList();
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
		currPos.setString("" + currentPos);
		try {
			String pathString = autoString.substring(0, 3);
			PathPlannerPath firstPath = PathPlannerPath.fromPathFile(pathString);

			Optional<Pose2d>  opPose = firstPath.getStartingHolonomicPose();
			Pose2d pose = opPose.isPresent() ? PoseUtils.flipPoseAlliance(opPose.get()) : new Pose2d();

			followPathCommands.addCommands(
				Commands.runOnce(() -> RobotContainer.drivetrain.resetPose(pose), RobotContainer.drivetrain),
				AutoBuilder.followPath(firstPath)
			);
			try {
				allPaths.add(PathPlannerPath.fromPathFile(autoString.substring(0, 3)));
			} catch(Exception e) {
				error.setString("Adding first path to allPaths issue");
			}
		} catch(Exception e) {
			error.setString("First path issue");
		}

		for (int i = 3; i < autoString.length(); i += 2) {
			if(
				(currentPos == 'L' || currentPos == 'R')
				&& currentPos == autoString.charAt(i)
				|| ((autoString.charAt(i+1) == '0') && (currentPos == 'D' || currentPos == 'O'))) {
				followPathCommands.addCommands(Commands.waitSeconds(timeToEmptyFuel));
			}
			try {
				followPathCommands.addCommands(
					AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentPos + autoString.substring(i, i + 2)))
				);
				try {
					allPaths.add(PathPlannerPath.fromPathFile(currentPos + autoString.substring(i, i + 2)));
				} catch(Exception e) {
					error.setString("Adding to all paths issue");
				}
			} catch(Exception e) {
				error.setString("Later path issue");
			}

			currentPos = autoString.charAt(i);
		}

		if(AllianceManager.getAlliance() == DriverStation.Alliance.Blue) {
			autoCommand.addCommands(
				Commands.parallel(
					followPathCommands,
					RobotContainer.rollers.spinUpCommand()
				)
			);
		} else {
			autoCommand.addCommands(
				RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
				Commands.waitUntil(() -> RobotContainer.pivot.atSetpoint()),
				Commands.parallel(
					followPathCommands,
					RobotContainer.rollers.spinUpCommand()
				)
			);
		}

		return autoCommand;
	}

	public void createAuto(String autoString) {
		autoCommand = buildAuto(autoString);
		drawPaths();
	}

	public Command getAutoCommand() {
		return autoCommand;
	}

	public void periodic() {
		alliance.setString(AllianceManager.getAlliance().toString());
		createAuto(autoString.getString(""));
	}
}
