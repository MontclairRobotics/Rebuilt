package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;
import frc.robot.util.AllianceManager;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.PoseUtils;

public class Auto extends SubsystemBase {
	private char currentPos;
	private double timeToEmptyFuel = 6.0; //TODO: get the actual value
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
	Alliance prevAlliance = Alliance.Blue;

	private String prevAutoString = "";

	public void setFeedback(String s, NotificationLevel level) {
		feedbackPub.set(s);
		Notification not = new Notification(level, "Auto Feedback", s, 3000);
		Elastic.sendNotification(not);
	}

	public Auto() {
		autoTopic.setRetained(true);
		stringEnt.set("");
		feedbackTopic.setRetained(true);
		feedbackPub.set("Enter auto string");
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

		if(autoString.length() < 3) {
			setFeedback("Auto string is too short!", NotificationLevel.ERROR);
			return false;
		}

		if(autoString.length() % 2 == 0) {
			setFeedback("One of the paths has an invalid length", NotificationLevel.ERROR);;
			return false;
		}

		String currentAutoString = autoString.substring(0,3);
		String currentPos = autoString.substring(1,2);

		try {
			AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentAutoString));
		} catch(Exception e) {
			setFeedback("Path " + currentAutoString + " doesn\'t exist", NotificationLevel.ERROR);;
			return false;
		}

		for(int i = 3; i < autoString.length(); i += 2) {
			currentAutoString = currentPos + autoString.substring(i, i + 2);
			try {
				AutoBuilder.followPath(PathPlannerPath.fromPathFile(currentAutoString));
			} catch(Exception e) {
				setFeedback("Path " + currentAutoString + " doesn\'t exist", NotificationLevel.ERROR);
				return false;
			}
			currentPos = autoString.substring(i, i + 1);
		}

		setFeedback("Auto String Valid!", NotificationLevel.INFO);
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
				setFeedback("Issue with adding the first path", NotificationLevel.ERROR);
			}
		} catch(Exception e) {
			setFeedback("Unknown error with the first path", NotificationLevel.ERROR);
		}

		for (int i = 3; i < autoString.length(); i += 2) {

			String pathString = currentPos + autoString.substring(i, i + 2);

			if(
				(currentPos == 'L' || currentPos == 'R')
				&& currentPos == autoString.charAt(i)
				|| ((autoString.charAt(i+1) == '0') && (currentPos == 'D' || currentPos == 'O'))) {
				// followPathCommands.addCommands(
				// 	Commands.deadline(
				// 		Commands.waitSeconds(timeToEmptyFuel),
				// 		RobotContainer.shooter.setParameters(() -> Aiming.calculateShot(TargetLocation.HUB, false, true))
				// 	),
				// 	RobotContainer.shooter.stowCommand().withTimeout(1)
				// );
				followPathCommands.addCommands(Commands.waitSeconds(timeToEmptyFuel));
				// followPathCommands.addCommands(RobotContainer.hood.setAngleCommand(HoodConstants.MIN_ANGLE));
			}
			try {
				PathPlannerPath path = PathPlannerPath.fromPathFile(pathString);
				followPathCommands.addCommands(
					AutoBuilder.followPath(path)
				);
				allPaths.add(path);
			} catch(Exception e) {
				setFeedback("Path " + pathString + " not found!", NotificationLevel.ERROR);
				allPaths.clear();
				return Commands.none();
			}

			currentPos = autoString.charAt(i);
		}

		SequentialCommandGroup pivotCommandGroup = new SequentialCommandGroup();

		if(RobotBase.isReal()) {
			pivotCommandGroup.addCommands(
				RobotContainer.pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
				Commands.waitUntil(() -> RobotContainer.pivot.atSetpoint())
			);
		} else {
			pivotCommandGroup.addCommands(
				Commands.none()
			);
		}
		if(AllianceManager.getAlliance() == DriverStation.Alliance.Blue) {
			autoCommand.addCommands(
				pivotCommandGroup,
				Commands.parallel(
					Commands.waitSeconds(3).andThen(RobotContainer.shooter.startShootingInAuto()),
					followPathCommands,
					RobotContainer.rollers.spinUpCommand()
				)
			);

		} else {
			autoCommand.addCommands(
				pivotCommandGroup,
				Commands.parallel(
					Commands.waitSeconds(3).andThen(RobotContainer.shooter.startShootingInAuto()),
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

	public void validateAndCreatePaths(String str) {
		clearField();
		if (isAutoValid(str)) {
			autoCommand = buildAuto(str);
			drawPaths();
		}
	}

	public void clearField() {
        allPaths.clear();
        for (int i = 0; i < 100; i++) {
            FieldObject2d obj = field.getObject("obj" + i);
            obj.setTrajectory(new Trajectory());
        }
    }

	public void periodic() {
		if(DriverStation.isDisabled()) {
			String str = stringEnt.get();
			SmartDashboard.putData(field);

			String autoString = "";
			Optional<Alliance> alliance = DriverStation.getAlliance();

			for (char a : str.toCharArray()) {
                if (a != ' ' && a != ',') {
                    autoString += a;
                }
            }

			if(!autoString.equals(prevAutoString)) {
				prevAutoString = autoString;
				validateAndCreatePaths(autoString);
				Logger.recordOutput("Auto/Auto String", autoString);
				System.out.print("RUNNING AUTO BUILDER");
			} else if (alliance.isPresent() && alliance.get() != prevAlliance) {
				prevAlliance = alliance.get();
				validateAndCreatePaths(autoString);
				Logger.recordOutput("Auto/Auto String", autoString);
				System.out.print("RUNNING AUTO BUILDER");
			}
		}
	}
}
