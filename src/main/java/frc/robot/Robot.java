// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

	private Command autonomousCommand;
	private RobotContainer robotContainer;

	public Robot() {

		robotContainer = new RobotContainer();

		// Record metadata
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		Logger.recordMetadata(
			"GitDirty",
			switch (BuildConstants.DIRTY) {
			case 0 -> "All changes committed";
			case 1 -> "Uncommitted changes";
			default -> "Unknown";
			});

		// Set up data receivers & replay source
		switch (Constants.CURRENT_MODE) {
		case REAL:
			// Running on a real robot, log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new WPILOGWriter());
			Logger.addDataReceiver(new NT4Publisher());
			break;

		case SIM:
			// Running a physics simulator, log to NT
			Logger.addDataReceiver(new NT4Publisher());
			break;

		case REPLAY:
			// Replaying a log, set up replay source
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
			break;
		}

		// Start AdvantageKit logger
		Logger.start();
	}

	@Override
	public void robotInit() {
		RobotContainer.drivetrain.resetPose(new Pose2d());
		DogLog.setOptions(
			new DogLogOptions()
				.withLogExtras(true)
				.withCaptureDs(true)
				.withNtPublish(true)
				.withCaptureNt(true));
		DogLog.setPdh(new PowerDistribution());
		if (isSimulation()) {
		// Do not spam the logs with "Button x on port y not available" log messages.
		DriverStation.silenceJoystickConnectionWarning(true);
		}
	}
	/** This function is called periodically during all modes. */
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		if (!RobotBase.isReal()) {
		robotContainer.resetSimulation();
		}
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {}

	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
		autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
		autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
		robotContainer.resetSimulation();
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
		robotContainer.fuelSim.updateSim();
		RobotContainer.drivetrain.mapleSimSwerveDrivetrain.update();
		robotContainer.displaySimFieldToAdvantageScope();
	}
}
