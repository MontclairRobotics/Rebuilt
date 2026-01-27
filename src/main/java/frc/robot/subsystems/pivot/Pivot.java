package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private RobotState targetState;

  public Pivot(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public void stop() {
    io.stop();
  }

  public void setTargetState(RobotState state) {
    this.targetState = state;
  }

  public RobotState getTargetState() {
    return targetState;
  }

  public Rotation2d setPivotAngle(Rotation2d angle) {
    return io.getPivotAngle();
  }

  public void resetPIDController() {
    io.resetPIDController();
  }

  public double getPercentRotation() {
    return io.getPercentRotation();
  }

  public Rotation2d getPivotAngle() {
    return io.getPivotAngle();
  }

  public void joystickControl() {
    double voltage =
        Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3)
            * 12;
    voltage += io.calculateStationaryFeedforward();
    voltage = MathUtil.clamp(voltage, -3, 3);
    io.setVoltage(voltage);
  }

  // --------------COMMANDS---------------\\

  public Command stopCommand() {
    return Commands.runOnce(() -> stop());
  }

  //simple go to angle
  public Command goToAngleCommand(Rotation2d angle) {
    return Commands.run(
            () -> {
              setPivotAngle(angle);
            },
            this)
        .until(this::atSetpoint)
        .finallyDo(
            () -> {
              stop();
              resetPIDController();
            });
  }

//keep going to the angle without stopping
  public Command goToAngleContinuousCommand(Rotation2d angle) {
    return Commands.run(
            () -> {
              setPivotAngle(angle);
            },
            this)
        .finallyDo(
            () -> {
              stop();
              resetPIDController();
            });
  }

  //joystick control command (
  public Command joystickControlCommand() {
    return Commands.run(this::joystickControl, this);
  }
}
