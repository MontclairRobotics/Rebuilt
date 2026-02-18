package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.PivotConstants.TOLERANCE;
import static frc.robot.constants.PivotConstants.kD;
import static frc.robot.constants.PivotConstants.kG;
import static frc.robot.constants.PivotConstants.kI;
import static frc.robot.constants.PivotConstants.kP;
import static frc.robot.constants.PivotConstants.kS;
import static frc.robot.constants.PivotConstants.kV;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  public PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private PIDController pidController;
  private ArmFeedforward pivotFeedfoward;
  private PivotVisualization visualization;

  public Pivot(PivotIO io) {
    this.io = io;
    this.visualization = new PivotVisualization();

    pidController = new PIDController(kP, kI, kD);
    pivotFeedfoward = new ArmFeedforward(kS, kG, kV);

    pidController.disableContinuousInput();
    pidController.setTolerance(TOLERANCE.in(Rotations));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    visualization.update();
    visualization.log();
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public Command testVoltageCommand() {
    return Commands.run(() -> io.setVoltage(2));
  }

  public void setPivotAngle(Angle angle) {
    double pidOutput =
        pidController.calculate(io.getPivotAngle().in(Rotations), angle.in(Rotations));
    double ffOutput = pivotFeedfoward.calculate(io.getPivotAngle().in(Radians), 0);
    double totalOutput = MathUtil.clamp(pidOutput + ffOutput, -12, 12);
    io.setVoltage(totalOutput);
  }

  public void joystickControl() {
    double voltage =
        Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3)
            * 12;
    io.setVoltage(voltage);
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> io.stop());
  }

  public Command goToAngleCommand(Angle angle) {
    return Commands.run(
            () -> {
              setPivotAngle(angle);
            },
            this)
        .until(this::atSetpoint)
        .finallyDo(
            () -> {
              io.stop();
              pidController.reset();
            });
  }

  public Command goToAngleCommand(Supplier<Angle> angle) {
    return Commands.run(
            () -> {
              setPivotAngle(angle.get());
            },
            this)
        .finallyDo(
            () -> {
              io.stop();
              pidController.reset();
            });
  }

  public Command joystickControlCommand() {
    return Commands.run(this::joystickControl, this);
  }
}
