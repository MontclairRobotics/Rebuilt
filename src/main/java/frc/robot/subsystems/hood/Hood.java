package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import java.util.function.DoubleSupplier;

public class Hood extends SubsystemBase {
  private HoodIO io;
  private HoodIOInputs inputs;

  public Hood(HoodIO hoodIO, HoodIOInputs hoodIOInputs) {
    this.io = hoodIO;
    this.inputs = hoodIOInputs;
  }

  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Hood", inputs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  /** Gets the angle in Rotations from the relative encoder */
  public double getAngle() {
    return io.getAngle();
  }

  public void setAngle(double goal) {
    io.setAngle(goal);
  }

  public void setAngle(DoubleSupplier goalSupplier) {
    io.setAngle(goalSupplier);
  }

  public void stop() {
    io.stop();
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public void doJoystickControls() {
    double voltage =
        Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3);
    voltage = MathUtil.clamp(voltage, -1, 1); // TODO: Use real clamp values -1, 1 is temporary
    // Maybe do feedforward?
    this.setVoltage(voltage);
  }

  public void angleToHub() { // Sets the angle to whatever gets us to score in the outpost
    // TODO: Use vision stuffs later to accomplish this task

  }

  public Command setAngleCommand(DoubleSupplier supplier) {
    return Commands.run(() -> setAngle(supplier), this);
  }

  public Command setAngleCommand(double angle) {
    return Commands.run(() -> setAngle(angle), this).until(() -> atSetpoint());
  }

  public Command joystickCommand() {
    return Commands.run(this::doJoystickControls, this);
  }
}
