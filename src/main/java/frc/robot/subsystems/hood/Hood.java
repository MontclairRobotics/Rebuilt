package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  public HoodIO io;
  public HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private HoodVisualization hoodVisualization;

  public Hood(HoodIO hoodIO) {
    this.io = hoodIO;
    hoodVisualization = new HoodVisualization();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    hoodVisualization.log();
    hoodVisualization.update();
  }

  public void applyJoystickInput() {
    double voltage =
        Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3)
            * 12;
    // Getting the y value of the joystick and then applies a deadband to ensure the value isn't too
    // high.
    // It's cubed to make the controls smoother, and it's multiplied by 12 to convert to voltage
    // for our 12v battery
    io.setVoltage(voltage);
  }

  public void angleToHub() { // Sets the angle to whatever gets us to score in the hub
    // TODO: Use vision stuffs later to accomplish this task

  }

  public Command setAngleCommand(DoubleSupplier supplier) {
    return Commands.run(() -> io.setAngle(supplier), this);
  }

  public Command setAngleCommand(double angle) {
    return Commands.run(() -> io.setAngle(angle), this).until(() -> io.atSetpoint());
  }

  public Command joystickCommand() {
    return Commands.run(this::applyJoystickInput, this);
  }
}
