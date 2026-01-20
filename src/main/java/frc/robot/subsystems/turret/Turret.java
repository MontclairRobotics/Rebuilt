package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The TurretIOHardware class interfaces with the TalonFX motor controller and CANCoders to manage
 * turret movement and sensor readings.
 */
public class Turret extends SubsystemBase {

  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private TurretVisualization turretVisualization;

  public Turret(TurretIO io) {
    this.io = io;
    turretVisualization = new TurretVisualization();
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> io.stop());
  }

  public Command setRobotRelativeAngleCommand(double target) {
    return Commands.run(() -> io.setRobotRelativeAngle(target), this).until(() -> io.atSetpoint());
  }

  public Command setRobotRelativeAngleContinuousCommand(DoubleSupplier target) {
    return Commands.run(() -> io.setRobotRelativeAngle(target), this);
  }

  public Command setFieldRelativeAngleCommand(double target) {
    return Commands.run(() -> io.setFieldRelativeAngle(target), this).until(() -> io.atSetpoint());
  }

  public Command setFieldRelativeAngleContinuousCommand(DoubleSupplier target) {
    return Commands.run(() -> io.setFieldRelativeAngle(target), this);
  }

  public Command setPositiveVoltageCommand() {
    return Commands.run(() -> io.setVoltage(2));
  }

  public Command setNegativeVoltageCommand() {
    return Commands.run(() -> io.setVoltage(-2));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("RobotPose", new Pose3d(2, 5, 0, Rotation3d.kZero));
    turretVisualization.update(inputs);
    turretVisualization.log();
  }
}
