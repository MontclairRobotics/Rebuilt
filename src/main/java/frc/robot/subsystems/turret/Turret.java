package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The TurretIOHardware class interfaces with the TalonFX motor controller and CANCoders to manage
 * turret movement and sensor readings.
 */
public class Turret extends SubsystemBase {

  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
  }
  public Command stopCommand(){
    return Commands.runOnce(() -> io.stop());
  }
  public Command setRobotRelativeAngleCommand(double target){
    return Commands.run(() -> io.setRobotRelativeAngle(target), this).until(() -> io.atSetpoint());
  }
  public Command setRobotRelativeAngleContinuousCommand(DoubleSupplier target){
    return Commands.run(() -> io.setRobotRelativeAngle(target), this);
  }
    public Command setFieldRelativeAngleCommand(double target){
    return Commands.run(() -> io.setFieldRelativeAngle(target), this).until(() -> io.atSetpoint());
  }
  public Command setFieldRelativeAngleContinuousCommand(DoubleSupplier target){
    return Commands.run(() -> io.setFieldRelativeAngle(target), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }
}
