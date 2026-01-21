package frc.robot.subsystems.pivot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.PivotConstants;
import frc.robot.util.PoseUtils;

public class PivotIOSim implements PivotIO {
  private DutyCycleEncoderSim encoder;
  private DutyCycleEncoder realEncoder; // TODO: fix this

  private SingleJointedArmSim
      sim; // representing our double jointed arm as a single jointed arm sim of the first joint,
  // and chose to ignore the second joint :)

  private PIDController pidController;
  private ArmFeedforward armFeedforward;

  private double appliedVoltage;

  public PivotIOSim() {
    pidController = new PIDController(0, 0, 0); // TODO: define this or replace
    armFeedforward = new ArmFeedforward(0, 0, 0);

    realEncoder = new DutyCycleEncoder(0, 0, PivotConstants.SHOULDER_ENCODER_OFFSET.getRotations());
    encoder = new DutyCycleEncoderSim(realEncoder);
    encoder.setConnected(true);

        sim =
            new SingleJointedArmSim(
                DCMotor.getNEO(1),
                PivotConstants.PIVOT_TO_MOTOR,
                10.0,
                0.0,
                1.0,
                1.0,
                true,
                1.0,
                0.0,
                0.0);
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.appliedVoltage = appliedVoltage;
    inputs.current = sim.getCurrentDrawAmps();

    inputs.angle = getPivotAngle();

    inputs.encoderConnected = encoder.getConnected();
  }

  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    sim.setInputVoltage(voltage);
    encoder.set(sim.getAngleRads());
  }

  public void stop() {
    sim.setInputVoltage(0);
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRotations(encoder.get() % 1);
  }

  public boolean atSetpoint() {
    return false; // TODO: huh???
  }

  public double getPercentRotation() {
    double distance =
        PoseUtils.getAngleDistance(getPivotAngle(), PivotConstants.PIVOT_MIN_ANGLE).getDegrees();
    double interval =
        PoseUtils.getAngleDistance(PivotConstants.PIVOT_MAX_ANGLE, PivotConstants.PIVOT_MIN_ANGLE)
            .getDegrees();
    return distance / interval;
    // TODO: please clarify if this is correct!!
    // TODO: also it said about fixing something in "PoseUtils"
  }

  public void resetPIDController() {
    pidController.reset();
  }

  public double calculateStationaryFeedforward() {
    double voltage = armFeedforward.calculate(getPivotAngle().getRadians(), 0);
    return getPivotAngle().getDegrees() > 0 ? voltage : -voltage;
  }

  public void setIdleMode(IdleMode mode) {}

  public Command setIdleModeCommand(IdleMode mode) {
    return Commands.runOnce(() -> setIdleMode(mode)).ignoringDisable(true);
  }
}
