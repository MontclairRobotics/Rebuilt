package frc.robot.subsystems.shooter.spindexer;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.spindexer.indexer.Indexer;
import frc.robot.subsystems.shooter.spindexer.serializer.Serializer;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Spindexer extends SubsystemBase {

    private Serializer serializer;
    private Indexer indexer;

    public LoggedTunableNumber spinUpTime = new LoggedTunableNumber("Spindexer/Spin Up Time", 1);
    public LoggedTunableNumber spinDownTime = new LoggedTunableNumber("Spindexer/Spin Down Time", 0.1);

    public Spindexer(Serializer serializer, Indexer indexer) {
        this.serializer = serializer;
        this.indexer = indexer;
    }

    public void spinUp() {
        serializer.setVoltage(12);
        indexer.setVoltage(12);
    }

    public void spinDown() {
        serializer.spinDown();
        indexer.spinDown();
    }

    public void stop() {
        serializer.stop();
        indexer.stop();
    }

    public void setCurrent(double serializerCurrent, double indexerCurrent) {
        serializer.setCurrent(serializerCurrent);
        indexer.setCurrent(indexerCurrent);
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.run(() -> {
            serializer.setVoltage(voltage);
            indexer.setVoltage(voltage);
        }, serializer, indexer);
    }

    public Command setCurrentCommand(double serializerCurrent, double indexerCurrent) {
        return Commands.run(() -> setCurrent(serializerCurrent, indexerCurrent), this);
    }

    public Command spinUpCommand() {
        return Commands.run(() -> spinUp(), serializer, indexer);
    }

    public Command spinDownCommand() {
        return Commands.run(() -> spinDown(), serializer, indexer);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop(), serializer, indexer);
    }

    public Command jiggleSerializerCommand() {
        return new SequentialCommandGroup(
            serializer.spinUpCommand().withTimeout(1),
            serializer.reverseCommand().withTimeout(1)
        ).repeatedly();
    }
}
