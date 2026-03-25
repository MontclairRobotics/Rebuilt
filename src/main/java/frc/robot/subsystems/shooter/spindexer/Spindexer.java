package frc.robot.subsystems.shooter.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.spindexer.indexer.Indexer;
import frc.robot.subsystems.shooter.spindexer.serializer.Serializer;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Spindexer {

    private Serializer serializer;
    private Indexer indexer;

    public LoggedTunableNumber spinUpTime = new LoggedTunableNumber("Spindexer/Spin Up Time", 1);
    public LoggedTunableNumber spindDownTime = new LoggedTunableNumber("Spindexer/Spin Down Time", 0.1);

    public Spindexer(Serializer serializer, Indexer indexer) {
        this.serializer = serializer;
        this.indexer = indexer;
    }

    public void spinUp() {
        serializer.spinUp();
        indexer.spinUp();
    }

    public void spinDown() {
        serializer.spinDown();
        indexer.spinDown();
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.run(() -> {
            serializer.setVoltage(6);
            indexer.setVoltage(voltage);
        }, serializer, indexer);
    }

    public Command spinUpCommand() {
        return Commands.run(() -> spinUp(), serializer, indexer);
    }

    public Command spinDownCommand() {
        return Commands.run(() -> spinDown(), serializer, indexer);
    }

    public Command jiggleCommand() {
        return Commands.repeatingSequence(
            spinUpCommand().withTimeout(spinUpTime.getAsDouble()),
            spinDownCommand().withTimeout(spindDownTime.getAsDouble())
        );
    }
}
