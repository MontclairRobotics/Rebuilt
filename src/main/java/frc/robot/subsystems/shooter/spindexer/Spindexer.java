package frc.robot.subsystems.shooter.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.spindexer.indexer.Indexer;
import frc.robot.subsystems.shooter.spindexer.serializer.Serializer;

public class Spindexer {

    private Serializer serializer;
    private Indexer indexer;

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

    public Command spinUpCommand() {
        return Commands.run(() -> spinUp(), serializer, indexer);
    }

    public Command spinDownCommand() {
        return Commands.run(() -> spinDown(), serializer, indexer);
    }
}
