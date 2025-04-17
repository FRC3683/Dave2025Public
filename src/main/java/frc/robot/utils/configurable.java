package frc.robot.utils;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface configurable {
    void configure();

    public static void configure_all(configurable... configurables) {
        configure_all(Commands.none(), configurables);
    }

    public static void configure_all(Command along_with, configurable... configurables) {
        AtomicBoolean configured = new AtomicBoolean(false);

        Commands.sequence(
            Commands.print("configuring..."),
            Commands.runOnce(() -> {
                new Thread(() -> {
                    for(var con : configurables) {
                        con.configure();
                    }
                    configured.set(true);
                }).start();
            }),
            Commands.waitUntil(() -> configured.get()),
            Commands.print("configured")
        ).deadlineFor(along_with).ignoringDisable(true).schedule();
    }
}
