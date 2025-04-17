package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bindings;
import frc.robot.constants;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.end_effector;
import frc.robot.subsystems.ramp;
import frc.robot.constants.elevator_state;
import frc.robot.subsystems.end_effector.gamepiece;

public class commands {

    public static Command intake_coral(ramp ramp, elevator elevator, end_effector end_effector) {
        return end_effector.intake(gamepiece.CORAL)
            .alongWith(Commands.run(() -> {
                end_effector.auto_flag = true;
                elevator.set_target_state(elevator_state.RAMP);
                ramp.run(constants.ramp.intake);
            }, ramp, elevator))
            .andThen(Commands.idle(ramp, elevator, end_effector));
    }

    public static Command auto_intake_coral_ramp(ramp ramp, elevator elevator, end_effector end_effector) {
        return Commands.deadline(
            Commands.sequence(
                ramp.smart_hold().until(() -> elevator.height_at_setpoint(elevator_state.RAMP)),
                Commands.runOnce(() -> {
                    end_effector.set_feed(constants.end_effector.intake_fast);
                    ramp.run(constants.ramp.intake);
                    end_effector.last_intake = gamepiece.CORAL;
                }, ramp, end_effector),
                Commands.idle(ramp, end_effector).until(new Trigger(() -> end_effector.lidar_sees_coral()).debounce(0.08)),
                end_effector.feed_coral_past_sensor(),
                end_effector.cmd_nudge()
            ),
            Commands.runOnce(() -> {
                end_effector.last_intake = gamepiece.CORAL;
            }),
            elevator.hold_target_state(elevator_state.RAMP),
            frc.robot.leds.flag_auto_intake.run()
        ).finallyDo(() -> {
            frc.robot.leds.flag_intake_flash.run().withTimeout(0.5).schedule();
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command intake_algae(elevator elevator, end_effector end_effector) {
        return end_effector.intake(gamepiece.ALGAE)
            .alongWith( Commands.run(() -> {
                elevator.set_target_state(bindings.height_for_the_elevator_algae_intake);
            }, elevator) );
    }

    public static Command prescore(elevator elevator, end_effector end_effector) {
        return Commands.run(() -> {
            switch(end_effector.last_intake) {
                case ALGAE:
                    elevator.set_target_state(bindings.height_for_the_elevator_algae_score);
                break;
                case CORAL:
                    elevator.set_target_state(bindings.height_for_the_elevator_coral);
                    break;
            }
        }, elevator);
    }

}
