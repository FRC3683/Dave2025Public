package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.commands;
import frc.robot.subsystems.end_effector.gamepiece;
import frc.robot.constants.elevator_state;
import frc.robot.utils.dave_led;
import frc.robot.utils.math_utils;
import frc.robot.utils.oi;
import frc.robot.utils.controls.dave_talon;

public final class bindings {
    public static elevator_state height_for_the_elevator_coral = elevator_state.L3;
    public static elevator_state height_for_the_elevator_algae_intake = elevator_state.ALGAE1;
    public static elevator_state height_for_the_elevator_algae_score = elevator_state.ALGAE_BARGE;
    public static Color8Bit abxy_color = dave_led.clr_blue;

    public static boolean manual_intake = false;
    public static boolean should_backup_coral = false;
    public static boolean post_algae_flag = false;

    private static boolean spit_pose_spit = false;
    private static Pose2d spit_pose = new Pose2d();

    // SWERVE STICKS
    public static final oi.shaping_chooser strafe_input_shaping = new oi.shaping_chooser("strafe_input_shaping");
    public static final oi.shaping_chooser turn_input_shaping = new oi.shaping_chooser("turn_input_shaping");
    public static final Supplier<Translation2d> ctrl_strafe = () -> oi.vec_deadband(oi.get_left_stick(oi.driver), oi.shaping_chooser::custom);
    public static final Supplier<Double> ctrl_turn = () -> oi.deadband_precise(-oi.driver.getRightX(), math_utils::sq);

    public static void configure_bindings(robot robot) {
        configure_auto(robot);


        // CONTROLS
        var ctrl_reset_heading = oi.cmd_driver.povDown();
        var ctrl_elevator_to_b = oi.cmd_driver.b();
        var ctrl_elevator_to_y = oi.cmd_driver.y();
        var ctrl_elevator_to_a = oi.cmd_driver.a();
        var ctrl_elevator_to_x = oi.cmd_driver.x();
        var ctrl_intake_coral = oi.cmd_driver.rightTrigger();
        var ctrl_prescore = oi.cmd_driver.leftBumper();
        var ctrl_spit = oi.cmd_driver.rightBumper();
        var ctrl_intake_algae = oi.cmd_driver.leftTrigger();
        var ctrl_auto_align_left = oi.cmd_driver.back(); // left middle
        var ctrl_auto_align_right = oi.cmd_driver.start(); // right middle
        var ctrl_climb_prep = ctrl_elevator_to_b.and(ctrl_elevator_to_y).and(ctrl_elevator_to_a).and(ctrl_elevator_to_x);
        var ctrl_uppies = oi.cmd_driver.a().and(robot.climb::prepped);
        var ctrl_tap = oi.cmd_driver.y().and(robot.climb::prepped);
        var ctrl_climb_snap = oi.cmd_driver.b().and(robot.climb::prepped);

        // XKEYS
        var ctrl_swerve_abs = oi.cmd_xkeys.button(11);
        var ctrl_manual_up = oi.cmd_xkeys.button(12);
        var ctrl_manual_down = oi.cmd_xkeys.button(13);
        var ctrl_manual_pivot_up = oi.cmd_xkeys.button(14);
        var ctrl_manual_pivot_down = oi.cmd_xkeys.button(15);
        var ctrl_end_manual = oi.cmd_xkeys.button(16);
        var ctrl_zero_elevator = oi.cmd_xkeys.button(17);
        var ctrl_zero_pivot = oi.cmd_xkeys.button(18);
        var ctrl_trap = oi.cmd_xkeys.button(19);

        var ctrl_aux1 = oi.cmd_xkeys.button(20);
        var ctrl_aux2 = oi.cmd_xkeys.button(21);
        var ctrl_aux3 = oi.cmd_xkeys.button(22);
        var ctrl_aux4 = oi.cmd_xkeys.button(23);
        var ctrl_aux5 = oi.cmd_xkeys.button(24);
        var ctrl_aux7 = oi.cmd_xkeys.button(26);
        var ctrl_aux8 = oi.cmd_xkeys.button(27);
        
        ctrl_aux1.onTrue(dave_talon.reset_music());
        ctrl_aux2.onTrue(dave_talon.play_selected());

        var ctrl_barf = oi.cmd_xkeys.button(25);
        var ctrl_flash_leds = new Trigger(() -> false);

        var swerve = robot.swerve;

        var trap = Commands.parallel(
            robot.elevator.hold_target_state(elevator_state.TRAP),
            robot.end_effector.set(constants.end_effector.intake_fast),
            robot.ramp.cmd_run(constants.ramp.intake.times(-1))
        );

        //                                              score coral         intake algae            score algae
        ctrl_elevator_to_y.onTrue(change_state(elevator_state.L4,   elevator_state.ALGAE2,          elevator_state.ALGAE_BARGE, dave_led.clr_yellow));
        ctrl_elevator_to_x.onTrue(change_state(elevator_state.L3,   elevator_state.ALGAE1,          elevator_state.ALGAE_BARGE, dave_led.clr_blue));
        ctrl_elevator_to_a.onTrue(change_state(elevator_state.L2,   elevator_state.ALGAE_LOLLIPOP,  elevator_state.ALGAE_PROCESSOR, dave_led.clr_green));
        ctrl_elevator_to_b.onTrue(change_state(elevator_state.L1,   elevator_state.ALGAE_GROUND,    elevator_state.ALGAE_PROCESSOR, dave_led.clr_red));

        var unjam_trigger = new Trigger(() -> robot.ramp.get_current_amps() >= 12 && Math.abs(robot.ramp.get_velocity_rps()) <= 3).debounce(0.2);

        var auto_intake = commands.auto_intake_coral_ramp(robot.ramp, robot.elevator, robot.end_effector);

        var unjam = Commands.run(() -> {
            auto_intake.cancel();
            robot.ramp.run(constants.ramp.unjam);
        }, robot.ramp).withTimeout(0.11);

        new Trigger(() -> robot.end_effector.lidar_sees_coral() && robot.end_effector.getDefaultCommand().isScheduled())
        .onTrue(robot.end_effector.feed_coral_past_sensor().andThen(robot.end_effector.cmd_nudge()));

        var auto_intake_trigger = new Trigger(() -> 
            (   ctrl_intake_coral.getAsBoolean() || ( DriverStation.isAutonomous() && robot.end_effector.auto_flag && !unjam.isScheduled() )   )
            && robot.ramp.has_coral()
            && !ctrl_trap.getAsBoolean()
        );

        ctrl_elevator_to_b.onTrue(Commands.runOnce(() -> {
            post_algae_flag = false;
        }));

        new Trigger(() -> swerve.is_assisting() && !swerve.reef_align_has_target()).whileTrue(
            robot.leds.flag_LL_no_target.run().alongWith(Commands.idle()) );

        ctrl_intake_coral.whileTrue(commands.intake_coral(robot.ramp, robot.elevator, robot.end_effector));

        ctrl_aux5.onTrue(Commands.runOnce(() -> {
            manual_intake = !manual_intake;
        }));

        ctrl_spit.onTrue(Commands.runOnce(() -> {
            if(auto_intake.isScheduled()) {
                auto_intake.cancel();
            }
        }));

        var barf = robot.end_effector.spit(gamepiece.ALGAE)
            .alongWith(robot.ramp.cmd_run(constants.ramp.intake.unaryMinus()));

        // I hate this. theres a better way using triggers but this was done during drive practice, ie. it has to work IMMEDIATELY. 
        // don't do commands like this if you can help it.
        robot.addPeriodic(() -> {
            if(auto_intake_trigger.getAsBoolean() && !auto_intake.isScheduled() && !ctrl_barf.getAsBoolean()) {
                auto_intake.schedule();
                if(ctrl_intake_coral.getAsBoolean()) {
                    oi.rumble_for(0.5);
                }
            }
            if(ctrl_barf.getAsBoolean() && !barf.isScheduled()) {
                auto_intake.cancel();
                barf.schedule();
            }
            if(unjam_trigger.getAsBoolean() && !unjam.isScheduled()) {
                auto_intake.cancel();
                unjam.schedule();
            }
            if(ctrl_trap.getAsBoolean() && !trap.isScheduled()) {
                auto_intake.cancel();
                trap.schedule();
            } else if(!ctrl_trap.getAsBoolean() && trap.isScheduled()) {
                trap.cancel();
            }

        }, 0.02);

        Trigger auto_align_any = ctrl_auto_align_left.or(ctrl_auto_align_right);

        ctrl_prescore.and(() -> robot.end_effector.last_intake == gamepiece.CORAL && bindings.height_for_the_elevator_coral != elevator_state.L1).whileTrue(
            Commands.run(() -> {
                robot.elevator.set_target_state(bindings.height_for_the_elevator_coral);
            }, robot.elevator)
        );

        var l1_prescore = ctrl_prescore.and(() -> robot.end_effector.last_intake == gamepiece.CORAL && bindings.height_for_the_elevator_coral == elevator_state.L1 && !auto_intake.isScheduled());

        l1_prescore.onTrue(
            Commands.sequence(
                Commands.run(() -> {
                    robot.elevator.set_target_state(elevator_state.L1);
                }, robot.elevator)
                    .until(ctrl_spit.or(ctrl_prescore.negate())),
                Commands.waitUntil(ctrl_spit.negate()),
                robot.elevator.hold_target_state(elevator_state.L1_andahalf)
                    .until(() -> robot.elevator.get_pivot().plus(robot.elevator.get_pivot_speed().times(Seconds.of(0.4))).gt(Degrees.of(20)) && l1_prescore.negate().getAsBoolean())
            ).deadlineFor(
                swerve.intake_throttle()
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        var barge_snap = robot.swerve.snap(() -> frc.robot.robot.is_red() ? 180.0 : 0.0)
            .alongWith(Commands.runOnce(() -> {
                spit_pose_spit = false;
            }));

        ctrl_prescore.and(() -> robot.end_effector.last_intake == gamepiece.ALGAE 
            && height_for_the_elevator_algae_score == elevator_state.ALGAE_BARGE).whileTrue(barge_snap);


        BooleanSupplier is_backed_up = () -> {
            if(frc.robot.robot.is_red()) {
                return robot.swerve.get_pose().getX() > spit_pose.getX() + 0.15;
            } else {
                return robot.swerve.get_pose().getX() < spit_pose.getX() - 0.15;
            }
        };

        new Trigger(() -> robot.swerve.theta_within(15)).debounce(0.1).and(() -> barge_snap.isScheduled())
            .onTrue(
                robot.swerve.blind(
                    robot.elevator.hold_target_state(elevator_state.ALGAE_BARGE)
                ).until(() -> !ctrl_prescore.getAsBoolean() && ( (is_backed_up.getAsBoolean() && spit_pose_spit) || (!spit_pose_spit) ) )
            );

        ctrl_prescore.and(() -> robot.end_effector.last_intake == gamepiece.ALGAE 
            && height_for_the_elevator_algae_score == elevator_state.ALGAE_PROCESSOR)
        .whileTrue(
            robot.swerve.snap(() -> frc.robot.robot.is_red() ? 90.0 : -90.0)
                .alongWith(
                    robot.elevator.hold_target_state(elevator_state.ALGAE_PROCESSOR)
                )
        );

        oi.cmd_xkeys.button(20).onTrue(swerve.reset_pose(config.LL_left));

        var auto_score_cmd = robot.end_effector.set(constants.end_effector.spit_coral).andThen(Commands.idle(robot.end_effector))
            .until(auto_align_any.negate());

        auto_align_any.and(swerve::close_to_score).and(() -> !auto_intake.isScheduled() && !post_algae_flag).onTrue(
            commands.prescore(robot.elevator, robot.end_effector)
                .until(auto_align_any.negate().and(() -> !auto_score_cmd.isScheduled() || robot.end_effector.get_roller_position().gt(constants.end_effector.auto_spit_rotations)))
        );

        ctrl_auto_align_left.whileTrue(Commands.startEnd(() -> {
            robot.swerve.elevator_assist_left = true;
        }, () -> {
            robot.swerve.elevator_assist_left = false;
        }));

        ctrl_auto_align_right.whileTrue(Commands.startEnd(() -> {
            robot.swerve.elevator_assist_right = true;
        }, () -> {
            robot.swerve.elevator_assist_right = false;
        }));

        auto_align_any.whileTrue(swerve.snap_reef_pose());

        Trigger auto_score = auto_align_any
            .and(swerve::ready_to_score)
            .and(new Trigger(() -> robot.elevator.within(height_for_the_elevator_coral, Inches.of(0.375))).debounce(0.05));

        auto_score.onTrue( auto_score_cmd );

        ctrl_spit.and(() -> robot.end_effector.last_intake == gamepiece.ALGAE).onFalse(Commands.runOnce(() -> {
            post_algae_flag = true;
            spit_pose = robot.swerve.get_pose();
            spit_pose_spit = true;
            // Thomas wanted the robot to have to back up to bring the elevator down. This is not the cleanest way of doing it.
            // but drive practice must continue so the quick way is the correct way.
        }));

        new Trigger(() -> post_algae_flag && robot.elevator.height_at_setpoint(elevator_state.STOW)).onTrue(Commands.runOnce(() -> {
            post_algae_flag = false;
            if(robot.ramp.has_coral()) {
                auto_intake.schedule();
            }
        }));

        ctrl_spit
            .or(() -> robot.elevator.get_target() == elevator_state.L1_andahalf && ctrl_prescore.getAsBoolean())
        .whileTrue(
            robot.end_effector.spit_last()
            .deadlineFor(leds.flag_intake_flash.run()) );

        ctrl_intake_algae.whileTrue( 
            commands.intake_algae(robot.elevator, robot.end_effector)
            .alongWith(swerve.intake_throttle())
        );

        ctrl_intake_algae.and(() -> (height_for_the_elevator_algae_intake == elevator_state.ALGAE1 || height_for_the_elevator_algae_intake == elevator_state.ALGAE2))
            .whileTrue(Commands.parallel(
                swerve.snap_reef_pose()
            ));

        ctrl_intake_algae.and(() -> (height_for_the_elevator_algae_intake == elevator_state.ALGAE1 || height_for_the_elevator_algae_intake == elevator_state.ALGAE2))
            .and(swerve::close_to_reef)
            .and(new Trigger(() -> robot.elevator.within_target(Inches.of(6))).debounce(0.02))
            .whileTrue(
                swerve.strafe_to_point(() -> {
                    int id = swerve.reef_assist_id;
                    var tag = constants.tags.getTagPose(id).get();
                    var tag_pos = tag.getTranslation().toTranslation2d();
                    var tag_angle = Rotation2d.fromRadians(tag.getRotation().getMeasureZ().in(Radians));
                    var offset = math_utils.find_translation(tag_angle, 0.5);
                    return tag_pos.plus(offset);
                },
                2, 0, 0)
            );

        ctrl_swerve_abs.onTrue(Commands.runOnce(() -> {
            swerve.reset_encoders();
        }).ignoringDisable(true));

        ctrl_reset_heading.onTrue(Commands.runOnce(() -> {
        swerve.zero_heading(frc.robot.robot.is_red() ? Rotation2d.k180deg : Rotation2d.kZero);
        }).ignoringDisable(true));

        var manual = robot.elevator.manual(oi.create_axis(ctrl_manual_up, ctrl_manual_down), oi.create_axis(ctrl_manual_pivot_up, ctrl_manual_pivot_down));
        ctrl_manual_up.or(ctrl_manual_down).or(ctrl_manual_pivot_up).or(ctrl_manual_pivot_down).onTrue(Commands.runOnce(() -> {
            auto_intake.cancel();
            manual.schedule();
        }));
        ctrl_end_manual.onTrue(Commands.runOnce(() -> {
            manual.cancel();
        }));

        ctrl_zero_elevator.and(() -> manual.isScheduled() || DriverStation.isDisabled()).onTrue(Commands.runOnce(() -> {
            robot.elevator.zero();
        }).ignoringDisable(true));

        ctrl_zero_pivot.and(() -> manual.isScheduled() || DriverStation.isDisabled()).onTrue(Commands.runOnce(() -> {
            robot.elevator.home_pivot();
        }).ignoringDisable(true));

        ctrl_flash_leds.whileTrue(robot.leds.flag_operator_flash.run());

        ctrl_climb_prep.onTrue(robot.elevator.hold_target_state(elevator_state.CLIMB));
        ctrl_climb_prep.onTrue(robot.ramp.cmd_stop());
        ctrl_climb_prep.onTrue(robot.climb.preclimb());
        ctrl_climb_prep.onTrue(robot.swerve.climb_throttle());
        ctrl_climb_prep.onTrue(leds.flag_flash_climb.run().until(robot.climb::prepped));
        new Trigger(robot.climb::prepped).onTrue(leds.flag_solid_climb.run());
        ctrl_uppies.onTrue(robot.climb.uppies());
        ctrl_tap.onTrue(robot.climb.tap());
        ctrl_climb_snap.whileTrue(robot.swerve.snap(() -> frc.robot.robot.is_red() ? -90.0 : 90.0));

        new Trigger(() -> DriverStation.isDisabled()).debounce(2).onTrue(Commands.runOnce(() -> {
            boolean auto = DriverStation.isAutonomous();
            robot.elevator.apply_current_limits(auto);
            robot.swerve.apply_current_limits(auto);
        }).ignoringDisable(true));

        new Trigger(() -> DriverStation.isTeleop()).onTrue(Commands.runOnce(() -> {
            robot.elevator.apply_current_limits(false);
            robot.swerve.apply_current_limits(false);
        }).ignoringDisable(true));

        new Trigger(() -> DriverStation.isAutonomous()).onTrue(Commands.runOnce(() -> {
            robot.elevator.apply_current_limits(true);
            robot.swerve.apply_current_limits(true);
        }).ignoringDisable(true));

        new Trigger(() -> DriverStation.isDisabled() && robot.elevator.at_home()).debounce(0.2).onTrue(
            Commands.runOnce( () -> {
                robot.elevator.zero();
                robot.swerve.brake();
            } )
            .alongWith(robot.leds.flag_flash_zero.run().alongWith(Commands.idle()).withTimeout(0.5))
            .ignoringDisable(true)
        );
    }

    private static Command change_state(elevator_state coral, elevator_state algae_in, elevator_state algae_score, Color8Bit clr) {
        return Commands.runOnce(() -> {
            height_for_the_elevator_coral = coral;
            height_for_the_elevator_algae_intake = algae_in;
            height_for_the_elevator_algae_score = algae_score;
            abxy_color = clr;
        }).ignoringDisable(true);
    }

    private static void configure_auto(robot robot) {

    }
}
