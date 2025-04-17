package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.config.LL;
import frc.robot.constants.elevator_state;
import frc.robot.utils.math_utils;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.constants.swerve.*;

import static frc.robot.utils.auto_utils.*;

public class autos {

    private static final double station_speed = 1.0;
    private static final double station_push = 0.7;

    private static Command score_deadline(robot robot, elevator_state score_height, double spit_deadline, boolean coast_swerve) {
        if(RobotBase.isSimulation()) {
            return Commands.waitSeconds(1);
        }
        return Commands.sequence(
            Commands.waitUntil(new Trigger(RobotBase::isSimulation).debounce(1.0).or(robot.swerve::close_to_score)),
            async(robot.elevator.hold_target_state(score_height), Commands.runOnce(() -> { 
                if(coast_swerve) {
                    // robot.swerve.coast();
                }
            })),
            Commands.waitUntil(new Trigger(RobotBase::isSimulation).debounce(1.0).or(() -> robot.swerve.ready_to_score() && robot.elevator.within(score_height, Inches.of(0.4)))),
            async(robot.end_effector.set(constants.end_effector.spit_coral).andThen(Commands.idle(robot.end_effector))),
            Commands.waitUntil(() -> robot.end_effector.get_roller_position().gt(constants.end_effector.auto_spit_rotations)).withTimeout(spit_deadline),
            async(robot.elevator.cmd_set_target_state(elevator_state.STOW)),
            Commands.waitUntil(() -> robot.elevator.get_height().lt(elevator_state.L4.height.minus(Inches.of(0.5))))
        );
    }

    private static Command score_deadline(robot robot, elevator_state score_height, double spit_deadline) {
        return score_deadline(robot, score_height, spit_deadline, false);
    }

    private static Command intake_score_deadline(robot robot, elevator_state prescore, elevator_state score_height, double spit_deadline, boolean coast_swerve) {
        return Commands.sequence(
            Commands.either(
                Commands.waitUntil(robot.end_effector::coral_seated), 
                Commands.waitSeconds(1.0), 
                RobotBase::isReal ),
            Commands.runOnce(() -> {
                robot.end_effector.getCurrentCommand().cancel();
            }),
            async(robot.elevator.hold_target_state(prescore)),
            score_deadline(robot, score_height, spit_deadline, coast_swerve)
        );
    }
    private static Command intake_score_deadline(robot robot, elevator_state prescore, elevator_state score_height, double spit_deadline) {
        return intake_score_deadline(robot, prescore, score_height, spit_deadline, false);
    }

    public static Command into_station(Command to_point, Rotation2d direction, double speed, robot robot, Command cmd) {
        var t = math_utils.find_translation(direction, speed);
        ChassisSpeeds speeds = new ChassisSpeeds(t.getX(), t.getY(), 0);
        return Commands.sequence(
            to_point,
            robot.swerve.strafe_field_relative(() -> speeds)
        ).alongWith(
            async(Commands.waitSeconds(0.1).andThen(cmd))
        ).until(new Trigger(RobotBase::isSimulation).debounce(1.0).or(robot.ramp::has_coral_middle).or(robot.end_effector::lidar_sees_coral));
    }

    public static Command into_station(Command to_point, Rotation2d direction, double speed, robot robot) {
        return into_station(to_point, direction, speed, robot, commands.intake_coral(robot.ramp, robot.elevator, robot.end_effector));
    }

    public static Command red_barge(robot robot) {
        var swerve = robot.swerve;
        var elevator = robot.elevator;

        final double spit_deadline = 0.25;
        final double field_width = constants.tags.getFieldWidth();

        var t = math_utils.find_translation(Rotation2d.fromDegrees(150), 1.0);
        ChassisSpeeds cache = new ChassisSpeeds(t.getX(), t.getY(), 0);

        final Translation2d station = new Translation2d(16.12, 0.55);

        return Commands.sequence(
            robot.swerve.cmd_reset_pose(new Translation2d(10.65, field_width - 6.1))
                .alongWith(async(robot.swerve.snap(60)))
                .alongWith(async(robot.elevator.hold_target_state(elevator_state.IDLE))),
            Commands.sequence(
                robot.swerve.strafe_to_point(new Translation2d(11.7, 2.3), max_speed_mps, 0.1, 2.6),
                robot.swerve.strafe_sttbolls(config.LL_right, 11, 1.0)
            ).withDeadline(Commands.sequence(
                Commands.waitSeconds(0.169),
                async(elevator.hold_target_state(elevator_state.STOW)),
                score_deadline(robot, elevator_state.L4, spit_deadline)
            )),
            // to intake
            async(swerve.snap(126)),
            swerve.strafe_to_point(new Translation2d(12.969, 2.257), max_speed_mps, 0.1, max_speed_mps),
            // intake
            into_station(
                swerve.strafe_to_point(new Translation2d(16.12, 0.55), max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(16.1, 0.49)), // assume flat on hp station
            // to reef 2
            async(swerve.snap(120)),
            Commands.sequence(
                swerve.strafe_line(new Translation2d(14.36, 2.21), Rotation2d.fromDegrees(120), max_speed_mps, 0.1, 2.9),
                swerve.strafe_sttbolls(config.LL_left, 6, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.AUTO_PRESCORE, elevator_state.L4, spit_deadline)),

            // to intake
            async(swerve.snap(126)),
            into_station(
                swerve.strafe_to_point(new Translation2d(16.05, 0.55), max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(16.1, 0.49)), // assume flat on hp station
            // to reef 3
            async(swerve.snap(120)),
            Commands.sequence(
                swerve.strafe_line(new Translation2d(14.04, 1.91), Rotation2d.fromDegrees(120), max_speed_mps, 0.1, 2.9),
                swerve.strafe_sttbolls(config.LL_right, 6, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.AUTO_PRESCORE, elevator_state.L4, spit_deadline)),
            // to intake
            async(swerve.snap(126)),
            into_station(
                swerve.strafe_to_point(new Translation2d(16.05, 0.55), max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(16.1, 0.49)), // assume flat on hp station
            async(swerve.snap(60)),
            // to reef 4
            Commands.sequence(
                Commands.sequence(
                    swerve.strafe_to_point(new Translation2d(12.8, 2.2), max_speed_mps, 0.1, 1.0),
                    swerve.strafe_field_relative(() -> cache).withTimeout(1.0)
                ).until(new Trigger(() -> LimelightHelpers.getTV(config.LL_left_name) && LimelightHelpers.getFiducialID(config.LL_left_name) == 11).debounce(0.03)),
                swerve.strafe_sttbolls(config.LL_left, 11, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.STOW, elevator_state.L4, spit_deadline, false))
            // async(swerve.snap(126))
            // // to station ?
            // swerve.strafe_to_point(new Translation2d(12.969, 2.257), max_speed_mps, 0.1, max_speed_mps),
            // into_station(
            //     swerve.strafe_to_point(new Translation2d(16.05, 0.55), 1.5, 0., 0.8),
            //     Rotation2d.fromDegrees(-54), 0.6, robot)
        ).finallyDo(() -> {
            swerve.coast();
        });
    }

    public static Command red_proc(robot robot) {
        var swerve = robot.swerve;
        var elevator = robot.elevator;

        final double spit_deadline = 0.25;

        final double field_width = constants.tags.getFieldWidth();
        final double i = -1;

        var t = math_utils.find_translation(Rotation2d.fromDegrees(i * 150), 1.5);
        ChassisSpeeds cache = new ChassisSpeeds(t.getX(), t.getY(), 0);

        final Translation2d station = new Translation2d(16.12, field_width - 0.55);

        return Commands.sequence(
            robot.swerve.cmd_reset_pose(new Translation2d(10.65, 6.1))
            // robot.swerve.cmd_reset_pose(new Pose2d(10.4, field_width - 2.34, Rotation2d.fromDegrees(90 * i)))
                .alongWith(async(robot.swerve.snap(60 * i)))
                .alongWith(async(robot.elevator.hold_target_state(elevator_state.IDLE))),
            Commands.sequence(
                robot.swerve.strafe_to_point(new Translation2d(11.7, field_width - 2.3), max_speed_mps, 0.1, 2.8),
                robot.swerve.strafe_sttbolls(config.LL_left, 9, 1.0)
            ).withDeadline(Commands.sequence(
                Commands.waitSeconds(0.169),
                async(elevator.hold_target_state(elevator_state.AUTO_PRESCORE)),
                score_deadline(robot, elevator_state.L4, spit_deadline)
            )),
            // to intake
            async(swerve.snap(126 * i)),
            swerve.strafe_to_point(new Translation2d(12.969, field_width - 2.257), max_speed_mps, 0.1, max_speed_mps),
            // intake
            into_station(
                swerve.strafe_to_point(station.plus(new Translation2d(0, Units.inchesToMeters(7))), max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 * i), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(16.1, field_width - 0.49)), // assume flat on hp station
            // to reef 2
            async(swerve.snap(120 * i)),
            Commands.sequence(
                swerve.strafe_line(new Translation2d(14.36, field_width - 2.21), Rotation2d.fromDegrees(120 * i), max_speed_mps, 0.1, 3.1),
                swerve.strafe_sttbolls(config.LL_right, 8, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.AUTO_PRESCORE, elevator_state.L4, spit_deadline)),

            // to intake
            async(swerve.snap(126 * i)),
            into_station(
                swerve.strafe_to_point(station, max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 * i), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(16.1, field_width - 0.49)), // assume flat on hp station
            // to reef 3
            async(swerve.snap(120 * i)),
            Commands.sequence(
                swerve.strafe_line(new Translation2d(14.04, field_width - 1.91), Rotation2d.fromDegrees(120 * i), max_speed_mps, 0.1, 3.1),
                swerve.strafe_sttbolls(config.LL_left, 8, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.AUTO_PRESCORE, elevator_state.L4, spit_deadline)),
            // to intake
            async(swerve.snap(126 * i)),
            into_station(
                swerve.strafe_to_point(station, max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 * i), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(16.1, field_width - 0.49)), // assume flat on hp station
            async(swerve.snap(60 * i)),
            // to reef 4
            Commands.sequence(
                // Commands.sequence(
                swerve.strafe_to_point(new Translation2d(12.60, 5.64), max_speed_mps, 0.03, 0.0)
                    // swerve.strafe_field_relative(() -> cache).withTimeout(1.0)
                // )
                    .until(new Trigger(() -> LimelightHelpers.getTV(config.LL_right_name) && LimelightHelpers.getFiducialID(config.LL_right_name) == 9).debounce(0.06)),
                swerve.strafe_sttbolls(config.LL_right, 9, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.STOW, elevator_state.L4, spit_deadline)),
            async(elevator.hold_target_state(elevator_state.STOW)),
            // center algae
            swerve.strafe_to_point(new Translation2d(12.4, field_width - 2.94), 2.0, 0.025, 0.0)
        );
    }

    public static Command blue_barge(robot robot) {
        var swerve = robot.swerve;
        var elevator = robot.elevator;

        final double spit_deadline = 0.25;
        final double field_width = constants.tags.getFieldWidth();
        final double field_length = constants.tags.getFieldLength();

        return Commands.sequence(
            robot.swerve.cmd_reset_pose(new Translation2d(field_length - 10.65, 6.1))
                .alongWith(async(robot.swerve.snap(60 + 180)))
                .alongWith(async(robot.elevator.hold_target_state(elevator_state.IDLE))),
            Commands.sequence(
                robot.swerve.strafe_to_point(new Translation2d(field_length - 11.7, field_width - 2.3), max_speed_mps, 0.1, 2.8),
                robot.swerve.strafe_sttbolls(config.LL_right, 20, 1.0)
            ).withDeadline(Commands.sequence(
                Commands.waitSeconds(0.169),
                async(elevator.hold_target_state(elevator_state.STOW)),
                score_deadline(robot, elevator_state.L4, spit_deadline)
            )),
            // to intake
            async(swerve.snap(126 + 180)),
            swerve.strafe_to_point(new Translation2d(field_length - 12.969, field_width - 2.257), max_speed_mps, 0.1, max_speed_mps),
            // intake
            into_station(
                swerve.strafe_to_point(new Translation2d(field_length - 16.05, field_width - 0.55), max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 + 180), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(field_length - 16.1, field_width - 0.49)), // assume flat on hp station
            // to reef 2
            async(swerve.snap(120 + 180)),
            Commands.sequence(
                swerve.strafe_line(new Translation2d(field_length - 14.36, field_width - 2.21), Rotation2d.fromDegrees(120 + 180), max_speed_mps, 0.1, 2.9),
                swerve.strafe_sttbolls(config.LL_left, 19, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.AUTO_PRESCORE, elevator_state.L4, spit_deadline)),

            // to intake
            async(swerve.snap(126 + 180)),
            into_station(
                swerve.strafe_to_point(new Translation2d(field_length - 16.05, field_width - 0.55), max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 + 180), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(field_length - 16.1, field_width - 0.49)), // assume flat on hp station
            // to reef 3
            async(swerve.snap(120 + 180)),
            Commands.sequence(
                swerve.strafe_line(new Translation2d(field_length - 14.04, field_width - 1.91), Rotation2d.fromDegrees(120 + 180), max_speed_mps, 0.1, 2.9),
                swerve.strafe_sttbolls(config.LL_right, 19, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.AUTO_PRESCORE, elevator_state.L4, spit_deadline)),
            // to intake
            async(swerve.snap(126 + 180)),
            into_station(
                swerve.strafe_to_point(new Translation2d(field_length - 16.05, field_width - 0.55), max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 + 180), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(field_length - 16.1, field_width - 0.49)), // assume flat on hp station
            async(swerve.snap(60 + 180)),
            // to reef 4
            Commands.sequence(
                swerve.strafe_to_point(new Translation2d(field_length - 12.8, field_width - 2.2), max_speed_mps, 0.1, 1.5)
                    .until(new Trigger(() -> LimelightHelpers.getTV(config.LL_left_name) && LimelightHelpers.getFiducialID(config.LL_left_name) == 20).debounce(0.06)),
                swerve.strafe_sttbolls(config.LL_left, 20, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.STOW, elevator_state.L4_SUB, spit_deadline)),
            async(elevator.hold_target_state(elevator_state.STOW))
                .alongWith(swerve.reset_pose(config.LL_left)),
            // center algae
            swerve.strafe_to_point(new Translation2d(field_length - 12.4, field_width - 2.94), 2.0, 0.025, 0.0)
        );
    }

    public static Command blue_proc(robot robot) {
        var swerve = robot.swerve;
        var elevator = robot.elevator;

        final double spit_deadline = 0.25;

        final double field_width = constants.tags.getFieldWidth();
        final double field_length = constants.tags.getFieldLength();
        final double i = -1;

        final Translation2d station = new Translation2d(field_length - 16.12, 0.55);

        return Commands.sequence(
            robot.swerve.cmd_reset_pose(new Translation2d(field_length - 10.65, field_width - 6.1))
                .alongWith(async(robot.swerve.snap(60 * i + 180)))
                .alongWith(async(robot.elevator.hold_target_state(elevator_state.IDLE))),
            Commands.sequence(
                robot.swerve.strafe_to_point(new Translation2d(field_length - 11.7, 2.3), max_speed_mps, 0.1, 2.8),
                robot.swerve.strafe_sttbolls(config.LL_left, 22, 1.0)
            ).withDeadline(Commands.sequence(
                Commands.waitSeconds(0.169),
                async(elevator.hold_target_state(elevator_state.AUTO_PRESCORE)),
                score_deadline(robot, elevator_state.L4_andabit, spit_deadline)
            )),
            // to intake
            async(swerve.snap(126 * i + 180)),
            swerve.strafe_to_point(new Translation2d(field_length - 12.969, 2.257), max_speed_mps, 0.1, max_speed_mps),
            // intake
            into_station(
                swerve.strafe_to_point(station.plus(new Translation2d(0, Units.inchesToMeters(7))), max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 * i + 180), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(field_length - 16.1, 0.49)), // assume flat on hp station
            // to reef 2
            async(swerve.snap(120 * i + 180)),
            Commands.sequence(
                swerve.strafe_line(new Translation2d(field_length - 14.36, 2.21), Rotation2d.fromDegrees(120 * i + 180), max_speed_mps, 0.1, 3.1),
                swerve.strafe_sttbolls(config.LL_right, 17, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.AUTO_PRESCORE, elevator_state.L4, spit_deadline)),

            // to intake
            async(swerve.snap(126 * i + 180)),
            into_station(
                swerve.strafe_to_point(station, max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 * i + 180), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(field_length - 16.1, 0.49)), // assume flat on hp station
            // to reef 3
            async(swerve.snap(120 * i + 180)),
            Commands.sequence(
                swerve.strafe_line(new Translation2d(field_length - 14.04, 1.91), Rotation2d.fromDegrees(120 * i + 180), max_speed_mps, 0.1, 3.1),
                swerve.strafe_sttbolls(config.LL_left, 17, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.AUTO_PRESCORE, elevator_state.L4, spit_deadline)),
            // to intake
            async(swerve.snap(126 * i + 180)),
            into_station(
                swerve.strafe_to_point(station, max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 * i + 180), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(field_length - 16.1, 0.49)), // assume flat on hp station
            async(swerve.snap(60 * i + 180)),
            // to reef 4
            Commands.sequence(
                swerve.strafe_to_point(new Translation2d(field_length - 12.60, field_width - 5.64), max_speed_mps, 0.03, 0.0)
                    .until(new Trigger(() -> LimelightHelpers.getTV(config.LL_right_name) && LimelightHelpers.getFiducialID(config.LL_right_name) == 22).debounce(0.03)),
                swerve.strafe_sttbolls(config.LL_right, 22, 1.0)
            ).withDeadline(intake_score_deadline(robot, elevator_state.STOW, elevator_state.L4, spit_deadline)),
            async(elevator.hold_target_state(elevator_state.STOW)),
            // center algae
            swerve.strafe_to_point(new Translation2d(field_length - 12.4, 2.94), 2.0, 0.025, 0.0)
        );
    }

    public static Command red_center(robot robot, LL ll) {
        var swerve = robot.swerve;
        var elevator = robot.elevator;

        var half_field_width = constants.tags.getFieldWidth() / 2.0;

        return Commands.sequence(
            swerve.cmd_reset_pose(new Pose2d(10.42, half_field_width, Rotation2d.fromDegrees(0)))
                .alongWith(async(elevator.hold_target_state(elevator_state.STOW)))
                .alongWith(async(swerve.snap(0))),
            Commands.waitSeconds(1.5),
            swerve.strafe_sttbolls(ll, 10, 1.0)
                .withDeadline(score_deadline(robot, elevator_state.L4, 0.4)),
            swerve.strafe_to_point(new Translation2d(11, half_field_width), 2, 0.05, 0.0)
        );
    }

    public static Command blue_center(robot robot, LL ll) {
        var swerve = robot.swerve;
        var elevator = robot.elevator;

        var half_field_width = constants.tags.getFieldWidth() / 2.0;

        return Commands.sequence(
            swerve.cmd_reset_pose(new Pose2d(7.128, half_field_width, Rotation2d.fromDegrees(180)))
                .alongWith(async(elevator.hold_target_state(elevator_state.STOW)))
                .alongWith(async(swerve.snap(180))),
            Commands.waitSeconds(1.5),
            swerve.strafe_sttbolls(ll, 21, 1.0)
                .withDeadline(score_deadline(robot, elevator_state.L4, 0.4)),
            swerve.strafe_to_point(new Translation2d(6.548, half_field_width), 2, 0.05, 0.0)
        );
    }

    public static Command test(robot robot) {
        var swerve = robot.swerve;
        var elevator = robot.elevator;

        final double spit_deadline = 0.25;

        final double field_width = constants.tags.getFieldWidth();
        final double field_length = constants.tags.getFieldLength();
        final double i = -1;

        return Commands.sequence(
            // robot.swerve.cmd_reset_pose(new Translation2d(field_length - 10.65, field_width - 6.1))
            //     .alongWith(async(robot.swerve.snap(60 * i + 180)))
            //     .alongWith(async(robot.elevator.hold_target_state(elevator_state.IDLE))),
            // Commands.sequence(
            //     robot.swerve.strafe_to_point(new Translation2d(field_length - 11.7, 2.3), max_speed_mps, 0.1, 2.8),
            //     robot.swerve.strafe_sttbolls(config.LL_left, 22, 1.0)
            // ).withDeadline(Commands.sequence(
            //     Commands.waitSeconds(0.169),
            //     async(elevator.hold_target_state(elevator_state.AUTO_PRESCORE)),
            //     score_deadline(robot, elevator_state.L4_andabit, spit_deadline)
            // )),
            // to intake
            // async(swerve.snap(126 * i + 180)),
            // swerve.strafe_to_point(new Translation2d(field_length - 12.969, 2.257), max_speed_mps, 0.1, max_speed_mps),
            // intake
            into_station(
                swerve.strafe_to_point(new Translation2d(field_length - 16.05, 0.55), max_speed_mps, 0.07, station_speed),
                Rotation2d.fromDegrees(-54 * i + 180), station_push, robot),
            swerve.cmd_reset_pose(new Translation2d(field_length - 16.1, 0.49)), // assume flat on hp station

            intake_score_deadline(robot, elevator_state.L2, elevator_state.L2, spit_deadline)
        );
    }
}
