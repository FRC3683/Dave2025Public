package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.constants.swerve.*;

import java.util.concurrent.BlockingQueue;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.bindings;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.config.LL;
import frc.robot.constants.elevator_state;
import frc.robot.utils.dave_auto;
import frc.robot.utils.dummy;
import frc.robot.utils.field_util;
import frc.robot.utils.math_utils;
import frc.robot.utils.controls.drivetrain_controller;
import frc.robot.utils.controls.swerve_lowlevel;
import frc.robot.utils.controls.swerve_slew3;

public class swerve extends swerve_lowlevel implements dave_auto.swerve {

    private ChassisSpeeds desired_field_relative_speeds = new ChassisSpeeds();
    private ChassisSpeeds cached_joystick_speeds = new ChassisSpeeds();
    private final swerve_slew3 slew_tele = new swerve_slew3(slew_strafe_accel, slew_omega, constants.control_dts);
    private final swerve_slew3 slew_auto = new swerve_slew3(max_speed_mps / 0.1, 9.0 / 0.15, constants.control_dts);
    private final swerve_slew3 elevator_slew = new swerve_slew3(max_speed_mps / 0.5, 0, constants.control_dts);

    private boolean reject_vision_updates = false;
    private Pose2d[] cam_pose = { new Pose2d(), new Pose2d() };
    private double[] last_vision_timestamps = { 0, 0 };

    // for slow modes
    private final Supplier<Distance> elevator_height;
    private boolean intake_throttle = false;
    private boolean climb_throttle = false;

    // reef assist
    public boolean elevator_assist_left = false;
    public boolean elevator_assist_right = false;
    private reef_assist_data leftLL_assist_data = new reef_assist_data(), rightLL_assist_data = new reef_assist_data();
    public int reef_assist_id = 6;
    private Translation2d reef_offset = new Translation2d();
    private Debouncer assist_debounce_x = new Debouncer(0.05);
    private Debouncer assist_debounce_y = new Debouncer(0.05);
    private Debouncer assist_close_debouncer = new Debouncer(0.2, DebounceType.kRising);
    private boolean cached_close_to_score_raw = false;
    private boolean cached_close_to_score = false;
    private boolean cached_ready_to_score = false;
    private ChassisSpeeds cached_assist_speeds_tele = new ChassisSpeeds();
    private LL auto_sttbolls_ll = config.LL_left;
    private int auto_sttbolls_id = 0;

    public final dummy strafe_subsystem, omega_subsystem;
    // IMPORTANT: dts is command dts, not swerve dts. Controller sets speeds in commands, wich run at default_period.
    protected final drivetrain_controller theta_con_rad = new drivetrain_controller(config.swerve.theta_config, constants.control_dts);
    protected final drivetrain_controller x_con_m = new drivetrain_controller(config.swerve.strafe_config, constants.control_dts);
    protected final drivetrain_controller y_con_m = new drivetrain_controller(config.swerve.strafe_config, constants.control_dts);
    protected final drivetrain_controller x_con_sttbolls_auton_m = new drivetrain_controller(config.swerve.sttbolls_auton_x_config, constants.control_dts);
    protected final drivetrain_controller y_con_sttbolls_auton_m = new drivetrain_controller(config.swerve.sttbolls_auton_y_config, constants.control_dts);
    protected final drivetrain_controller x_con_sttbolls_tele_m = new drivetrain_controller(config.swerve.sttbolls_tele_config, constants.control_dts);
    protected final drivetrain_controller y_con_sttbolls_tele_m = new drivetrain_controller(config.swerve.sttbolls_tele_config, constants.control_dts);

    private final String turning_bias_pref_key = "swerve_turning_bias";
    private final String reef_assist_key = "reef_assist";

    public swerve(TimedRobot robot, Supplier<Distance> elevator_height) {
        super(robot);
        strafe_subsystem = new dummy();
        omega_subsystem = new dummy();

        this.elevator_height = elevator_height;

        theta_con_rad.setTolerance(Units.degreesToRadians(1));

        Preferences.initDouble(turning_bias_pref_key, 0.6);

        robot.addPeriodic(this::periodic, constants.control_dts);

        strafe_subsystem.setDefaultCommand(
            tele_swerve_strafe(bindings.ctrl_strafe));

        omega_subsystem.setDefaultCommand(
            tele_swerve_omega(bindings.ctrl_turn)
        );

        Preferences.initBoolean(reef_assist_key, true);
    }

    private void periodic() {
        // probably want a different turn bias for auto and teleop, and driver preference
        if(DriverStation.isAutonomous()) {
            kin.clamp(desired_field_relative_speeds, 0.7);
        } else {
            kin.clamp(desired_field_relative_speeds, 0.6);
        }
        var slewed = desired_field_relative_speeds.times(1); // copy speeds
        var field_speeds = get_field_relative_speeds();
        if(DriverStation.isAutonomous()) {
            slew_auto.calculate(slewed, field_speeds);
        } else {
            slew_tele.calculate(slewed, field_speeds);
        }
        set_swerve_lowlevel(slewed);

        var heading = get_heading();
        var heading_rate = get_heading_rate();
        handle_vision_pose(config.LL_left, 0, heading, heading_rate);
        handle_vision_pose(config.LL_right, 1, heading, heading_rate);

        var pos = math_utils.tag_translation2d(config.LL_left, 0);
        SmartDashboard.putNumber("limelight_raw_left_x", pos.getX());
        SmartDashboard.putNumber("limelight_raw_left_y", pos.getY());

        pos = math_utils.tag_translation2d(config.LL_right, 0);
        SmartDashboard.putNumber("limelight_raw_right_x", pos.getX());
        SmartDashboard.putNumber("limelight_raw_right_y", pos.getY());

        SignalLogger.writeBoolean("swerve/close_to_score", cached_close_to_score);

        if(DriverStation.isTeleop()) {
            cache_reef_zone();
            cache_reef_decision(heading);
        }
    }

    private void handle_vision_pose(LL ll, int index, Rotation2d heading, AngularVelocity heading_rate) {
        LimelightHelpers.SetRobotOrientation(ll.name, heading.getDegrees(), 
            heading_rate.in(DegreesPerSecond), 0, 0, 0, 0);
        if(DriverStation.isAutonomousEnabled()) {
            return;
        }
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll.name);

        if(reject_vision_updates) {
            return;
        }

        cam_pose[index] = mt2.pose;
        field.getObject(ll.name).setPose(cam_pose[index]);
        ctre_log_pose(ll.name, cam_pose[index]);

        if(mt2.tagCount == 0) {
            return;
        }
        if(Math.abs(heading_rate.in(DegreesPerSecond)) > 360) {
            return;
        }
        if(mt2.avgTagDist > 3 || mt2.avgTagArea < 0.1) {
            return;
        }

        if(math_utils.close_enough(last_vision_timestamps[index], mt2.timestampSeconds, 0.02)) {
            // repeat vision measurement
        } else {
            add_vision_measurement(new Pose2d(cam_pose[index].getTranslation(), heading), mt2.timestampSeconds, mt2_st_devs);
            last_vision_timestamps[index] = mt2.timestampSeconds;
        }
    }

    private void reset_pose_now(LL ll) {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll.name);
        if(mt2 != null && mt2.tagCount > 0 && mt2.pose != null) {
            reset_pose(new Pose2d(mt2.pose.getTranslation(), get_heading()));
        }
    }

    public Command reset_pose(LL ll) {
        return Commands.runOnce(() -> {
            reset_pose_now(ll);
        });
    }

    private class reef_assist_data {
        Translation2d offset = new Translation2d();
        // Rotation2d snap_angle = new Rotation2d();
        // Rotation2d snap_err = new Rotation2d();
        boolean valid = false;
        // int tag_id = 0;
        Debouncer debouncer = new Debouncer(0.07, DebounceType.kFalling);

        void update(LL ll, Rotation2d heading, Rotation2d snap_err) {
            boolean tv = LimelightHelpers.getTV(ll.name);
            int tag_id = (int)LimelightHelpers.getFiducialID(ll.name);
            boolean raw_valid = false;
            if( tv && tag_id == reef_assist_id ) {
                // this.tag_id = tag_id;
                // var tag = constants.tags.getTagPose(tag_id).orElse(new Pose3d());
                // snap_angle = tag.getRotation().toRotation2d().plus(Rotation2d.k180deg);
                // snap_err = snap_angle.minus(heading);
                offset = math_utils.tag_translation2d(ll, snap_err.getDegrees());
                raw_valid = Math.abs(snap_err.getDegrees()) < 40;
            }
            valid = debouncer.calculate(raw_valid);
        }
    }


    private void cache_reef_zone() {
        var pose = get_pose().getTranslation();//.plus(math_utils.trans(get_commanded_speeds().times(0.05)));
        boolean red = pose.getX() > constants.tags.getFieldLength() / 2.0;

        int[] reef_tags = red ? constants.red_reef_tags : constants.blue_reef_tags;

        var current_assist = constants.tags.getTagPose(reef_assist_id).get();
        double closest_dist = pose.getDistance(math_utils.trans2d(current_assist));

        for(int id : reef_tags) {
            var tag_pose = constants.tags.getTagPose(id);
            var dist = pose.getDistance(math_utils.trans2d(tag_pose.get()));
            if(dist < closest_dist - 0.1) { // hysteresis
                closest_dist = dist;
                reef_assist_id = id;
            }
        }
    }

    private void cache_reef_decision(Rotation2d heading) {
        var tag = constants.tags.getTagPose(reef_assist_id).get();
        var snap_angle = Rotation2d.fromRadians(tag.getRotation().getMeasureZ().in(Radians)).plus(Rotation2d.k180deg);
        var snap_err = snap_angle.minus(heading);

        leftLL_assist_data.update(config.LL_left, heading, snap_err);
        rightLL_assist_data.update(config.LL_right, heading, snap_err);

        cached_close_to_score_raw = false;
        cached_ready_to_score = false;

        boolean correct_camera = (elevator_assist_left && rightLL_assist_data.valid) || (elevator_assist_right && leftLL_assist_data.valid);

        var target = new Translation2d(0.23, 0);
        var pose = get_pose().getTranslation();
        if(elevator_assist_left) {
            if(rightLL_assist_data.valid) {
                reef_offset = rightLL_assist_data.offset;
            } else {
                reef_offset = tag.getTranslation().toTranslation2d().minus(pose).rotateBy(heading.unaryMinus()).minus(config.LL_right.mount_offset.toTranslation2d());
                reef_offset = new Translation2d(reef_offset.getX(), -reef_offset.getY());
            }
        } else if(elevator_assist_right) {
            if(leftLL_assist_data.valid) {
                reef_offset = leftLL_assist_data.offset;
            } else {
                reef_offset = tag.getTranslation().toTranslation2d().minus(pose).rotateBy(heading.unaryMinus()).minus(config.LL_left.mount_offset.toTranslation2d());
                reef_offset = new Translation2d(reef_offset.getX(), -reef_offset.getY());
            }
        }

        SmartDashboard.putNumber("reef assist tag", reef_assist_id);
        SmartDashboard.putNumber("reef ofset x", reef_offset.getX());
        SmartDashboard.putNumber("reef ofset y", reef_offset.getY());

        double max_speed = 4;
        double max_decel = 3;
        if (elevator_height.get().gt(elevator_state.L2.height.plus(Inches.of(1)))) {
            max_speed = 1.0;
        }
        if(elevator_height.get().gt(elevator_state.L3.height.minus(Inches.of(1)))) {
            max_speed = 1.2;
            max_decel = 2;
        }
        if(elevator_height.get().gt(elevator_state.L4.height.minus(Inches.of(1)))) {
            max_speed = 0.4;
            max_decel = 1;
        }
        calc_assist_tele(max_speed, max_decel, 0, target, reef_offset, cached_assist_speeds_tele, snap_err);

        SmartDashboard.putNumber("assist radianspersecond", cached_assist_speeds_tele.omegaRadiansPerSecond);
        SmartDashboard.putBoolean("correct camera", correct_camera);

        if(!correct_camera) {
            cached_ready_to_score = false;
        }

        if( elevator_assist_left == elevator_assist_right ) { // both or neither assist should result in no assist
            cached_assist_speeds_tele.vxMetersPerSecond = 0;
            cached_assist_speeds_tele.vyMetersPerSecond = 0;
            cached_assist_speeds_tele.omegaRadiansPerSecond = 0;
            cached_close_to_score_raw = false;
            cached_ready_to_score = false;
        }

        cached_close_to_score = assist_close_debouncer.calculate(cached_close_to_score_raw);
    }

    public boolean reef_align_has_target() {
        return ( LimelightHelpers.getTV(config.LL_left_name) && LimelightHelpers.getFiducialID(config.LL_left_name) == reef_assist_id) 
            || (LimelightHelpers.getTV(config.LL_right_name) && LimelightHelpers.getFiducialID(config.LL_right_name) == reef_assist_id);
    }

    private static Translation2d prescore_threshold() {
        if(bindings.height_for_the_elevator_coral == elevator_state.L2) {
            return new Translation2d(Units.inchesToMeters(48), Units.inchesToMeters(24));
        }
        if(bindings.height_for_the_elevator_coral == elevator_state.L3) {
            return new Translation2d(Units.inchesToMeters(24), Units.inchesToMeters(12));
        }
        return new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(3));
    }

    private ChassisSpeeds calc_assist_tele(double max_speed, double max_decel, double min, Translation2d target, Translation2d offset, ChassisSpeeds out, Rotation2d snap_err) {
        final var close = prescore_threshold();
        final double y_tolerance = Units.inchesToMeters(bindings.height_for_the_elevator_coral == elevator_state.L4 ? 0.8 : 0.85);
        final double x_tolerance = Units.inchesToMeters(1.5);
        double x_speed = min;
        double y_speed = 0;
        cached_ready_to_score = true;

        double x_dist = offset.getX();
        double y_dist = offset.getY();
        y_speed = math_utils.clamp(-max_speed, max_speed, y_con_sttbolls_tele_m.calculate(target.getY(), y_dist, 0, max_decel));
        var x_lim = Math.sqrt( Math.max(math_utils.sq(max_speed) - math_utils.sq(y_speed * 1.0), 0) );
        x_speed = math_utils.clamp(-x_lim, x_lim, x_con_sttbolls_tele_m.calculate(-target.getX(), -x_dist, 0.0, max_decel));

        if(math_utils.close_enough(target.getX(), offset.getX(), close.getX()) && math_utils.close_enough(target.getY(), offset.getY(), close.getY())) {
            cached_close_to_score_raw = true;
        }

        if(assist_debounce_y.calculate(Math.abs(y_dist - target.getY()) < y_tolerance)) {
            y_speed = 0;
        } else {
            cached_ready_to_score = false;
        }
        if(assist_debounce_x.calculate(Math.abs(x_dist - target.getX()) < x_tolerance)) {
            x_speed = 0;
        } else {
            cached_ready_to_score = false;
        }

        
        if(math_utils.abs(snap_err).getDegrees() > 20) {
            x_speed = 0;
            y_speed = 0;
            cached_ready_to_score = false;
            cached_close_to_score_raw = false;
        }
        out.vxMetersPerSecond = x_speed;
        out.vyMetersPerSecond = y_speed;
        return out;
    }

    public void set_strafe(ChassisSpeeds speeds) {
        desired_field_relative_speeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
        desired_field_relative_speeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
    }

    public void set_turn(double omega_radps) {
        desired_field_relative_speeds.omegaRadiansPerSecond = omega_radps;
    }

    public void set_swerve(ChassisSpeeds speeds) {
        set_strafe(speeds);
        set_turn(speeds.omegaRadiansPerSecond);
    }

    public boolean is_assisting() {
        return elevator_assist_left || elevator_assist_right;
    }

    public boolean close_to_score() {
        return cached_close_to_score;
    }

    private Debouncer debounce_pitch = new Debouncer(0.5);
    public boolean ready_to_score() {
        return cached_ready_to_score;// && debounce_pitch.calculate(Math.abs(get_pitch().in(Degrees)) < 1.5);
    }

    public boolean theta_within(double tol_deg) {
        return theta_con_rad.within(Units.degreesToRadians(tol_deg));
    }

    public Command cmd_reset_pose(Pose2d pose) {
        return Commands.runOnce(() -> {
            reset_pose(pose);
        }).ignoringDisable(true);
    }

    public Command cmd_reset_pose(Translation2d pose) {
        return Commands.runOnce(() -> {
            reset_pose(new Pose2d(pose, get_heading()));
        }).ignoringDisable(true);
    }

    public void reject_vision(boolean do_reject) {
        reject_vision_updates = do_reject;
    }

    public Command cmd_reject_vision(boolean do_reject) {
        return Commands.runOnce(() -> {
            reject_vision(do_reject);
        });
    }

    public Command blind(Command to_wrap) {
        return to_wrap.alongWith(Commands.runOnce(() -> {
            reject_vision(true);
        })).finallyDo(() -> {
            reject_vision(false);
        });
    }

    @Override
    public void reset_pose(Pose2d pose) {
        super.reset_pose(pose);
    }

    public Command make_form_x_command() {
        ChassisSpeeds stop = new ChassisSpeeds(0, 0, 0);
        return baselock_when_stop(strafe_field_relative(() -> stop).alongWith(turn(() -> 0.0)));
    }

    public Command baselock_when_stop(Command command) {
        return Commands.runOnce(() -> {
            form_x_when_stopped(true);
        })
        .andThen(command)
        .finallyDo(() -> {
            form_x_when_stopped(false);
        });
    }

    public Command baselock_when_stop() {
        return baselock_when_stop(Commands.idle());
    }

    public Command strafe_field_relative(Supplier<ChassisSpeeds> strafe_supplier) {
        return Commands.run(() -> {
            set_strafe(strafe_supplier.get());
        }, strafe_subsystem)
        .finallyDo(() -> {
            set_strafe(new ChassisSpeeds(0, 0, 0));
        });
    }

    public Command strafe_robot_relative(Supplier<ChassisSpeeds> strafe_supplier) {
        return strafe_field_relative(() -> ChassisSpeeds.fromRobotRelativeSpeeds(strafe_supplier.get(), get_heading()));
    }

    public Command strafe_omega(Supplier<ChassisSpeeds> supplier) {
        return strafe_field_relative(supplier).alongWith(turn(() -> supplier.get().omegaRadiansPerSecond));
    }

    public Command strafe_to_point(final Supplier<Translation2d> point, final double max_vel, final double tolerance, final double through_vel) {
        return Commands.runOnce(() -> {
            x_con_m.setTolerance(tolerance);
            x_con_m.reset();
            ctre_log_pose("strafe_to_point", point.get());
        })
        .alongWith(strafe_field_relative(() -> {
            var err = point.get().minus(get_pose().getTranslation());
            double err_len = err.getNorm();
            double speed = math_utils.clamp(-max_vel, max_vel, 
                x_con_m.calculate(0, -err_len, through_vel)
            );
            if(err_len < 0.02) {
                speed = 0;
            }
            var output = err.div(err_len == 0 ? 1 : err_len).times(speed);
            return new ChassisSpeeds(output.getX(), output.getY(), 0);
        }).until(() -> point.get().getDistance(get_pose().getTranslation()) <= tolerance));
    }

    public boolean close_to_reef() {
        var tag = constants.tags.getTagPose(reef_assist_id).get();
        var tag_pos = tag.getTranslation().toTranslation2d();
        var tag_angle = Rotation2d.fromRadians(tag.getRotation().getMeasureZ().in(Radians));
        var offset = math_utils.find_translation(tag_angle, 0.5);
        return tag_pos.plus(offset).getDistance(get_pose().getTranslation()) < 0.8;
    }

    @Override
    public Command strafe_to_point(final Translation2d point, final double max_vel, final double tolerance, final double through_vel) {
        return strafe_to_point(() -> point, max_vel, tolerance, through_vel);
    }
    public Command strafe_to_fixed_point(Translation2d point, final double max_vel, final double tolerance, final double through_vel) {
        return strafe_to_point(field_util.fix(point), max_vel, tolerance, through_vel);
    }

    @Override
    public Command strafe_line(Translation2d point, Rotation2d direction, double max_vel, double tolerance, final double through_vel) {
        return strafe_line(point, direction, max_vel, tolerance, through_vel, config.swerve.pid_line_y_weight);
    }

    public Command strafe_line(Translation2d point, Rotation2d direction, double max_vel, double tolerance, final double through_vel, final double y_weight) {
        return strafe_line(point, direction, max_vel, tolerance, through_vel, y_weight, config.swerve.strafe_config.max_accel);
    }

    public Command strafe_line(Translation2d point, Rotation2d direction, double max_vel, double tolerance, final double through_vel, final double y_weight, double accel) {
        return Commands.runOnce(() -> {
            y_con_m.reset();
            x_con_m.reset();
            ctre_log_pose("strafe_to_point", point, direction.getDegrees());
        })
        .andThen(
            strafe_field_relative(() -> {
                var err = point.minus(get_pose().getTranslation()).rotateBy(direction.unaryMinus());
                var y_out = math_utils.clamp(-max_vel, max_vel, y_con_m.calculate(0, -err.getY(), 0, accel));
                if(Math.abs(y_out) < 0.02) {
                    y_out = 0;
                }
                var x_lim = Math.sqrt( math_utils.sq(max_vel) - math_utils.sq(y_out * y_weight) );
                var x_out = math_utils.clamp(-x_lim, x_lim, x_con_m.calculate(0, -err.getX(), through_vel, accel));
                if(Math.abs(x_out) < 0.02) {
                    x_out = 0;
                }
                var output = new Translation2d(
                    x_out,
                    y_out
                ).rotateBy(direction);
                output = math_utils.clamp(output, max_vel);
                return new ChassisSpeeds(output.getX(), output.getY(), 0);
            }).until(() -> math_utils.close_enough(point, get_pose().getTranslation(), tolerance))
        );
    }

    public Command strafe_line_fixed(Translation2d point, Rotation2d direction, double max_vel, double tolerance, final double through_vel) {
        return strafe_line(field_util.fix(point), field_util.fix(direction), max_vel, tolerance, through_vel);
    }

    public Command strafe_arc(Translation2d point, Translation2d arc_center, boolean clockwise, double max_vel, double tolerance) {
        final var arc_radius = point.minus(arc_center).getNorm();
        final var target_angle = point.minus(arc_center).getAngle();
        return strafe_field_relative(() -> {
            var pose = get_pose().getTranslation();
            var curr_dist = pose.minus(arc_center);
            var curr_angle = curr_dist.getAngle();
            var target_dist = new Translation2d(arc_radius, curr_angle);
            var angle_err = math_utils.err(target_angle, curr_angle, clockwise);
            var dist_err = target_dist.minus(curr_dist);
            var x_lim = math_utils.remap_clamp(0.2, 0.9, Math.abs(dist_err.getNorm()), max_vel, 0);
            var err_arc = angle_err.getRadians() * curr_dist.getNorm();
            var arc_speed = math_utils.clamp(-x_lim, x_lim, x_con_m.calculate(0, -err_arc, 0));
            var dist_speed = math_utils.clamp(-max_vel, max_vel, y_con_m.calculate(0, -dist_err.getNorm(), 0));
            var arc_output = curr_dist.div(curr_dist.getNorm()).rotateBy(Rotation2d.fromDegrees(clockwise ? -90 : 90)).times(arc_speed);
            var center_output = curr_dist.div(curr_dist.getNorm()).times(-dist_speed);
            var output = arc_output.plus(center_output);
            return new ChassisSpeeds(output.getX(), output.getY(), 0);
        }).until(() -> math_utils.close_enough(point, get_pose().getTranslation(), tolerance));
    }

    public Command strafe_arc_fixed(Translation2d point, Translation2d arc_center, boolean clockwise, double max_vel, double tolerance) {
        return strafe_arc(field_util.fix(point), field_util.fix(arc_center), clockwise, max_vel, tolerance);
    }

    public Command strafe_sttbolls(LL ll, int tag_id, double y_bias) {
        final double y_tolerance = Units.inchesToMeters(0.625);
        final double x_tolerance = Units.inchesToMeters(2.0);
        final double y_prescore = Units.inchesToMeters(10);
        final double x_prescore = Units.inchesToMeters(30);
        final double distance_setpoint = 0.23;
        final Translation2d target = new Translation2d(distance_setpoint, 0);
        Debouncer debounce_x = new Debouncer(0.05);
        Debouncer debounce_y = new Debouncer(0.05);
        Debouncer debounce_theta = new Debouncer(0.05);
        Debouncer debounce_patch = new Debouncer(1.5);
        var snap_to = constants.tags.getTagPose(tag_id).get().getRotation().toRotation2d().plus(Rotation2d.k180deg);
        SmartDashboard.putBoolean("close", false);
        SmartDashboard.putBoolean("x_ready", false);
        SmartDashboard.putBoolean("y_ready", false);
        return Commands.runOnce(() -> {
            x_con_sttbolls_auton_m.reset();
            y_con_sttbolls_auton_m.reset();
            cached_close_to_score_raw = false;
            cached_close_to_score = false;
            cached_ready_to_score = false;
            auto_sttbolls_id = tag_id;
            auto_sttbolls_ll = ll;
        })
        .andThen(
            strafe_robot_relative(() -> {
                // reset_pose_now(auto_sttbolls_ll);
                // return cached_assist_speeds_tele;
                double x_speed = 0.5;
                double y_speed = 0;
                var heading = get_heading();
                var err_degs = snap_to.minus(heading).getDegrees();
                if(LimelightHelpers.getTV(ll.name)) {
                    Translation2d offset = math_utils.tag_translation2d(ll, err_degs);
                    double x_dist = offset.getX();
                    double y_dist = offset.getY();
                    y_speed = y_con_sttbolls_auton_m.calculate(0, y_dist, 0);
                    var x_lim = Math.sqrt( Math.max(math_utils.sq(max_speed_mps) - math_utils.sq(y_speed * y_bias), 0) );
                    x_speed = math_utils.clamp(-x_lim, x_lim, x_con_sttbolls_auton_m.calculate(-distance_setpoint, -x_dist, 0));

                    boolean y_ready = debounce_y.calculate(Math.abs(y_dist) < y_tolerance);
                    boolean x_ready = debounce_x.calculate(Math.abs(x_dist - distance_setpoint) < x_tolerance);
                    // if(y_ready) {
                    //     y_speed = 0;
                    // } 
                    if(x_ready) {
                        x_speed = 0;
                    }
                    var remapped_x_xlose = math_utils.remap(15, 4, Math.abs(err_degs), 0, x_prescore);
                    var remapped_y_xlose = math_utils.remap(15, 4, Math.abs(err_degs), 0, y_prescore);
                    boolean x_close = Math.abs(x_dist - distance_setpoint) < remapped_x_xlose;
                    boolean y_close = Math.abs(y_dist) < remapped_y_xlose;
                    SignalLogger.writeBoolean("swerve/x_close", x_close);
                    SignalLogger.writeBoolean("swerve/y_close", y_close);
                    if(x_close && y_close) {
                        cached_close_to_score = true;
                    }
                    if(x_ready && y_ready && cached_close_to_score) {
                        cached_ready_to_score = true;
                    }
                    SmartDashboard.putBoolean("close", cached_close_to_score);
                    SmartDashboard.putBoolean("x_ready", x_ready);
                    SmartDashboard.putBoolean("y_ready", y_ready);

                    reset_pose_now(ll);
                }
                var spit_whatever = debounce_patch.calculate(!LimelightHelpers.getTV(ll.name));
                if(spit_whatever) {
                    cached_close_to_score = true;
                    cached_ready_to_score = true;
                }

                if(!debounce_theta.calculate(theta_within(10))) {
                    x_speed = math_utils.remap(25, 10, err_degs, 0, x_speed);
                    y_speed = math_utils.remap(25, 10, err_degs, 0, y_speed);
                }
                if(RobotBase.isSimulation()) {
                    x_speed = 0.3;
                }
                return new ChassisSpeeds(x_speed, y_speed, 0);
            })
        ).finallyDo(() -> {
            cached_close_to_score = false;
            cached_ready_to_score = false;
        });
    }

    static boolean is_reef(int id) {
        return (6 <= id && id <= 11) || (17 <= id && id <= 22);
    }

    public Command turn(Supplier<Double> omega_rad_supplier) {
        return Commands.run(() -> {
            set_turn(omega_rad_supplier.get());
        }, omega_subsystem)
        .finallyDo(() -> {
            set_turn(0.0);
        });
    }

    public Command maintain_heading() {
        var wrapper = new Object() { Rotation2d angle; };
        return Commands.sequence(
            Commands.runOnce(() -> {
                wrapper.angle = get_heading();
            }),
            snap(() -> wrapper.angle.getDegrees())
        );
    }

    @Override
    public Command snap(double theta_deg) {
        return snap_with_omega(() -> new theta_omega(Units.degreesToRadians(theta_deg), 0.0));
    }
    public Command fixed_snap(double theta_deg) {
        return snap(field_util.fix(Rotation2d.fromDegrees(theta_deg)).getDegrees());
    }
    public Command snap(Supplier<Double> theta_deg) {
        return snap_with_omega(() -> new theta_omega(Units.degreesToRadians(theta_deg.get()), 0));
    }

    public Command snap_deadzone(Supplier<Double> theta_deg, double deadzone, double debounce) {
        Debouncer deb = new Debouncer(debounce);
        return turn(() -> {
            var pid_out = theta_con_rad.calculate(Units.degreesToRadians(theta_deg.get()), get_heading().getRadians());
            if(deb.calculate(theta_con_rad.within(Units.degreesToRadians(deadzone)))) {
                return 0.0;
            }
            return pid_out;
        });
    }

    class theta_omega {
        double theta_rad, omega_radps;
        theta_omega(double theta_deg, double omega_degps) {
            this.theta_rad = theta_deg;
            this.omega_radps = omega_degps;
        }
    }

    public Command snap_with_omega(Supplier<theta_omega> supplier) {
        Debouncer debounce = new Debouncer(0.1);
        return turn(() -> {
            var thetas = supplier.get();
            var feedback = theta_con_rad.calculate(thetas.theta_rad, get_heading().getRadians(), thetas.omega_radps);
            return feedback;
        });
    }

    public Command snap_trans(Supplier<Translation2d> look_at, double offset_deg) {
        theta_omega thetas = new theta_omega(offset_deg, offset_deg);
        return snap_with_omega(() -> {
            var pose = get_pose().getTranslation();
            var target = look_at.get();
            var diff = target.minus(pose);
            double target_rad = diff.getAngle().getRadians() + Units.degreesToRadians(offset_deg);
            var speeds = math_utils.trans(get_field_relative_speeds()).times(constants.control_dts);
            var next_pose = pose.plus(speeds);
            var next_diff = target.minus(next_pose);
            var next_target = next_diff.getAngle().getRadians() + Units.degreesToRadians(offset_deg);
            var ang_vel = (next_target - target_rad) / constants.control_dts;
            var vel_kP = 1;
            thetas.theta_rad = target_rad;
            thetas.omega_radps = ang_vel * vel_kP;
            return thetas;
        });
    }
    public Command snap_trans(Translation2d look_at, double offset_deg) {
        return snap_trans(() -> look_at, offset_deg);
    }

    public Command intake_throttle() {
        return Commands.startEnd(() -> {
            intake_throttle = true;
        }, () -> {
            intake_throttle = false;
        });
    }

    public Command climb_throttle() {
        return Commands.startEnd(() -> {
            climb_throttle = true;
        }, () -> {
            climb_throttle = false;
        });
    }

    private final double[] reef_angles = { 0, 60, 120, 180, -60, -120, -180 };

    public Command snap_reef_pose() {
        return snap(() -> {
            return Units.radiansToDegrees( constants.tags.getTagPose(reef_assist_id).get().getRotation().getZ() ) + 180;
        });
    }

    public boolean close_to_any_reef_angle() {
        var heading = MathUtil.inputModulus(get_heading().getDegrees(), -180, 180);
        for(double reef_angle : reef_angles) {
            double angle_diff = heading - reef_angle;
            if(Math.abs(angle_diff) < 3) {
                return true;
            }
        }
        return false;
    }

    public Command tele_swerve_strafe(Supplier<Translation2d> ctrl_strafe) {
        // final String pref_key = "drive_speed", pref_key_intake_assist = "intake_assist";

        // Preferences.initBoolean(pref_key_intake_assist, false);
        // Preferences.initDouble(pref_key, 10);

        return strafe_field_relative(() -> {
            double throttle = 1.0;
            if(intake_throttle) {
                throttle *= 0.7;
            }
            if(climb_throttle) {
                throttle *= 0.5;
            }
            Distance elevator_height = this.elevator_height.get();
            boolean elevator_high = elevator_height.gt(constants.elevator_state.L2.height.plus(Inches.of(1)));
            boolean elevator_very_high = elevator_height.gt(constants.elevator_state.L3.height.plus(Inches.of(1)));
            if(elevator_high) {
                throttle *= 0.5;
            }
            if(elevator_very_high) {
                throttle *= 0.6;
            }
            double pref = 10;
            double scale = math_utils.remap(1, 10, pref, constants.swerve.max_speed_mps*0.1, constants.swerve.max_speed_mps);
            var speeds = ctrl_strafe.get().times(scale * throttle);

            int flip = robot.is_red() ? -1 : 1;

            var chassis_speeds = new ChassisSpeeds(
                speeds.getX() * flip,
                speeds.getY() * flip,
                0);

                cached_joystick_speeds.vxMetersPerSecond = chassis_speeds.vxMetersPerSecond;
                cached_joystick_speeds.vyMetersPerSecond = chassis_speeds.vyMetersPerSecond;

            if(elevator_high) {
                elevator_slew.calculate(chassis_speeds, get_field_relative_speeds(), elevator_very_high ? 0.5 : 1);
            }
            if( Preferences.getBoolean(reef_assist_key, true) ) {
                var assist_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(cached_assist_speeds_tele, get_heading());
                if(math_utils.hypot(cached_joystick_speeds) < 0.1 
                        || math_utils.angle_between(cached_joystick_speeds, assist_speeds).abs(Degrees) < 65) {
                    return assist_speeds.plus(cached_joystick_speeds.times(0.1));
                }
            }

            return chassis_speeds;
        });
    }

    public Command tele_swerve_omega(Supplier<Double> ctrl_turn) {
        // final String pref_key = "turn_speed";
        // Preferences.initDouble(pref_key, 5.9);
        double max_omega = get_max_chassis_radps() * 0.95;

        return turn(() -> {
            double throttle = 1.0;
            if(climb_throttle) {
                throttle *= 0.5;
            }
            double theta_in = ctrl_turn.get();
            double pref = 6.7;//Preferences.getDouble(pref_key, 7);
            double scale = math_utils.remap(1, 10, pref, max_omega * 0.3, max_omega);
            cached_joystick_speeds.omegaRadiansPerSecond = theta_in * scale * throttle;
            double radpersec = cached_joystick_speeds.omegaRadiansPerSecond;// + cached_assist_speeds_tele.omegaRadiansPerSecond;
            if(Math.abs(radpersec) < Units.degreesToRadians(1)) {
                final double kD = 0.3;
                return MathUtil.applyDeadband(-get_heading_rate().in(RadiansPerSecond) * kD, Units.degreesToRadians(6));
            }
            return radpersec;
        });
    }

    public Command stop_motors() {
        return Commands.startEnd(() -> {
            stop_motors(true);
        }, () -> {
            stop_motors(false);
        }, strafe_subsystem, omega_subsystem);
    }
}
