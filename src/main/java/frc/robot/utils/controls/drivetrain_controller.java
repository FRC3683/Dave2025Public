package frc.robot.utils.controls;

import edu.wpi.first.math.MathUtil;
import frc.robot.utils.clamped_pid;
import frc.robot.utils.math_utils;

/**
 * for when a mechanism (usually a drivetrain) already has PID running on the motors, so we close loop position with a linear
 * deceleration profile instead of another P loop on top, which would have non-optimal polynomial deceleration
 * 
 * we still use a P controller within a certain threshold, to avoid oscillation caused by discrete time-steps
 * 
 * The P loop having a small threshold allows it to be more aggressive too
 * 
 * 
 * THIS IS UNFORTUNATELY kinda not necessary at all... currently testing running swerve open-loop, ie. running drive motors 
 * in voltage mode, and a simple PID controller with that has identical performance. There's not the same fine-grain control
 * over acceleration with that approach but it's fine.
 */
public class drivetrain_controller {

    private final double max_vel, max_accel, epsilon, pid_threshold;
    private final clamped_pid pid;
    private double tolerance;

    public drivetrain_controller(configuration config, double control_dts) {
        pid = new clamped_pid(config.kP, config.kI, config.kD, config.max_vel, config.deadzone, control_dts);
        if(config.continuous_input) {
            pid.enableContinuousInput(config.min_input, config.max_input);
        }
        max_accel = config.max_accel;
        max_vel = config.max_vel;
        epsilon = config.epsilon;
        pid_threshold = config.pid_threshold;
    }

    public void reset() {
        pid.reset();
    }

    public boolean atSetpoint() {
        return within(tolerance);
    }

    public boolean within(double tolerance) {
        return MathUtil.isNear(0, pid.getError(), tolerance);
    }

    // the main math that would work on its own if we weren't limited to such big time steps (even 250hz doesnt seem like enough in sim)
    private double calc_ideal(double err, double target_vel, double target_accel) {
        if(err > 0) {
            err -= epsilon;
        } else {
            err += epsilon;
        }
        return math_utils.clamp(-max_vel, max_vel, Math.signum(err) * Math.sqrt(math_utils.sq(target_vel) + Math.abs(2 * target_accel * err))); // cool af
    }

    public double calculate(double target_pos, double current_pos, double target_vel, double target_accel) {
        target_vel = math_utils.clamp(-max_vel, max_vel, target_vel);
        target_accel = math_utils.clamp(-max_accel, max_accel, target_accel);
        var pid_out = pid.calculate(current_pos, target_pos);

        // discrete time-steps cause oscillation without this
        if(Math.abs(pid.getError()) < pid_threshold && Math.abs(target_vel) < 0.001) {
            return math_utils.clamp(-max_vel, max_vel, pid_out);
        }

        // vf^2 = vi^2 + 2ad
        // vi = sqrt(vf^2 - 2ad) * -signum(err)
        // vf = target_vel, vi = output, a = max_accel, d = err
        return calc_ideal(pid.getError(), target_vel, target_accel);
    }

    public double calculate(double target_pos, double current_pos) {
        return calculate(target_pos, current_pos, 0, max_accel);
    }

    public double calculate(double target_pos, double current_pos, double target_vel) {
        return calculate(target_pos, current_pos, target_vel, max_accel);
    }

    public double calculate(double error) {
        return calculate(0, error, 0);
    }

    public void setTolerance(double tol) {
        tolerance = tol;
    }

    public double get_tolerance() {
        return tolerance;
    }

    public static class configuration {
        public double max_vel, max_accel, epsilon, pid_threshold;
        public double kP, kI, kD;
        public boolean continuous_input = false;
        public double min_input, max_input;
        public double deadzone = 0;

        public configuration with_max_vel(double max_vel) {
            this.max_vel = max_vel;
            return this;
        }

        public configuration with_max_accel(double max_accel) {
            this.max_accel = max_accel;
            return this;
        }

        public configuration with_epsilon(double epsilon) {
            this.epsilon = epsilon;
            return this;
        }

        public configuration with_pid_threshold(double pid_threshold) {
            this.pid_threshold = pid_threshold;
            return this;
        }

        public configuration with_pid(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        public configuration with_continuous_input(double min_input, double max_input) {
            continuous_input = true;
            this.min_input = min_input;
            this.max_input = max_input;
            return this;
        }

        public configuration with_deadzone(double deadzone) {
            this.deadzone = deadzone;
            return this;
        }

    }
}
