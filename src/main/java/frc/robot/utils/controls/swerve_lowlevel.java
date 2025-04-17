package frc.robot.utils.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.constants.swerve.*;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.sim.swerve_mech2d;
import frc.robot.utils.configurable;
import frc.robot.utils.math_utils;
import frc.robot.utils.controls.swerve_kin2.module_state;

public abstract class swerve_lowlevel implements configurable {

    private final Pigeon2 pig = new Pigeon2(config.pigeon.can_id, config.pigeon.canbus);
    public final swerve_module[] modules;

    private final StatusSignal<Angle> heading_signal;
    private final StatusSignal<AngularVelocity>  heading_rate_signal_odom, heading_rate_signal;
    private final StatusSignal<Angle>  pitch_signal;
    // CTRE recommends seperate status signals for seperate threads

    protected final swerve_kin2 kin = new swerve_kin2(module_offsets, max_module_speed_mps);
    private SwerveDrivePoseEstimator pose_estimator;

    private final module_state[] form_x_states = kin.form_x();

    private final ReadWriteLock input_lock = new ReentrantReadWriteLock();
    private final ChassisSpeeds target_field_relative = new ChassisSpeeds();
    private boolean form_x_when_stopped = false;
    private boolean stop_motors = false;

    private final ReadWriteLock state_lock = new ReentrantReadWriteLock();
    private Pose2d pose = new Pose2d();
    private final ChassisSpeeds commanded_field_relative_speeds = new ChassisSpeeds();
    private final ChassisSpeeds current_field_relative_speeds = new ChassisSpeeds();

    private final module_state[] desired_states = {
        new module_state(0, 0, 0), 
        new module_state(0, 0, 0), 
        new module_state(0, 0, 0), 
        new module_state(0, 0, 0),
    };
    private final module_state[] current_states = {
        new module_state(0, 0, 0), 
        new module_state(0, 0, 0), 
        new module_state(0, 0, 0), 
        new module_state(0, 0, 0),
    };
    private final SwerveModulePosition[] module_positions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
    };

    protected final Field2d field = new Field2d();
    private final swerve_mech2d mech2d = new swerve_mech2d(kin, 3);

    private final odometry_thread odom_thread;

    /**
     * heavily inspired by (stolen from) from CTRE LegacySwerveDrivetrain
     */
    /* Perform swerve module updates in a separate thread to minimize latency */
    private class odometry_thread {
        private static final int START_THREAD_PRIORITY = 1; // Testing shows 1 (minimum realtime) is sufficient for tighter
                                                            // odometry loops.
                                                            // If the odometry period is far away from the desired frequency,
                                                            // increasing this may help

        private final Thread thread;
        private volatile boolean is_running = false;

        private final BaseStatusSignal[] all_signals = {
            heading_signal, heading_rate_signal_odom,
            modules[0].drive_pos_signal, modules[0].drive_vel_signal, modules[0].turn_pos_signal, modules[0].turn_vel_signal,
            modules[1].drive_pos_signal, modules[1].drive_vel_signal, modules[1].turn_pos_signal, modules[1].turn_vel_signal,
            modules[2].drive_pos_signal, modules[2].drive_vel_signal, modules[2].turn_pos_signal, modules[2].turn_vel_signal,
            modules[3].drive_pos_signal, modules[3].drive_vel_signal, modules[3].turn_pos_signal, modules[3].turn_vel_signal
        };

        private final MedianFilter peak_remover = new MedianFilter(3);
        private final LinearFilter low_pass = LinearFilter.movingAverage(50);
        private double last_time = 0;
        private double current_time = 0;
        private double average_loop_time = 0;
        private int successful_daqs = 0;
        private int failed_daqs = 0;

        private int last_thread_priority = START_THREAD_PRIORITY;
        private volatile int thread_priority_to_set = START_THREAD_PRIORITY;

        private odometry_thread() {
            thread = new Thread(this::run);
            /* Mark this thread as a "daemon" (background) thread
             * so it doesn't hold up program shutdown */
            thread.setDaemon(true);
        }

        private void start() {
            is_running = true;
            thread.start();
        }

        private void stop() {
            stop(0);
        }

        private void stop(long millis) {
            is_running = false;
            try {
                thread.join(millis);
            } catch (final InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        private void run() {
            BaseStatusSignal.setUpdateFrequencyForAll(odom_freq, all_signals);
            Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

            /* Run as fast as possible, our signals will control the timing */
            while (is_running) {
                if (!HootReplay.waitForPlaying(0.100)) {
                    continue;
                }

                /* Synchronously wait for all signals in drivetrain */
                /* Wait up to twice the period of the update frequency */
                StatusCode status;
                if (config.swerve.ctre_pro) {
                    status = BaseStatusSignal.waitForAll(2.0 / odom_freq, all_signals);
                } else {
                    /* Wait for the signals to update */
                    Timer.delay(1.0 / odom_freq);
                    status = BaseStatusSignal.refreshAll(all_signals);
                }

                try {
                    state_lock.writeLock().lock();
                    input_lock.readLock().lock();

                    last_time = current_time;
                    current_time = Utils.getCurrentTimeSeconds();
                    /* We don't care about the peaks, as they correspond to GC events, and we want the period generally low passed */
                    average_loop_time = low_pass.calculate(peak_remover.calculate(current_time - last_time));

                    /* Get status of first element */
                    if (status.isOK()) {
                        successful_daqs++;
                    } else {
                        failed_daqs++;
                    }

                    /* Now update odometry */
                    /* Keep track of the change in azimuth rotations */
                    for (int i = 0; i < 4; ++i) {
                        modules[i].get_odom(module_positions[i]);
                        modules[i].get_state(current_states[i]);
                    }
                    var yaw = BaseStatusSignal.getLatencyCompensatedValue(
                            heading_signal, heading_rate_signal_odom);

                    Rotation2d pig_rotation = Rotation2d.fromDegrees(yaw.in(Degrees));
                    // if(RobotBase.isSimulation()) {
                    //     pig_rotation = pose_estimator.getEstimatedPosition().getRotation().plus(Rotation2d.fromRadians(commanded_field_relative_speeds.omegaRadiansPerSecond * odom_dts));
                    // }

                    pose = pose_estimator.update(pig_rotation, module_positions);
                    var heading = pose.getRotation();

                    var speeds = kin.to_chassis_speeds(current_states);
                    var field_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, heading);
                    current_field_relative_speeds.vxMetersPerSecond = field_speeds.vxMetersPerSecond;
                    current_field_relative_speeds.vyMetersPerSecond = field_speeds.vyMetersPerSecond;
                    current_field_relative_speeds.omegaRadiansPerSecond = field_speeds.omegaRadiansPerSecond;

                    // handle_vision(heading);

                    ctre_log_pose("fused_pose", pose);

                    if(burning_motors() || stop_motors) {
                        modules[0].stop();
                        modules[1].stop();
                        modules[2].stop();
                        modules[3].stop();
                    } else {
                        var desired_chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(target_field_relative, heading);
                        if(swerve_kin2.is_moving(desired_chassis_speeds)) {
                            kin.to_module_states(desired_chassis_speeds, desired_states);
                            kin.desaturateWheelSpeeds(desired_states, desired_chassis_speeds);
                        } else if(form_x_when_stopped) {
                            for(int i = 0; i < 4; ++i) {
                                desired_states[i].speed_mps = form_x_states[i].speed_mps;
                                desired_states[i].theta_rad = form_x_states[i].theta_rad;
                                desired_states[i].omega_radps = form_x_states[i].omega_radps;
                            }
                        } else {
                            desired_states[0].speed_mps = 0;
                            desired_states[1].speed_mps = 0;
                            desired_states[2].speed_mps = 0;
                            desired_states[3].speed_mps = 0;
                        }
                
                        modules[0].set_state(desired_states[0]);
                        modules[1].set_state(desired_states[1]);
                        modules[2].set_state(desired_states[2]);
                        modules[3].set_state(desired_states[3]);
                    }
            

                    // swerve_module.set_state() can modify the desired state with cosine scale and optimize
                    // best to put the commanded chassis speeds calculation after the set_state()
                    var commanded_chassis_speeds = kin.to_chassis_speeds(desired_states);
                    var out = ChassisSpeeds.fromRobotRelativeSpeeds(commanded_chassis_speeds, heading);
                    commanded_field_relative_speeds.vxMetersPerSecond = out.vxMetersPerSecond;
                    commanded_field_relative_speeds.vyMetersPerSecond = out.vyMetersPerSecond;
                    commanded_field_relative_speeds.omegaRadiansPerSecond = out.omegaRadiansPerSecond;

                    pig.getSimState().addYaw(Radians.of(out.omegaRadiansPerSecond * odom_dts));
                    pig.getSimState().setAngularVelocityZ(RadiansPerSecond.of(out.omegaRadiansPerSecond));

                } finally {
                    state_lock.writeLock().unlock();
                    input_lock.readLock().unlock();
                }

                /**
                 * This is inherently synchronous, since lastThreadPriority
                 * is only written here and threadPriorityToSet is only read here
                 */
                if (thread_priority_to_set != last_thread_priority) {
                    Threads.setCurrentThreadPriority(true, thread_priority_to_set);
                    last_thread_priority = thread_priority_to_set;
                }
            }
        }

        public boolean odometryIsValid() {
            return successful_daqs > 2; // Wait at least 3 daqs before saying the odometry is valid
        }

        /**
         * Sets the DAQ thread priority to a real time priority under the specified priority level
         *
         * @param priority Priority level to set the DAQ thread to.
         *                 This is a value between 0 and 99, with 99 indicating higher priority and 0 indicating lower priority.
         */
        public void setThreadPriority(int priority) {
            thread_priority_to_set = priority;
        }
    }

    public swerve_lowlevel(TimedRobot robot) {
        modules = new swerve_module[]{
            new swerve_module(config.swerve.module_configs[0], robot),
            new swerve_module(config.swerve.module_configs[1], robot),
            new swerve_module(config.swerve.module_configs[2], robot),
            new swerve_module(config.swerve.module_configs[3], robot),
        };
        heading_signal = pig.getYaw();
        heading_rate_signal_odom = pig.getAngularVelocityZWorld();
        heading_rate_signal = heading_rate_signal_odom.clone();
        pitch_signal = pig.getPitch();

        pose_estimator = new SwerveDrivePoseEstimator(kin, pig.getRotation2d(), module_positions, pose);

        robot.addPeriodic(this::nt_periodic, RobotBase.isSimulation() ? 1.0/100.0 : 1.0/16.0, 1.0/32.0);
        odom_thread = new odometry_thread();
        odom_thread.start();

        mech2d.init();
        SmartDashboard.putData(field);
    }

    private void nt_periodic() {
        
        try {
            state_lock.readLock().lock();

            field.setRobotPose(pose);

            SmartDashboard.putBoolean("toasty warning", !toasty_motors());
            SmartDashboard.putBoolean("burning motors", burning_motors());
            mech2d.update(get_heading(), desired_states, current_states);
            if(config.dev) {
                var speeds = get_field_relative_speeds();
                SmartDashboard.putNumber("swerve/vx", speeds.vxMetersPerSecond);
                SmartDashboard.putNumber("swerve/vy", speeds.vyMetersPerSecond);
                SmartDashboard.putNumber("swerve/vt", speeds.omegaRadiansPerSecond);
                SmartDashboard.putNumber("swerve/x", pose.getX());
                SmartDashboard.putNumber("swerve/theta", get_heading().getRadians());
            }
        } finally {
            state_lock.readLock().unlock();
        }

    }

    public void add_vision_measurement(Pose2d pose, double timestamp, Matrix<N3, N1> st_devs) {
        try {
            state_lock.writeLock().lock();
            pose_estimator.addVisionMeasurement(pose, timestamp, st_devs);
        } finally {
            state_lock.writeLock().unlock();
        }
    }

    public double get_max_chassis_radps() {
        return kin.max_chassis_radps;
    }

    public double chassis_radius(int module) {
        return kin.module_mount_positions[module].getNorm();
    }

    public void set_swerve_lowlevel(ChassisSpeeds speeds) {
        try {
            input_lock.writeLock().lock();
            target_field_relative.vxMetersPerSecond = speeds.vxMetersPerSecond;
            target_field_relative.vyMetersPerSecond = speeds.vyMetersPerSecond;
            target_field_relative.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
        } finally {
            input_lock.writeLock().unlock();
        }
    }

    public Angle get_pitch() {
        return pitch_signal.refresh().getValue();
    }

    public void apply_current_limits(boolean auto) {
        if(auto) {
            apply_current_limits(config.swerve.auto_current_limits);
        } else {
            apply_current_limits(config.swerve.tele_current_limits);
        }
    }

    private void apply_current_limits(CurrentLimitsConfigs limits) {
        modules[0].apply_drive_current_limits(limits);
        modules[1].apply_drive_current_limits(limits);
        modules[2].apply_drive_current_limits(limits);
        modules[3].apply_drive_current_limits(limits);
    }

    public void coast() {
        modules[0].coast();
        modules[1].coast();
        modules[2].coast();
        modules[3].coast();
    }

    public void brake() {
        modules[0].brake();
        modules[1].brake();
        modules[2].brake();
        modules[3].brake();
    }

    public boolean toasty_motors() {
        return modules[0].toasty() 
            || modules[1].toasty() 
            || modules[2].toasty()
            || modules[3].toasty();
    }

    public boolean burning_motors() {
        return modules[0].burning()
            || modules[1].burning()
            || modules[2].burning()
            || modules[3].burning();
    }

    public void reset_encoders() {
        modules[0].reset_turning();
        modules[1].reset_turning();
        modules[2].reset_turning();
        modules[3].reset_turning();
    }

    public boolean has_abs() {
        return modules[0].has_abs()
                && modules[1].has_abs()
                && modules[2].has_abs()
                && modules[3].has_abs();
    }

    public void form_x_when_stopped(boolean do_form_x) {
        try {
            input_lock.writeLock().lock();
            form_x_when_stopped = do_form_x;
        } finally {
            input_lock.writeLock().unlock();
        }
    }

    public void stop_motors(boolean do_stop_motors) {
        try {
            input_lock.writeLock().lock();
            stop_motors = do_stop_motors;
        } finally {
            input_lock.writeLock().unlock();
        }
    }

    public SwerveModulePosition[] copy_module_positions() {
        final int length = module_positions.length;
        SwerveModulePosition[] copy = new SwerveModulePosition[length];
        for(int i = 0; i < length; ++i) {
            copy[i] = new SwerveModulePosition(module_positions[i].distanceMeters, Rotation2d.fromRadians(module_positions[i].angle.getRadians()));
        }
        return copy;
    }

    public ChassisSpeeds get_commanded_speeds() {
        try {
            state_lock.readLock().lock();
            return new ChassisSpeeds(
                commanded_field_relative_speeds.vxMetersPerSecond,
                commanded_field_relative_speeds.vyMetersPerSecond,
                commanded_field_relative_speeds.omegaRadiansPerSecond
            );
        } finally {
            state_lock.readLock().unlock();
        }
    }

    public void zero_heading(Rotation2d new_heading) {
        reset_pose(new Pose2d(get_pose().getTranslation(), new_heading));
    }

    public void reset_pose(Pose2d pose) {
        try {
            state_lock.writeLock().lock();
            pose_estimator.resetPosition(pig.getRotation2d(), module_positions, pose);
        } finally {
            state_lock.writeLock().unlock();
        }
    }

    public Pose2d get_pose() {
        try {
            state_lock.readLock().lock();
            return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().times(1));
        } finally {
            state_lock.readLock().unlock();
        }
    }

    protected static void ctre_log_pose(String name, Pose2d pose) {
        SignalLogger.writeDoubleArray(name, new double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    }
    protected static void ctre_log_pose(String name, Translation2d pose) {
        SignalLogger.writeDoubleArray(name, new double[]{pose.getX(), pose.getY(), 0.0});
    }
    protected static void ctre_log_pose(String name, Translation2d pose, double angle) {
        SignalLogger.writeDoubleArray(name, new double[]{pose.getX(), pose.getY(), angle});
    }

    public Rotation2d get_heading() {
        return get_pose().getRotation();
    }

    public AngularVelocity get_heading_rate() {
        return heading_rate_signal.refresh().getValue();
    }

    public Rotation2d get_raw_pigeon_rotation() {
        return pig.getRotation2d();
    }

    public ChassisSpeeds get_field_relative_speeds() {
        try {
            state_lock.readLock().lock();
            return new ChassisSpeeds(
                current_field_relative_speeds.vxMetersPerSecond,
                current_field_relative_speeds.vyMetersPerSecond,
                current_field_relative_speeds.omegaRadiansPerSecond
            );
        } finally {
            state_lock.readLock().unlock();
        }
    }

    public ChassisSpeeds get_speeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(get_field_relative_speeds(), get_heading());
    }

    @Override
    public void configure() {
        modules[0].configure();
        modules[1].configure();
        modules[2].configure();
        modules[3].configure();
        pig.getConfigurator().apply(config.swerve.pig_config);
        reset_encoders();
    }
    
}
