// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.autos;
import frc.robot.commands.test_autos;
import frc.robot.utils.auto_selector;
import frc.robot.utils.configurable;
import frc.robot.utils.ctre_logs;
import frc.robot.utils.uptime;
import frc.robot.utils.voltage_warning;
import frc.robot.utils.auto_selector.auto;
import frc.robot.subsystems.climb;
import frc.robot.subsystems.ramp;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.end_effector;
import frc.robot.subsystems.swerve;

public final class robot extends TimedRobot {

    public final swerve swerve;
    public final climb climb;
    public final elevator elevator;
    public final end_effector end_effector;
    public final ramp ramp;
    public final leds leds;
    private final auto_selector selector;
    public final PowerDistribution pd;
    private Command auto_command;
    public static AtomicBoolean zerod = new AtomicBoolean(false);

    private final configurable[] configurables;

    public static boolean is_red() {
        var a = DriverStation.getAlliance();
        return a.isPresent() && a.get().equals(Alliance.Red);
    }

    public robot() {
        super(constants.control_dts);

        end_effector = new end_effector(this);
        elevator = new elevator(this, end_effector::lidar_sees_coral_raw);
        swerve = new swerve(this, elevator::get_height);
        climb = new climb(this);
        ramp = new ramp(this, elevator::get_height);
        configurables = new configurable[]{ swerve, elevator, end_effector, ramp, climb };

        // can_savior.init(config.can_loop);

        leds = new leds(this, swerve);
        pd = new PowerDistribution();

        selector = new auto_selector(
            new auto[]{
                // auto.from("wednesday",      () -> autos.red_barge(this),                    () -> autos.blue_barge(this)),
                auto.from("lefty loosey",   () -> autos.red_barge(this),                     () -> autos.blue_barge(this)),
                auto.from("stripped bolt",   () -> autos.red_center(this, config.LL_left),   () -> autos.blue_center(this, config.LL_left)),
                auto.from("righty tighty",  () -> autos.red_proc(this),                     () -> autos.blue_proc(this)),
                // auto.from("center_proc",    () -> autos.red_center(this, config.LL_right),  () -> autos.blue_center(this, config.LL_right)),
            }, new auto[]{
                auto.from("calibrate wheel radius", () -> test_autos.clibrate_wheel_radius(this)),
                auto.from("swerve PID tune",        () -> test_autos.straight(this, 4, constants.swerve.max_speed_mps)),
                auto.from("ballet calibration",     () -> test_autos.skew_test(this, 1.5, 2, 4)),
                auto.from("sttbolls",               () -> test_autos.sttbolls_test(swerve)),
                auto.from("testy test",             () -> autos.test(this),                     () -> autos.test(this)),
            }, swerve::reset_pose
        );

        var tags = constants.tags;
        tags.getTagPose(0); // force constants static initialization
    }

    @Override
    public void robotInit() {
        swerve.reset_encoders();
        bindings.configure_bindings(this);
        debug.add_dashboard_commands(this);
        selector.init();
        voltage_warning.set_thresholds(constants.voltage_warning_threshold_comp, constants.voltage_warning_threshold_prac);
        voltage_warning.add_nt_listener(this, pd, 2);
        // can_savior.begin(this);
        uptime.init(this);
        configurable.configure_all(leds.flag_configuring.run(), configurables);

        ctre_logs.init();

        new Thread(() -> {
            while(!zerod.get()) {
                try {
                    Thread.sleep(3000);
                    swerve.reset_pose(new Pose2d());
                    Thread.sleep(100);
                    zerod.set(true);
                } catch(Exception e) {}
            }
        }).start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("match_time", DriverStation.getMatchTime());
    }

    // boolean reset = true;
    // Timer timer = new Timer();
    @Override
    public void disabledInit() {
        // timer.restart();
    }

    @Override
    public void disabledPeriodic() {
        // if(reset && timer.get() > 1) {
            // swerve.reset_pose(new Pose2d());
            // reset = false;
        // }
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        auto_command = selector.get_auto_command();
        if(auto_command != null) {
            auto_command.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        swerve.apply_current_limits(false);
        elevator.apply_current_limits(false);
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        swerve.stop_motors().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
        ramp.cmd_stop().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
        // climb.reset_servo();
        Commands.waitSeconds(1).andThen(climb.open_servo().andThen(Commands.idle(climb).withTimeout(1.0))).schedule();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
