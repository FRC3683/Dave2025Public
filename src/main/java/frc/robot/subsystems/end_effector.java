package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.bindings;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.constants.elevator_state;
import frc.robot.sim.elevator_mech2d;
import frc.robot.utils.configurable;
import frc.robot.utils.lazy_ctre;
import frc.robot.utils.controls.dave_talon;

import static frc.robot.constants.end_effector.*;

public class end_effector extends SubsystemBase implements configurable {
    private final dave_talon roller_motor = new dave_talon(config.can_endeffector_roller);
    private final DigitalInput lidar = new DigitalInput(config.dio_end_effector_lidar);
    private final Debouncer lidar_debouncer = new Debouncer(0.03);
    private boolean lidar_sees_coral = false;
    private AngularVelocity target_feed = RotationsPerSecond.zero();
    public gamepiece last_intake = gamepiece.CORAL;
    private boolean coral_seated = false;
    private Timer l1_timer = new Timer();

    public boolean auto_flag = false;

    private final StatusSignal<AngularVelocity> roller_velocity_signal = roller_motor.getVelocity();
    private final StatusSignal<Angle> roller_position_signal = roller_motor.getPosition();
    private final StatusSignal<Double> roller_error_signal = roller_motor.getClosedLoopError();
    private final StatusSignal<Current> roller_amps_signal = roller_motor.getStatorCurrent();

    private final VelocityVoltage roller_req_velocity = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage roller_req_position = new PositionVoltage(0).withSlot(1);
    private final TorqueCurrentFOC hold_algae_req = new TorqueCurrentFOC(hold_ball_current).withMaxAbsDutyCycle(hold_ball_max_dutycycle);

    public end_effector(TimedRobot robot) {
        if(RobotBase.isSimulation()) {
            roller_motor.init_sim(robot, 250, DCMotor.getKrakenX60(1), 1, 0.0001);
        }

        l1_timer.restart();

        roller_motor.stopMotor();
        roller_motor.setPosition(0);

        setDefaultCommand(Commands.run(() -> {
            switch (last_intake) {
                case ALGAE:
                    roller_motor.setControl(hold_algae_req);
                    break;
                case CORAL:
                    set_feed(DegreesPerSecond.zero());
                    break;
            }
        }, this));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll( roller_velocity_signal, roller_position_signal, roller_error_signal, roller_amps_signal );
        lidar_sees_coral = lidar_debouncer.calculate(lidar_sees_coral_raw());
        SmartDashboard.putBoolean("end_effector_lidar", lidar_sees_coral_raw());
        SignalLogger.writeBoolean("end_effector/lidar", lidar_sees_coral_raw());
        SignalLogger.writeBoolean("end_effector/lidar_filtered", lidar_sees_coral());
        SignalLogger.writeBoolean("end_effector/cora_seated", coral_seated);
        if(roller_motor.getAppliedControl() == roller_req_velocity) {
            elevator_mech2d.sim.update_roller(target_feed, roller_velocity_signal.getValue());
        } else {
            elevator_mech2d.sim.update_roller(roller_req_position.getPositionMeasure(), roller_position_signal.getValue());
        }
    }

    public Command feed_coral_past_sensor() {
        return Commands.sequence(
            Commands.run(() -> {
                last_intake = gamepiece.CORAL;
                set_feed(constants.end_effector.intake_precise);
            }, this)
                .until(() -> !lidar_sees_coral_raw())
        );
    }

    public boolean has_ball() {
        return Math.abs(roller_velocity_signal.getValueAsDouble()) < 0.5 && roller_amps_signal.getValueAsDouble() > 20;
    }

    public boolean lidar_sees_coral_raw() {
        return !lidar.get();
    }

    public boolean lidar_sees_coral() {
        return lidar_sees_coral;
    }
    public boolean coral_seated() {
        return coral_seated;
    }

    public void set_feed(AngularVelocity speed) {
        target_feed = speed;
        if(speed.abs(DegreesPerSecond) < 1) {
            roller_motor.stopMotor();
        } else {
            roller_motor.setControl(roller_req_velocity.withVelocity(target_feed));
        }
    }

    public Command set(AngularVelocity roller_speed) {
        return Commands.runOnce(() -> {
            coral_seated = false;
            roller_motor.setPosition(0);
            set_feed(roller_speed);
        }, this);
    }

    public Command spit_last() {
        return Commands.runOnce(() -> {
            roller_motor.setPosition(0);
        }).alongWith(Commands.run(() -> {
            coral_seated = false;
            switch (last_intake) {
                case ALGAE:
                    if(bindings.height_for_the_elevator_algae_score == elevator_state.ALGAE_PROCESSOR) {
                        set_feed(spit_algae_proc);
                    } else {
                        set_feed(spit_algae);
                    }
                    break;
                case CORAL:
                    if(bindings.height_for_the_elevator_coral == elevator_state.L1) {
                        if(roller_position_signal.getValue().gt(Rotations.of(6))) {
                            if(l1_timer.get() < 0.3) {
                                set_feed(spit_coral_l1_fast);
                            } else if(l1_timer.get() < 0.4) {
                                set_feed(spit_coral_l1);
                            } else {
                                l1_timer.restart();
                            }
                        } else {
                            set_feed(spit_coral_l1);
                        }
                        return;
                    }
                    if(bindings.height_for_the_elevator_coral == elevator_state.L4) {
                        set_feed(spit_coral_l4);
                    } else {
                        set_feed(spit_coral);
                    }
                    break;
            }
        }, this));
    }

    public Command spit(gamepiece gp) {
        return Commands.runOnce(() -> {
            coral_seated = false;
            switch (gp) {
                case ALGAE:
                    set_feed(spit_algae);
                    break;
                case CORAL:
                    set_feed(spit_coral);
                    break;
            }
        }).andThen(Commands.idle(this));
    }

    public Command intake(gamepiece gp) {
        return Commands.run(() -> {
            coral_seated = false;
            last_intake = gp;
            switch (gp) {
                case ALGAE:
                    set_feed(constants.end_effector.intake_algae);
                    break;
                case CORAL:
                    if (bindings.manual_intake) {
                        set_feed(constants.end_effector.intake_manual);
                    } else {
                        set_feed(constants.end_effector.intake_fast);
                    }
                    break;
            }
        }, this);
    }

    public void nudge() {
        coral_seated = true;
        auto_flag = false;
        roller_motor.setControl(roller_req_position.withPosition(roller_position_signal.refresh().getValue().plus(constants.end_effector.roller_nudge)));
    }

    public Angle get_roller_position() {
        return roller_position_signal.getValue();
    }

    public Command cmd_nudge() {
        return Commands.sequence(
            Commands.runOnce(this::nudge, this),
            Commands.waitSeconds(0.05),
            Commands.waitUntil( () -> Math.abs(roller_error_signal.getValue()) < Units.degreesToRotations(15) )
        );
    }

    @Override
    public void configure() {
        lazy_ctre.lazy_config(roller_motor, config.end_effector.roller_config);
    }

    public enum gamepiece {
        CORAL,
        ALGAE
    }
}
