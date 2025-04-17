package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ResourceBundle.Control;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.bindings;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.sim.elevator_mech2d;
import frc.robot.utils.configurable;
import frc.robot.utils.lazy_ctre;
import frc.robot.utils.math_utils;
import frc.robot.utils.talon_temps_safety;
import frc.robot.utils.controls.dave_talon;
import frc.robot.constants.elevator_state;

import static frc.robot.constants.elevator.*;

public class elevator extends SubsystemBase implements configurable {
    private final dave_talon motor_left = new dave_talon(config.can_elevator_left);
    private final dave_talon motor_right = new dave_talon(config.can_elevator_right);
    private final dave_talon pivot_motor = new dave_talon(config.can_endeffector_pivot);
    private final DigitalInput homing_hallfx = new DigitalInput(config.dio_elevator_hallfx);

    private final StatusSignal<Angle> pivot_position_signal = pivot_motor.getPosition();
    private final StatusSignal<AngularVelocity> pivot_speed_signal = pivot_motor.getVelocity();
    private final StatusSignal<Angle> height_position_signal = motor_left.getPosition();
    private final StatusSignal<AngularVelocity> height_velocity_signal = motor_left.getVelocity();
    private final Supplier<Boolean> end_effector_lidar;

    private final talon_temps_safety left_safety, right_safety;

    private boolean homing = false;

    private boolean auto_speeds = true;
    private final StaticBrake brake = new StaticBrake();
    private final VoltageOut pivot_home_req = new VoltageOut(0.0);
    private final MotionMagicVoltage pivot_request_mm = new MotionMagicVoltage(0);
    private MotionMagicVoltage position_request = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private DynamicMotionMagicVoltage position_request_comp = new DynamicMotionMagicVoltage(0, 0, 0 ,0).withSlot(0).withEnableFOC(true);
    private VelocityVoltage velocity_request = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
    private VelocityVoltage pivot_velocity_request = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
    private elevator_state current_setpoint = elevator_state.IDLE;
    private elevator_state target_setpoint = elevator_state.IDLE; // to avoid guillotining one's self
    private Distance manual_setpoint = Inches.of(0);
    private Angle manual_pivot = constants.elevator.pivot_stowed;
    private LinearVelocity manual_elevator_velocity = InchesPerSecond.zero();
    private AngularVelocity manual_pivot_velocity = DegreesPerSecond.zero();

    public elevator(TimedRobot robot, Supplier<Boolean> end_effector_lidar) {
        this.end_effector_lidar = end_effector_lidar;

        if(RobotBase.isSimulation()) {
            motor_left.init_sim(robot, 200, DCMotor.getKrakenX60(2), 1.0, 0.0008);
            pivot_motor.init_sim(robot, 250, DCMotor.getKrakenX60(1), 1, 0.0002);
        }
        stop();

        home_pivot();
        zero();

        left_safety = new talon_temps_safety(motor_left, "elevator/left_", Celsius.of(45), Celsius.of(70));
        right_safety = new talon_temps_safety(motor_left, "elevator/right_", Celsius.of(45), Celsius.of(70));

        setDefaultCommand(Commands.either(
            homing_cmd().andThen(cmd_set_target_state(elevator_state.STOW)).andThen(Commands.idle(this)),
            Commands.idle(this),
            DriverStation::isTeleop
        ));
        // setDefaultCommand(cmd_set_target_state(elevator_state.RAMP).andThen(Commands.idle(this)));
    }

    public void zero() {
        motor_left.setPosition(0);
        motor_right.setPosition(0);
        homing = false;
        manual_setpoint = Meters.of(0);
    }

    public void home_pivot() {
        pivot_motor.setPosition(pivot_home);
        manual_pivot = pivot_home;
    }

    public boolean height_at_setpoint(elevator_state state) {
        return math_utils.close_enough(get_height(), state.height, Inches.of(0.25));
    }

    public boolean within_target(Distance tolerance) {
        return math_utils.close_enough(get_height(), current_setpoint.height, tolerance);
    }

    public boolean within(elevator_state state, Distance distance) {
        return math_utils.close_enough(get_height(), state.height, distance);
    }

    public boolean at_home() {
        return !homing_hallfx.get();
    }

    public void apply_current_limits(boolean auto) {
        auto_speeds = auto;
        if(auto) {
            apply_current_limits(config.elevator.auto_limits, config.elevator.auto_mm);
        } else {
            apply_current_limits(config.elevator.tele_limits, config.elevator.tele_mm);
        }
    }

    private void apply_current_limits(CurrentLimitsConfigs limits, MotionMagicConfigs mm) {
        motor_left.getConfigurator().apply(limits);
        motor_right.getConfigurator().apply(limits);
        motor_left.getConfigurator().apply(mm);
        motor_right.getConfigurator().apply(mm);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll( height_position_signal, height_velocity_signal );
        BaseStatusSignal.refreshAll( pivot_position_signal, pivot_speed_signal );
        SmartDashboard.putNumber("end effector pivot deg", pivot_position_signal.getValue().in(Degrees));

        SmartDashboard.putString("elevator prescore", bindings.height_for_the_elevator_coral.name());
        SmartDashboard.putNumber("elevator height m", get_height().in(Meters));
        elevator_mech2d.sim.set_elevator_pos(current_setpoint.height, get_height());
        elevator_mech2d.sim.set_wrist_angle(current_setpoint.pivot, pivot_position_signal.getValue());

        SmartDashboard.putBoolean("hallfx", homing_hallfx.get());

        left_safety.periodic();
        right_safety.periodic();

        if(left_safety.cutoff() || right_safety.cutoff()) {
            stop();
            return;
        }

        if(homing) {
            return;
        }

        if(!end_effector_lidar.get() || target_setpoint == elevator_state.MANUAL || target_setpoint == elevator_state.TRAP){
            current_setpoint = target_setpoint;
        }
        if(current_setpoint == elevator_state.VELOCITY) {
            pivot_motor.setControl(pivot_request_mm.withPosition(current_setpoint.pivot));
            return;
        } else if(current_setpoint == elevator_state.IDLE) {
            stop();
        } else {
            Distance elevator_setpoint = current_setpoint.height;
            Angle pivot_setpoint = pivot_stowed;
            if(get_height().isNear(elevator_setpoint, current_setpoint.tol) ) {
                pivot_setpoint = current_setpoint.pivot;
            }
            if(current_setpoint == elevator_state.MANUAL) {
                pivot_setpoint = manual_pivot;
                elevator_setpoint = manual_setpoint;
            }
            ControlRequest elevator_req = position_request.withPosition(elevator_setpoint.in(Meters));
            if(config.is_comp) {
                position_request_comp.Acceleration = auto_speeds ? config.elevator.auto_mm.MotionMagicAcceleration : config.elevator.tele_mm.MotionMagicAcceleration;
                position_request_comp.Velocity = auto_speeds ? config.elevator.auto_mm.MotionMagicCruiseVelocity : config.elevator.tele_mm.MotionMagicCruiseVelocity;
                // if(math_utils.close_enough(get_height(), elevator_setpoint, Inches.of(24))) {
                //     // position_request_comp.Acceleration = config.elevator.tele_mm.MotionMagicAcceleration * 0.3; // less aggressive deceleration
                // }
                if(get_height().minus(elevator_setpoint).gt(Inches.of(12))) { // moving down go slower pls
                    position_request_comp.Velocity *= 0.7;
                }
                elevator_req = position_request_comp.withPosition(elevator_setpoint.in(Meters));
            }
            ControlRequest pivot_req = pivot_request_mm.withPosition(pivot_setpoint.in(Rotations));

            if(current_setpoint == elevator_state.MANUAL) {
                elevator_req = velocity_request.withVelocity(manual_elevator_velocity.in(MetersPerSecond));
                pivot_req = pivot_velocity_request.withVelocity(manual_pivot_velocity.in(RotationsPerSecond));
                if(manual_elevator_velocity.isNear(MetersPerSecond.zero(), MetersPerSecond.of(0.001))) {
                    elevator_req = brake;
                }
                if(manual_pivot_velocity.isNear(DegreesPerSecond.zero(), DegreesPerSecond.of(0.001))) {
                    pivot_req = brake;
                }
            }

            if(current_setpoint == elevator_state.TRAP) {
                if(get_height().lt(Inches.of(10))) {
                    elevator_req = velocity_request.withVelocity(InchesPerSecond.of(6).in(MetersPerSecond));
                } else {
                    elevator_req = position_request.withPosition(Inches.of(10.5).in(Meters));
                }
            }

            pivot_motor.setControl(pivot_req);
            motor_left.setControl(elevator_req);
            motor_right.setControl(elevator_req);
        }
    }

    public Command manual(Supplier<Integer> height_speed, Supplier<Integer> pivot_speed) {
        return Commands.startRun(() -> {
            target_setpoint = elevator_state.MANUAL;
            current_setpoint = elevator_state.MANUAL;
            manual_setpoint = get_height();
            manual_pivot = get_pivot();
        }, () -> {
            var h = Math.signum(height_speed.get());
            var p = Math.signum(pivot_speed.get());
            SmartDashboard.putNumber("p", p);
            manual_elevator_velocity = MetersPerSecond.of(0.4).times(h);
            manual_pivot_velocity = DegreesPerSecond.of(20).times(p);
            if(h != 0) {
                manual_setpoint = get_height().plus(Inches.of(0.5).times(h));
            }
            if(p != 0) {
                manual_pivot = get_pivot().plus(Degrees.of(1).times(p));
            }
        }, this).finallyDo(() -> {
            target_setpoint = elevator_state.STOW;
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public elevator_state get_target() {
        return target_setpoint;
    }

    boolean should_home() {
        switch(current_setpoint) {
            case ALGAE1:
            case ALGAE2:
            case ALGAE_GROUND:
            case ALGAE_PROCESSOR:
            case IDLE:
            case PRE_HOMING:
            case RAMP:
            case MANUAL:
            case TRAP:
                return false;
            case ALGAE_BARGE:
            case L1:
            case L2:
            case L3:
            case L4:
            default:
                return true;
        }
    }

    public Command homing_cmd() {
        var home_sequence = Commands.sequence(
            Commands.either(
                cmd_set_target_state(elevator_state.PRE_HOMING)
                    .andThen( Commands.waitUntil( 
                        new Trigger(() -> get_height().lt(elevator_state.PRE_HOMING.height.plus(Inches.of(3))))
                        // .or( new Trigger( () -> Math.abs(height_velocity_signal.getValueAsDouble()) < 0.01 ).debounce(2.0) )
                    ) ),
                Commands.none(),
                () -> get_height().gt(elevator_state.PRE_HOMING.height.plus(Inches.of(1)))
            ),
            Commands.runOnce(() -> {
                homming();
            }, this),
            Commands.waitUntil(this::at_home)
                .withTimeout(5),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> {
                zero();
            })
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        return Commands.either(home_sequence, Commands.none(), this::should_home).finallyDo(() -> {
            homing = false;
        });
    }

    public void homming() { 
        homing = true;
        var current_amps = -0.8;
        var max_dutycycle = 0.1;
        // var control = new VoltageOut(-1.2);
        var control = new TorqueCurrentFOC(current_amps).withMaxAbsDutyCycle(max_dutycycle);
        motor_left.setControl(control);
        motor_right.setControl(control);
    }

    public  Distance get_height() {
        return Meters.of(height_position_signal.getValueAsDouble());
    }

    public void set_target_state(elevator_state state) {
        target_setpoint = state;
    }

    public Command cmd_set_target_state(elevator_state state) {
        return Commands.runOnce(() -> {
            set_target_state(state);
        }, this);
    }

    public Angle get_pivot() {
        return pivot_position_signal.getValue();
    }

    public AngularVelocity get_pivot_speed() {
        return pivot_speed_signal.getValue();
    }

    public Command hold_target_state(elevator_state state) {
        return cmd_set_target_state(state).andThen(Commands.idle(this));
    }

    public Command velocity(double speed) {
        return Commands.runOnce(() -> {
            this.target_setpoint = elevator_state.VELOCITY;
            motor_left.setControl(velocity_request.withVelocity(speed));
            motor_right.setControl(velocity_request.withVelocity(speed));
        }).andThen(Commands.idle(this));
    }

    public void stop() {
        motor_left.stopMotor();
        motor_right.stopMotor();
        pivot_motor.stopMotor();
    }

    @Override
    public void configure() {
        lazy_ctre.lazy_config(motor_left, config.elevator.get_config(InvertedValue.Clockwise_Positive));
        lazy_ctre.lazy_config(motor_right, config.elevator.get_config(InvertedValue.CounterClockwise_Positive));
        lazy_ctre.lazy_config(pivot_motor, config.elevator.pivot_config);
    }
}


