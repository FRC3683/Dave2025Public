package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.constants.elevator_state;
import frc.robot.sim.elevator_mech2d;
import frc.robot.utils.configurable;
import frc.robot.utils.lazy_ctre;
import frc.robot.utils.controls.dave_talon;

public class ramp extends SubsystemBase implements configurable {
    private final VoltageOut hold_voltage = new VoltageOut(-0.07);
    private final dave_talon motor = new dave_talon(config.can_ramp); 
    private final DutyCycle lidar_edge = new DutyCycle(new DigitalInput(config.dio_ramp_lidar_edge));
    private final DutyCycle lidar_middle = new DutyCycle(new DigitalInput(config.dio_ramp_lidar_middle));
    private boolean filtered_lidar_edge = false, filtered_lidar_middle = false;
    private final Debouncer lidar_edge_debounce = new Debouncer(0.04, DebounceType.kBoth);
    private final Debouncer lidar_middle_debounce = new Debouncer(0.02, DebounceType.kBoth);
    private AngularVelocity target_feed = DegreesPerSecond.zero();
    private final Supplier<Distance> elevator_height;

    private final StatusSignal<AngularVelocity> velocity_signal = motor.getVelocity();
    private final VelocityVoltage output_req = new VelocityVoltage(0);

    public ramp(robot robot, Supplier<Distance> elevator_height) {
        this.elevator_height = elevator_height;
        if(RobotBase.isSimulation()) {
            motor.init_sim(robot, 250, DCMotor.getKrakenX60(1), 1, 0.0001);
        }
        motor.stopMotor();

        setDefaultCommand(smart_hold());
    }

    public Command smart_hold() {
        return Commands.run(() -> {
            if (filtered_lidar_edge && filtered_lidar_middle) {
                stop();
            } else if (filtered_lidar_edge && !filtered_lidar_middle) {
                run(constants.ramp.pre_intake);
            } else if (!filtered_lidar_edge && filtered_lidar_middle) {
                run(constants.ramp.idle.unaryMinus());
                // run(constants.ramp.idle);
            } else {
                run(constants.ramp.idle);
            }
        }, this);
    }

    private Angle last_motor_pos;
    @Override
    public void periodic() {
        velocity_signal.refresh();
        filtered_lidar_edge = lidar_edge_debounce.calculate(get_raw_lidar_edge());
        filtered_lidar_middle = lidar_middle_debounce.calculate(get_raw_lidar_middle());
        // SmartDashboard.putNumber("middle nano", lidar_middle.);
        if (!filtered_lidar_edge && !filtered_lidar_middle) {
            last_motor_pos = motor.getRotorPosition().getValue();
        }
        SignalLogger.writeBoolean("ramp/raw_lidar_edge", get_raw_lidar_edge());
        SignalLogger.writeBoolean("ramp/filtered_lidar_edge", filtered_lidar_edge);

        SmartDashboard.putNumber("lidar edge high time", lidar_edge.getHighTimeNanoseconds());
        SmartDashboard.putNumber("lidar middle high time", lidar_middle.getHighTimeNanoseconds());
        SmartDashboard.putBoolean("filtered lidar_edge", filtered_lidar_edge);
        SmartDashboard.putBoolean("filtered lidar_middle", filtered_lidar_middle);
        
        SmartDashboard.putBoolean("raw lidar_edge", get_raw_lidar_edge());
        SmartDashboard.putBoolean("both lidar_edges see", filtered_lidar_edge && filtered_lidar_middle);
        elevator_mech2d.sim.update_ramp(target_feed, velocity_signal.getValue());
    }

    private boolean get_raw_lidar_edge() {
        var high_time = lidar_edge.getHighTimeNanoseconds();
        return 1000000 < high_time && high_time < 1400000;
    }

    public boolean get_raw_lidar_middle() {
        var high_time = lidar_middle.getHighTimeNanoseconds();
        return 1000000 < high_time && high_time < 1350000;
    }

    public boolean has_coral() {
        return filtered_lidar_edge || filtered_lidar_middle;
    }

    public boolean has_coral_edge() {
        return filtered_lidar_edge;
    }

    public boolean has_coral_middle() {
        return filtered_lidar_middle;
    }

    public void run(AngularVelocity speed) {
        target_feed = speed;
        motor.setControl(output_req.withVelocity(target_feed));
   }

    public Command cmd_run(AngularVelocity speed) {
        return Commands.runOnce(() -> {
            run(speed);
        }, this);
    }

    public void stop() {
        target_feed = DegreesPerSecond.zero();
        motor.stopMotor();
    }

    public Command cmd_stop() {
        return Commands.runOnce(this::stop, this).andThen(Commands.idle(this));
    }

    public double get_current_amps() {
        return motor.getStatorCurrent().getValue().in(Amps);
    }

    public double get_velocity_rps() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public void configure() {
        lazy_ctre.lazy_config(motor, config.ramp.config);
    }

}
