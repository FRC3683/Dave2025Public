package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config;
import frc.robot.sim.climb_mech2d;
import frc.robot.utils.configurable;
import frc.robot.utils.lazy_ctre;
import frc.robot.utils.controls.dave_talon;

public class climb extends SubsystemBase implements configurable {
    private final Servo left = new Servo(config.pwm_servo_left);
    private final Servo right = new Servo(config.pwm_servo_right);
    private final dave_talon motor = config.is_comp ? new dave_talon(config.can_climb) : null;
    private final climb_mech2d sim = new climb_mech2d();
    private final Debouncer disabled_debouncer = new Debouncer(3);

    private final double preclimb_rotations = 113.7;
    private final double climb_rotations = 182;
    private final MotionMagicVoltage preclimb_request = new MotionMagicVoltage(preclimb_rotations).withSlot(0);
    private final MotionMagicVoltage climb_request = new MotionMagicVoltage(climb_rotations).withSlot(1);

    private final StatusSignal<Angle> position_signal = config.is_comp ? motor.getPosition() : null;
    private final StatusSignal<Current> stator_signal = config.is_comp ? motor.getStatorCurrent() : null;

    // private boolean prepped = false;

    public climb(TimedRobot robot) {
        if(!config.is_comp) {
            setDefaultCommand(run(() -> {}));
            return;
        }
        if(RobotBase.isSimulation()) {
            // motor.init_sim(robot, 250, DCMotor.getKrakenX60(1), 10, 0.01);
        }
        motor.setPosition(0);
        motor.stopMotor();
        left.set(0.5);
        right.set(0.5);

        setDefaultCommand(Commands.run(() -> {
            // prepped = false;
            motor.stopMotor();
            left.set(0.5);
            right.set(0.5);
        }, this));

    }

    public boolean prepped() {
        if(!config.is_comp) {
            return false;
        }
        return position_signal.getValueAsDouble() > preclimb_rotations - 15;
    }

    public void reset() {
        if(!config.is_comp) {
            return;
        }
        motor.setPosition(0);
    }

    @Override
    public void periodic() {
        if(!config.is_comp) {
            return;
        }
        BaseStatusSignal.refreshAll(position_signal, stator_signal);
        SmartDashboard.putNumber("climb_rotations", position_signal.getValueAsDouble());

        // SmartDashboard.putNumber("climb pos raw", position_signal.getValueAsDouble());
        sim.update_servo(Degrees.of(left.getAngle()));
        // sim.update_climb(climb_target, position_signal.getValue());
    }

    public Command tap() {
        if(!config.is_comp) {
            return Commands.idle(this).withTimeout(0.2);
        }
        return Commands.run(() -> {
            motor.set(0.6);
        }, this).withTimeout(0.35);
    }

    public Command preclimb() {
        if(!config.is_comp) {
            return Commands.idle(this);
        }
        return Commands.sequence(
            open_servo(),
            Commands.waitSeconds(0.8),
            Commands.run(() -> {
                // motor.setControl(preclimb_request);
                var pos = position_signal.getValueAsDouble();
                if(pos < preclimb_rotations - 15) {
                    motor.set(1.0);
                } else if(pos < preclimb_rotations) {
                    motor.set(0.4);
                } else {
                    motor.stopMotor();
                }
            }, this)
        );
    }

    public Command open_servo() {
        return Commands.runOnce(() -> {
            left.set(1.0);
            right.set(0.0);
        }, this);
    }

    public Command uppies() {
        if(!config.is_comp) {
            return Commands.idle(this);
        }
        return Commands.run(() -> {
            // if(position_signal.getValueAsDouble() < climb_rotations) {
            //     motor.set(0.7);
            // } else {
            //     motor.stopMotor();
            // }
            motor.setControl(climb_request);
        }, this);
    }

    @Override
    public void configure() {
        if(!config.is_comp) {
            return;
        }
        lazy_ctre.lazy_config(motor, config.climb.config);
    }
}
