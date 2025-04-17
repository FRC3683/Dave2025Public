package frc.robot.utils.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.spns.SpnValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.can;
import frc.robot.utils.math_utils;

public class dave_talon extends TalonFX {
    private final PIDController[] sim_pid = {
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0),
    };
    private final SimpleMotorFeedforward[] sim_ff = {
        new SimpleMotorFeedforward(0, 0),
        new SimpleMotorFeedforward(0, 0),
        new SimpleMotorFeedforward(0, 0),
    };
    private TrapezoidProfile.State profile_state = new TrapezoidProfile.State();
    private final TrapezoidProfile.State goal_profile_state = new TrapezoidProfile.State();
    private TrapezoidProfile profile = new TrapezoidProfile(new Constraints(10, 10));
    private DCMotorSim sim;
    private double control_dts;

    private static final String[] library = { "whiplash", "megalovania", "zelda", "under_the_sea" };
    private static int selection = 0;
    public static String get_selected_song() {
        return library[selection];
    }
    private static Orchestra orchestra = new Orchestra(get_selected_song());
    public static Command play_selected() {
        SmartDashboard.putString("ipod_touch", "in queue: " + get_selected_song());
        return Commands.runOnce(() -> {
            orchestra.loadMusic(get_selected_song() + ".chrp");
            orchestra.play();
            SmartDashboard.putString("ipod_touch", "now playing: " + get_selected_song());
        }).ignoringDisable(true);
    }

    public static Command reset_music() {
        return Commands.runOnce(() -> {
            orchestra.stop();
            ++selection;
            if(selection >= library.length) {
                selection = 0;
            }
            SmartDashboard.putString("ipod_touch", "in queue: " + get_selected_song());
        }).ignoringDisable(true);
    }

    public dave_talon(can can) {
        super(can.can_id, can.canbus);
        orchestra.addInstrument(this, 0);
    }

    @Override
    public TalonFXConfigurator getConfigurator() {
        if(RobotBase.isReal()) {
            return super.getConfigurator();
        }
        return new SimConfigurator();
    }

    public dave_talon init_sim(TimedRobot robot, int control_hz, DCMotor motor, double gear_ratio, double moment_of_inertia) {
        var lin_sys = LinearSystemId.createDCMotorSystem(motor, moment_of_inertia, gear_ratio);
        sim = new DCMotorSim(lin_sys, motor);
        control_dts = 1.0 / (double)control_hz;
        sim_pid[0] = new PIDController(0, 0, 0, control_dts);
        sim_pid[1] = new PIDController(0, 0, 0, control_dts);
        sim_pid[2] = new PIDController(0, 0, 0, control_dts);
        robot.addPeriodic(this::sim_periodic, control_dts);
        return this;
    }

    private void sim_periodic() {
        var req = getAppliedControl();
        if(req.getClass().equals(CoastOut.class)) {
            sim.setInputVoltage(0);
        }
        else if(req.getClass().equals(PositionVoltage.class)) {
            var control = (PositionVoltage)req;
            double output = pid_position(control.Slot, control.Position, control.Velocity);
            sim.setInputVoltage(output + control.FeedForward);
        }
        else if(req.getClass().equals(VelocityVoltage.class)) {
            var control = (VelocityVoltage)req;
            double output = pid_velocity(control.Slot, control.Velocity, control.Acceleration);
            sim.setInputVoltage(output + control.FeedForward);
        } else if(req.getClass().equals(MotionMagicVelocityVoltage.class)) {
            // TODO
            var control = (MotionMagicVelocityVoltage)req;
            double output = pid_velocity(control.Slot, control.Velocity, control.Acceleration);
            sim.setInputVoltage(output + control.FeedForward);
        }
        else if(req.getClass().equals(MotionMagicVoltage.class)) {
            var control = (MotionMagicVoltage)req;
            var velocity_setpoint = profile_state.velocity;
            profile_state = profile.calculate(control_dts, profile_state, goal_profile_state);
            var output = pid_position(control.Slot, profile_state.position, velocity_setpoint, profile_state.velocity);
            sim.setInputVoltage(output + control.FeedForward);
        }
        else {
            sim.setInputVoltage(0);
        }

        if(Math.abs(sim.getAngularVelocityRPM()) < 10) { // wierd things happen
            sim.setState(sim.getAngularPositionRad(), 0);
        }

        sim.update(control_dts);
    }

    @Override
    public StatusSignal<Angle> getPosition() {
        if(RobotBase.isReal()) {
            return super.getPosition();
        }
        return new StatusSignal<Angle>(deviceIdentifier, SpnValue.PRO_PosAndVel_Position.value, () -> {}, Angle.class, 
            val -> get_sim_position(),
            "Position");
    }

    @Override
    public StatusSignal<AngularVelocity> getVelocity() {
        if(RobotBase.isReal()) {
            return super.getVelocity();
        }
        return new StatusSignal<AngularVelocity>(deviceIdentifier, SpnValue.PRO_PosAndVel_Position.value, () -> {}, AngularVelocity.class, 
            val -> get_sim_velocity(),
            "Velocity")
            // TODO?
            //  {
            //     @Override
            //     public double getValueAsDouble() {
            //         return 0;
            //     }
            // }
            ;
    }

    public Angle get_sim_position() {
        if(sim == null) {
            return Degrees.zero();
        }
        return sim.getAngularPosition();
    }

    public AngularVelocity get_sim_velocity() {
        if(sim == null) {
            return DegreesPerSecond.zero();
        }
        return sim.getAngularVelocity();
    }

    private double pid_position(int slot, double position, double velocity, double next_velocity) {
        double feedforward = sim_ff[slot].calculateWithVelocities(velocity, next_velocity);
        double feedback = sim_pid[slot].calculate(sim.getAngularPositionRotations(), position);
        return feedforward + feedback;
    }

    private double pid_position(int slot, double position, double velocity) {
        double feedforward = sim_ff[slot].calculate(velocity);
        double feedback = sim_pid[slot].calculate(sim.getAngularPositionRotations(), position);
        return feedforward + feedback;
    }

    private double pid_velocity(int slot, double velocity, double acceleration) {
        double feedforward = sim_ff[slot].calculateWithVelocities(velocity, velocity + (acceleration * control_dts));
        double feedback = sim_pid[slot].calculate(Units.radiansToRotations(sim.getAngularVelocityRadPerSec()), velocity);
        double output = feedforward + feedback;
        if(Math.abs(output) < 0.1) {
            output = 0;
        }
        return output;
    }

    public void set_position(double rotations) {
        sim.setState(Units.rotationsToRadians(rotations), 0);
    }

    @Override
    public StatusCode setControl(MotionMagicVoltage req) {
        if(!math_utils.close_enough(goal_profile_state.position, req.Position, Units.degreesToRotations(0.5))) {
            profile_state.position = getPosition().getValue().in(Rotations);
            profile_state.velocity = getVelocity().getValue().in(RotationsPerSecond);
            goal_profile_state.position = req.Position;
        }
        return super.setControl(req);
    }

    private class SimConfigurator extends TalonFXConfigurator {
        private SimConfigurator() {
            super(deviceIdentifier);
        }

        @Override
        public StatusCode apply(TalonFXConfiguration config) {
            sim_pid[0].setPID(config.Slot0.kP, config.Slot0.kI, config.Slot0.kD);
            sim_ff [0] = new SimpleMotorFeedforward(config.Slot0.kS, config.Slot0.kV, config.Slot0.kA);
            sim_pid[1].setPID(config.Slot1.kP, config.Slot1.kI, config.Slot1.kD);
            sim_ff [1] = new SimpleMotorFeedforward(config.Slot1.kS, config.Slot1.kV, config.Slot1.kA);
            sim_pid[2].setPID(config.Slot2.kP, config.Slot2.kI, config.Slot2.kD);
            sim_ff [2] = new SimpleMotorFeedforward(config.Slot2.kS, config.Slot2.kV, config.Slot2.kA);
            profile = new TrapezoidProfile(new Constraints(config.MotionMagic.MotionMagicCruiseVelocity, config.MotionMagic.MotionMagicAcceleration));
            if(config.ClosedLoopGeneral.ContinuousWrap) {
                sim_pid[0].enableContinuousInput(-0.5, 0.5);
                sim_pid[1].enableContinuousInput(-0.5, 0.5);
                sim_pid[2].enableContinuousInput(-0.5, 0.5);
            } else {
                sim_pid[0].disableContinuousInput();
                sim_pid[1].disableContinuousInput();
                sim_pid[2].disableContinuousInput();
            }
            return StatusCode.OK;
        }
    }
}
