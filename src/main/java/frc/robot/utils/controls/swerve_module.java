package frc.robot.utils.controls;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.config.swerve.module_config;
import frc.robot.utils.can_savior;
import frc.robot.utils.configurable;
import frc.robot.utils.lazy_ctre;
import frc.robot.utils.talon_temps_safety;
import frc.robot.utils.controls.swerve_kin2.module_state;

public class swerve_module implements configurable {

    public swerve_module(module_config module_config, TimedRobot robot) {
        drive_output_configs = config.swerve.drive_configs(module_config.drive_inverted).MotorOutput;
        turn_output_configs = config.swerve.turn_configs(module_config.drive_inverted).MotorOutput;
        this.nt_name = module_config.name;
        this.module_config = module_config;
        double hard_coded_offset = config.is_comp ? module_config.comp_abs_offset : module_config.prac_abs_offset;
        abs_offset = Preferences.getDouble(nt_name+"_abs_rad", hard_coded_offset);
        couple_ratio = module.couple_ratio;

        drive = new dave_talon(module_config.can_drive);
        turn = new dave_talon(module_config.can_turn);
        can_savior.add_talons(drive, turn);
        abs = new DutyCycleEncoder(module_config.dio_abs);
        abs.setDutyCycleRange(1.0 / 4096, 4095.0 / 4096);

        drive_pos_signal = drive.getPosition();
        drive_vel_signal = drive.getVelocity();
        turn_pos_signal = turn.getPosition();
        turn_vel_signal = turn.getVelocity();

        turn.setSafetyEnabled(false);
        drive.setSafetyEnabled(false);

        BaseStatusSignal.setUpdateFrequencyForAll(constants.swerve.odom_freq,
            drive.getMotorVoltage()
        );

        BaseStatusSignal.setUpdateFrequencyForAll(4, 
            drive.getClosedLoopReference(),
            turn.getClosedLoopReference()
        );

        drive_temps = new talon_temps_safety(drive, "swerve/"+nt_name+"_drive_", Celsius.of(70), Celsius.of(85));
        turn_temps = new talon_temps_safety(turn, "swerve/"+nt_name+"_turn_", Celsius.of(70), Celsius.of(85));

        ParentDevice.optimizeBusUtilizationForAll(drive, turn);

        reset_turning();
        robot.addPeriodic(this::zero_abs_periodic, 0.02);
        robot.addPeriodic(this::periodic, RobotBase.isReal() ? 0.25 : 0.02, 0.125); // 4hz, 125ms offset
        if(RobotBase.isSimulation()) {
            turn.init_sim(robot, 200, DCMotor.getKrakenX60Foc(1), module.steer_ratio, 0.002);
            drive.init_sim(robot, 200, DCMotor.getKrakenX60Foc(1), module.drive_ratio, 0.1);
        }
    }

    private void periodic() {
        var base = "swerve/"+nt_name+"_";
        SmartDashboard.putNumber(base + "abs", Math.toDegrees(get_abs()));
        SmartDashboard.putBoolean(base+"abs?", has_abs());
        if(config.dev) {
            SmartDashboard.putNumber(base+"position_m", get_drive_position_m());
            SmartDashboard.putNumber(base+"coupling", get_drive_position().in(Rotations) / get_turning_position().in(Rotations));
            SmartDashboard.putNumber(base+"desired_mps", applied_state.speed_mps);
        }
        drive_temps.periodic();
        turn_temps.periodic();
    }

    public void reset_turning() {
        zeroed = false;
        abs_filter.reset();
        has_abs_debouncer.calculate(false);
    }

    private void zero_abs_periodic() {
        if(RobotBase.isSimulation()) {
            zeroed = true;
            return;
        }
        abs_filter.calculate(get_abs() / (Math.PI * 2.0));
        boolean has_debounced = has_abs_debouncer.calculate(abs.isConnected());
        if(!abs.isConnected()) {
            abs_filter.reset();
        }
        if(has_debounced && !zeroed) {
            turn.setPosition(abs_filter.lastValue());
            zeroed = true;
        }
    }

    public boolean toasty() {
        return turn_temps.toasty() || drive_temps.toasty();
    }

    public boolean burning() {
        return turn_temps.cutoff() || drive_temps.cutoff();
    }

    public void save_abs_offset(double theta_raw) {
        abs_offset = theta_raw;
        Preferences.setDouble(nt_name+"_abs_rad", theta_raw);
    }

    public double get_abs_raw() {
        return Units.rotationsToRadians(abs.get());
    }

    public double get_abs() {
        return get_abs_raw() - abs_offset;
    }

    public boolean has_abs() {
        return abs.isConnected() && zeroed;
    }

    public Measure<AngleUnit> get_turning_position() {
        return BaseStatusSignal.getLatencyCompensatedValue(turn_pos_signal, turn_vel_signal);
    }

    public AngularVelocity get_turning_velocity() {
        return turn_vel_signal.getValue();
    }

    public Measure<AngleUnit> get_drive_position() {
        return BaseStatusSignal.getLatencyCompensatedValue(drive_pos_signal, drive_vel_signal).plus((get_turning_position().times(couple_ratio)));
    }

    public double get_drive_position_m() {
        return get_drive_position().in(Radians) * constants.swerve.wheel_radius;
    }

    public AngularVelocity get_drive_velocity() {
        return drive_vel_signal.getValue().minus(get_turning_velocity().times(couple_ratio));
    }

    public double get_drive_velocity_mps() {
        return get_drive_velocity().in(RadiansPerSecond) * constants.swerve.wheel_radius;
    }

    public void get_odom(SwerveModulePosition position) {
        position.angle = Rotation2d.fromRotations(get_turning_position().in(Rotations));
        position.distanceMeters = get_drive_position_m();
    }

    public void get_state(module_state current_state) {
        current_state.speed_mps = get_drive_velocity_mps();
        current_state.theta_rad = get_turning_position().in(Radians);
        current_state.omega_radps = get_turning_velocity().in(RadiansPerSecond);
    }

    private void set_speed(double speed_mps) {
        if(last_vel == speed_mps) {
            return;
        }
        last_vel = speed_mps;
        double speed_rotations_ps = Units.radiansToRotations(speed_mps / constants.swerve.wheel_radius);
        drive.setControl(drive_control_out.withVelocity(speed_rotations_ps));
    }

    private void set_turn(double theta_rad, double omega_radps) {
        if(last_ang == theta_rad) {
            return;
        }
        last_ang = theta_rad;
        turn.setControl(turn_control_out.withPosition(theta_rad / (Math.PI * 2.0)).withVelocity(omega_radps / (Math.PI * 2.0)));
    }

    public void stop() {
        drive.stopMotor();
        turn.stopMotor();
    }

    public void set_state(module_state desired_state) {
        desired_state.optimize(Rotation2d.fromRotations(get_turning_position().in(Rotations)));

        set_turn(desired_state.theta_rad, desired_state.omega_radps);

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double azimuth_err = desired_state.theta_rad - get_turning_position().in(Radians);
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        double cosine_scalar = Math.cos(azimuth_err);
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        if(cosine_scalar < 0.0) {
            cosine_scalar = 0.0;
        }

        desired_state.speed_mps *= cosine_scalar;

        /* Back out the expected shimmy the drive motor will see */
        /* Find the angular rate to determine what to back out */
        double azimuth_turn_rotations_ps = get_turning_velocity().in(RotationsPerSecond);
        /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        double drive_rate_back_out = Units.rotationsToRadians(azimuth_turn_rotations_ps * couple_ratio) * constants.swerve.wheel_radius;

        set_speed(desired_state.speed_mps - drive_rate_back_out);

        applied_state = desired_state;
    }

    public void coast() {
        drive.getConfigurator().apply(drive_output_configs.withNeutralMode(NeutralModeValue.Coast));
        turn.getConfigurator().apply(turn_output_configs.withNeutralMode(NeutralModeValue.Coast));
    }

    public void brake() {
        drive.getConfigurator().apply(drive_output_configs.withNeutralMode(NeutralModeValue.Brake));
        turn.getConfigurator().apply(turn_output_configs.withNeutralMode(NeutralModeValue.Brake));
    }

    public final String nt_name;
    public final talon_temps_safety drive_temps, turn_temps;
    private final dave_talon drive, turn;
    private final DutyCycleEncoder abs;
    public final StatusSignal<Angle> drive_pos_signal, turn_pos_signal;
    public final StatusSignal<AngularVelocity> drive_vel_signal, turn_vel_signal;
    private final PositionVoltage turn_control_out = new PositionVoltage(0).withEnableFOC(config.swerve.ctre_pro).withSlot(0);
    private final VelocityVoltage drive_control_out = new VelocityVoltage(0).withEnableFOC(config.swerve.ctre_pro).withSlot(0);
    private final module_config module_config;
    private final double couple_ratio;
    private final Debouncer has_abs_debouncer = new Debouncer(1.5, DebounceType.kRising);
    private final LinearFilter abs_filter = LinearFilter.movingAverage(6);

    private boolean zeroed = false;
    private double abs_offset;
    private double last_vel, last_ang;
    private module_state applied_state = new module_state(0, 0, 0);

    private final MotorOutputConfigs drive_output_configs, turn_output_configs;

    @Override
    public void configure() {
        drive.clearStickyFaults();
        lazy_ctre.lazy_config(drive, config.swerve.drive_configs(module_config.drive_inverted));
        turn.clearStickyFaults();
        lazy_ctre.lazy_config(turn, config.swerve.turn_configs(module_config.turn_inverted));
    }

    public void apply_drive_current_limits(CurrentLimitsConfigs limits) {
        drive.getConfigurator().apply(limits);
    }
}
