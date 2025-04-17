package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.elevator.pivot_home;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

public class constants {
    public static final int control_freq = 50;
    public static final double control_dts = 1.0 / control_freq;

    public static final double voltage_warning_threshold_comp = 12.6;
    public static final double voltage_warning_threshold_prac = 12.1;

    public static AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static final int[] red_reef_tags = { 6, 7, 8, 9, 10, 11 };
    public static final int[] blue_reef_tags = { 17, 18, 19, 20, 21, 22 };

    private static final double past_home = 120;
    private static final double ramp_m_prac = 0.069 - Units.inchesToMeters(0.25 + 0.25 + 1.0 + 0.5 + 0.25);
    private static final double ramp_m_comp = 0.025;
    private static final double L1_comp = 0.48 + Units.inchesToMeters(0.25 - 1.25 + 0.5 + 0.5);
    private static final double L1_prac = 0.48 + Units.inchesToMeters(0.25);
    private static final double L3_prac = 1.08;
    private static final double L3_comp = 1.08 + Units.inchesToMeters(1.5);
    private static final double barge_fudge_in = config.is_comp ? 3.0 : 0;

    public enum elevator_state {
        IDLE(0, elevator.pivot_stowed),
        L1(config.is_comp ? L1_comp : L1_prac, elevator.pivot_home.in(Degrees) - 140),
        L1_andahalf(0.48 + Units.inchesToMeters(0.25), elevator.pivot_home),
        L2(0.7, past_home),
        L3(config.is_comp ? L3_comp : L3_prac, past_home),
        L4(1.725 + Units.inchesToMeters(1.0 + 0.5 + 0.5), past_home),
        L4_andabit(1.725 + Units.inchesToMeters(1.0 + 0.5 + 0.5), past_home),
        L4_SUB(1.725 + Units.inchesToMeters(1.0 - (3.0/8.0) - 0.5), past_home),
        RAMP(config.is_comp ? ramp_m_comp : ramp_m_prac, elevator.pivot_stowed.minus(Degrees.of(27))),
        STOW(RAMP.height.in(Meters), elevator.pivot_stowed),
        AUTO_PRESCORE(0.4, past_home),
        TRAP(0, pivot_home.minus(Degrees.of(35))),
        CLIMB(RAMP.height.in(Meters), pivot_home.minus(Degrees.of(7))),

        ALGAE_GROUND(0.01, elevator.pivot_home.in(Degrees) - 81),
        ALGAE_LOLLIPOP(0.01, elevator.pivot_home.in(Degrees) - 34.69 - 2.0), // TODO
        ALGAE1(0.37, elevator.pivot_home.in(Degrees) - 35),
        ALGAE2(0.82 - Units.inchesToMeters(1), elevator.pivot_home.in(Degrees) - 40),
        L2_andahalf(0.82, elevator.pivot_home),
        ALGAE_1andahalf(0.37, elevator.pivot_home),
        ALGAE_PROCESSOR(0.0, elevator.pivot_stowed),
        ALGAE_BARGE(1.68 + Units.inchesToMeters(8 + 1.5 + 1 + barge_fudge_in), elevator.pivot_home.in(Degrees) - 15),

        PRE_HOMING(0.02, elevator.pivot_stowed),
        VELOCITY(0, elevator.pivot_stowed),
        MANUAL(0, Degrees.of(0))
        ;

        public Distance height;
        public Angle pivot;
        public Distance tol = Meters.of(1000);
        elevator_state(double height_m, double angle_deg) {
            height = Meters.of(height_m);
            pivot = Degrees.of(angle_deg);
        } //defines the first height throught the constuctor that each of the object (enum) is getting

        elevator_state(double height_m, double angle_deg, Distance tol) {
            height = Meters.of(height_m);
            pivot = Degrees.of(angle_deg);
            this.tol = tol;
        } //defines the first height throught the constuctor that each of the object (enum) is getting

        elevator_state(double height_m, Angle angle) {
            height = Meters.of(height_m);
            pivot = angle;
        } //defines the first height throught the constuctor that each of the object (enum) is getting


    }

    public final class end_effector {

        public static final Angle roller_nudge = Degrees.of(45);

        public static final AngularVelocity intake_manual = RotationsPerSecond.of(8);

        public static final AngularVelocity intake_fast = RotationsPerSecond.of(30);
        public static final AngularVelocity intake_algae = RotationsPerSecond.of(90);
        public static final AngularVelocity intake_precise = RotationsPerSecond.of(20);
        public static final AngularVelocity spit_coral = RotationsPerSecond.of(30);
        public static final AngularVelocity spit_coral_l4 = RotationsPerSecond.of(110);
        public static final AngularVelocity spit_coral_l1 = RotationsPerSecond.of(20);
        public static final AngularVelocity spit_coral_l1_fast = RotationsPerSecond.of(50);
        public static final AngularVelocity spit_algae = RotationsPerSecond.of(-30);
        public static final AngularVelocity spit_algae_proc = RotationsPerSecond.of(-10);

        public static final Current hold_ball_current = Amps.of(50);
        public static final double hold_ball_max_dutycycle = 0.25;

        public static final Angle auto_spit_rotations = Rotations.of(25);
    }

    public final class elevator {
        public static final Angle pivot_home = Degrees.of(112);
        public static final Angle pivot_stowed = Degrees.of(past_home);// pivot_home.minus(Degrees.of(5));
    }

    public final class ramp {
        private static final double fudge = 4.0 / 3.0;

        public static final AngularVelocity intake = RotationsPerSecond.of(40 * fudge);
        public static final AngularVelocity unjam = RotationsPerSecond.of(-50 * fudge);
        public static final AngularVelocity idle = RotationsPerSecond.of(15 * fudge);
        public static final AngularVelocity pre_intake = RotationsPerSecond.of(6 * fudge);
    }

    public final class climb {

    }

    public final class swerve {
        public enum module_e {
            mk4i_L1(8.14, 150.0/7.0, (25.0 / 19.0) * (15.0 / 45.0), FeetPerSecond.of(12.4)),
            mk4i_L2(6.75, 150.0/7.0, (27.0 / 17.0) * (15.0 / 45.0), FeetPerSecond.of(15.0)),
            mk4i_L3(6.12, 150.0/7.0, (28.0 / 16.0) * (15.0 / 45.0), FeetPerSecond.of(16.5)),
            ;

            public final double drive_ratio, steer_ratio, couple_ratio;
            public final LinearVelocity free_speed;
            module_e(double drive_ratio, double steer_ratio, double couple_ratio, LinearVelocity free_speed) {
                this.drive_ratio = drive_ratio;
                this.steer_ratio = steer_ratio;
                this.couple_ratio = couple_ratio;
                this.free_speed = free_speed;
            }
        }

        private static final int default_odom_freq = 100;
        private static final int canivore_odom_freq = 250;
        public static final int odom_freq = (RobotBase.isSimulation() || config.swerve.ctre_pro) ? canivore_odom_freq : default_odom_freq;
        public static final double odom_dts = 1.0/odom_freq;

        public static final module_e module = module_e.mk4i_L2;
        private static final double k = 1.01843; 
        public static final double wheel_diameter = Units.inchesToMeters(3.75) * k;
        public static final double wheel_radius = wheel_diameter / 2.0;

        public static final double half_wheelbase_meters = Units.inchesToMeters(26) / 2.0;
        public static final double half_trackwidth_meters = Units.inchesToMeters(22.5) / 2.0;
        public static final double max_module_speed_mps = module.free_speed.in(MetersPerSecond) * 0.95;
        public static final double max_speed_mps = module.free_speed.in(MetersPerSecond) * 0.95;
        public static final double slew_strafe_accel = max_speed_mps / 0.1; // 0 to 4.3 in 200ms
        public static final double slew_omega = 9.0 / 0.15; // 0 to 9 radps in 150ms

        public static final Translation2d offset_fr = new Translation2d(half_wheelbase_meters, -half_trackwidth_meters);
        public static final Translation2d offset_fl = new Translation2d(half_wheelbase_meters, half_trackwidth_meters);
        public static final Translation2d offset_br = new Translation2d(-half_wheelbase_meters, -half_trackwidth_meters);
        public static final Translation2d offset_bl = new Translation2d(-half_wheelbase_meters, half_trackwidth_meters);

        public static final Translation2d[] module_offsets = { offset_fr, offset_fl, offset_br, offset_bl };

        public static final Matrix<N3, N1> mt2_st_devs = VecBuilder.fill(0.4, 0.4, 999999999);
    }
}
