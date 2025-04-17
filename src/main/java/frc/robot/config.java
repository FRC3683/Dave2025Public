package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.swerve.max_speed_mps;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.controls.drivetrain_controller;

public final class config {

    public static boolean is_comp = !RobotController.getComments().equals("practice");
    static {
        SmartDashboard.putString("name", RobotController.getComments());
        SmartDashboard.putBoolean("is_comp", is_comp);
    }
    
    public static final boolean dev = true; // extra networktable values for code stuff... unused? lol

    public static final String can_ivore = "canivore";
    public static final String can_rio = "rio";
    
    public static final can swerve_fl_turn = new can(3, can_ivore);
    public static final can swerve_fl_drive = new can(6, can_ivore);

    public static final can swerve_fr_turn = new can(9, can_ivore);
    public static final can swerve_fr_drive = new can(8, can_ivore);

    public static final can swerve_bl_turn = new can(5, can_ivore);
    public static final can swerve_bl_drive = new can(7, can_ivore);

    public static final can swerve_br_turn = new can(4, can_ivore);
    public static final can swerve_br_drive = new can(2, can_ivore);

    public static final can pigeon = new can(10, can_ivore);

    public static final can can_climb = new can(11, can_rio);
    public static final can can_elevator_left = new can(12, is_comp ? can_ivore : can_rio);
    public static final can can_elevator_right = new can(13, is_comp ? can_ivore : can_rio);
    public static final can can_ramp = new can(14, can_rio);
    public static final can can_endeffector_pivot = new can(15, can_rio);
    public static final can can_endeffector_roller = new can(16, can_rio);

    public static final int dio_swerve_fl_abs = 6;
    public static final int dio_swerve_fr_abs = 7;
    public static final int dio_swerve_bl_abs = 8;
    public static final int dio_swerve_br_abs = 9;
    public static final int dio_end_effector_lidar = 5;
    public static final int dio_ramp_lidar_edge = 4;
    public static final int dio_ramp_lidar_middle = 3;
    public static final int dio_elevator_hallfx = 2;

    public static final int pwm_servo_left = 0;
    public static final int pwm_servo_right = 1;
    public static final int pwm_leds = 4;

    // public static final int[] can_loop = { swerve_fl_turn, swerve_fl_drive, 
    //     swerve_fr_turn, swerve_fr_drive, 
    //     swerve_bl_drive, swerve_bl_turn,
    //     swerve_br_turn, swerve_br_drive, pigeon };



    public static final String LL_right_name = "limelight-right";
    public static final String LL_left_name = "limelight-left";
    public static final double pole_spacing_m = Units.inchesToMeters(13);
    public static final double ll_up_m = Units.inchesToMeters(6.75);
    public static final double ll_forward_m = Units.inchesToMeters(10);
    public static final Rotation2d ll_pitch = Rotation2d.fromDegrees(31);
    public static final LL LL_left = new LL(LL_left_name, new Translation3d(ll_forward_m, pole_spacing_m/2.0, ll_up_m), ll_pitch);
    public static final LL LL_right = new LL(LL_right_name, new Translation3d(ll_forward_m, -pole_spacing_m/2.0, ll_up_m), ll_pitch);



    public final class elevator {
        private static CurrentLimitsConfigs limits() {
            return new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(90))
            .withSupplyCurrentLowerLimit(Amps.of(60))
            .withSupplyCurrentLowerTime(Seconds.of(2));
        }

        public static final CurrentLimitsConfigs auto_limits = limits()
            .withStatorCurrentLimit(Amps.of(95));

        public static final CurrentLimitsConfigs tele_limits = limits()
            .withStatorCurrentLimit(Amps.of(95));


        public static final MotionMagicConfigs auto_mm = new MotionMagicConfigs()
            .withMotionMagicAcceleration(is_comp ? 19 : 19)
            .withMotionMagicCruiseVelocity(is_comp ? 3.3 : 3.2)
            .withMotionMagicJerk(300)
        ;

        public static final MotionMagicConfigs tele_mm = new MotionMagicConfigs()
            .withMotionMagicAcceleration(is_comp ? 12 : 12)
            .withMotionMagicCruiseVelocity(is_comp ? 3.3 : 3.3)
            .withMotionMagicJerk(300)
        ;

        private static double elevator_fudge = is_comp ? 0.95 : 1.1;

        public static TalonFXConfiguration get_config(InvertedValue inversion) {
            return new TalonFXConfiguration()
                .withSlot0(new Slot0Configs()
                    .withKG(0.35)
                    .withKS(0.1)
                    .withKP(100.0)
                    .withKV(3.3)
                    .withKA(0.1) // reca.lc
                )
                .withSlot1(new Slot1Configs()
                    .withKG(0.35)
                    .withKS(0.1)
                    .withKP(1.0)
                    .withKV(3.5)
                    .withKA(0.07) // reca.lc
                )
                .withMotionMagic(auto_mm)
                .withCurrentLimits(auto_limits)
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Coast)
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio( (54.0 / 18.0) / 30.0 / 0.005 * elevator_fudge ) // ( ratio / pulley teeth / belt tooth spacing )
                )
                .withVoltage(new VoltageConfigs()
                    .withPeakReverseVoltage(is_comp ? -12 : -5) // comp uses DynamicMotionMagic because canivore yay
                )
                .withAudio(new AudioConfigs()
                    .withAllowMusicDurDisable(true)
                )
            ;
        }

        static final double pivot_ratio = is_comp ? (50.0 / 8.0) * (42.0 / 15.0) : (50.0 / 9.0) * (42.0 / 15.0);

        static final Slot0Configs prac_pivot_slot0 = new Slot0Configs()
            .withKP(35.0)
            .withKS(0.1)
            .withKV(1.5)
            .withKA(0.05)
            .withKG(0.25)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        ;

        static final Slot0Configs comp_pivot_slot0 = new Slot0Configs()
            .withKP(10)
            .withKI(1)
            .withKV(1.0)
            .withKA(0.06)
            .withKG(0.49)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        ;

        static final Slot1Configs prac_pivot_slot1 = new Slot1Configs()
            .withKV(10)
            .withKP(1.0)
            .withKG(0.0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        ;

        static final Slot1Configs comp_pivot_slot1 = new Slot1Configs()
            .withKV(10)
            .withKP(1.0)
            .withKG(0.0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        ;

        static final Current pivot_current_limit = is_comp ? Amps.of(40) : Amps.of(26);

        public static TalonFXConfiguration pivot_config = new TalonFXConfiguration()
            .withSlot0(is_comp ? prac_pivot_slot0 : prac_pivot_slot0)
            .withSlot1(is_comp ? prac_pivot_slot1 : comp_pivot_slot1)
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(pivot_ratio)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(3.5))
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(7.0))
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(pivot_current_limit)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(60))
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            )
            .withAudio(new AudioConfigs()
                .withAllowMusicDurDisable(true)
            )
        ;
    }

    public final class end_effector {
        private static final TalonFXConfiguration practice_roller_config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0.2)
                .withKD(0.005)
                .withKS(.38)
                .withKV(0.15)
            )
            .withSlot1(new Slot1Configs()
                .withKS(0.38)
                .withKV(0)
                .withKP(10)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(40))
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(60))
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            )
            .withAudio(new AudioConfigs()
                .withAllowMusicDurDisable(true)
            )
        ;

        private static final TalonFXConfiguration comp_roller_config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(is_comp ? 0.25 : 0.2)
                .withKD(0.005)
                .withKS(.38)
                .withKV(is_comp ? 0.2 : 0.15)
            )
            .withSlot1(new Slot1Configs()
                .withKS(0.38)
                .withKV(0)
                .withKP(10)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(is_comp ? 70 : 40))
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(is_comp ? 80 : 60))
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            )
            .withAudio(new AudioConfigs()
                .withAllowMusicDurDisable(true)
            )
        ;

        public static final TalonFXConfiguration roller_config = is_comp ? comp_roller_config : practice_roller_config;
    }

    public static class climb {
        public static TalonFXConfiguration config = new TalonFXConfiguration()
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                // .withForwardSoftLimitEnable(true)
                // .withForwardSoftLimitThreshold(0)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(100)
                .withMotionMagicCruiseVelocity(100)
            )
            .withSlot0(new Slot0Configs()
                .withKP(1.0)
                .withKV(0.01)
            )
            .withSlot1(new Slot1Configs()
                .withKP(1.0)
                .withKV(0.01)
                .withKG(0.5)
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12)
                .withPeakReverseVoltage(0)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
            )
            .withAudio(new AudioConfigs()
                .withAllowMusicDurDisable(true)
            )
        ;
    }

    public final class ramp {
        public static TalonFXConfiguration config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0.06)
                .withKD(0.0)
                .withKS(0.15)
                .withKV(0.09)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(20))
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(60))
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            )
            .withAudio(new AudioConfigs()
                .withAllowMusicDurDisable(true)
            )
        ;
    }

    public final class swerve {

        public static final boolean ctre_pro = true;

        private static final double drive_kS = 0.22;
        private static final double max_speed_rotations_ps = Units.radiansToRotations(constants.swerve.max_module_speed_mps / constants.swerve.wheel_radius);
        private static final double drive_kV = (12.0 - drive_kS) / max_speed_rotations_ps;

        public static CurrentLimitsConfigs current_limits() {
            return new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(90))
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(Amps.of(60))
                .withSupplyCurrentLowerTime(Seconds.of(2))
            ;
        }

        public static CurrentLimitsConfigs auto_current_limits = current_limits()
            .withStatorCurrentLimit(125);

        public static CurrentLimitsConfigs tele_current_limits = current_limits()
            .withStatorCurrentLimit(85);

        public static TalonFXConfiguration drive_configs(InvertedValue inversion) {
            return new TalonFXConfiguration()
                .withCurrentLimits(auto_current_limits)
                .withSlot0(new Slot0Configs()
                    .withKV(drive_kV)
                    .withKP(2.0)
                    .withKS(drive_kS)
                    .withKD(0.0)
                    .withKI(0.0)
                )
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(0.01)
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(constants.swerve.module.drive_ratio)
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withDutyCycleNeutralDeadband(0.05)
                )
                .withTorqueCurrent(new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(150)
                    .withPeakReverseTorqueCurrent(-150)
                )
                .withAudio(new AudioConfigs()
                    .withAllowMusicDurDisable(true)
                )
            ;
        }

        public static TalonFXConfiguration turn_configs(InvertedValue inversion) {
            var closed_loop = new ClosedLoopGeneralConfigs();
            closed_loop.ContinuousWrap = true;
            return new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(60)
                    .withSupplyCurrentLimitEnable(true)
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(constants.swerve.module.steer_ratio)
                )
                .withSlot0(new Slot0Configs()
                    .withKV(2.0) // TODO tune
                    .withKP(100)
                    .withKD(0.0)
                )
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(0.02)
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Coast)
                )
                .withClosedLoopGeneral(closed_loop)
                .withAudio(new AudioConfigs()
                    .withAllowMusicDurDisable(true)
                )
            ;
        }

        public static final Pigeon2Configuration pig_config = new Pigeon2Configuration()
            .withMountPose(new MountPoseConfigs()
                .withMountPosePitch(Degrees.of(is_comp ? 0 : 0))
            )
        ;

        public static drivetrain_controller.configuration theta_config = new drivetrain_controller.configuration()
            .with_pid(8.0, 0.0, 0.0)
            .with_pid_threshold(Units.degreesToRadians(10))
            .with_max_vel(Units.degreesToRadians(500))
            .with_max_accel(Units.degreesToRadians(650))
            .with_continuous_input(-Math.PI, Math.PI)
            .with_epsilon(Units.degreesToRadians(5.0))
            .with_deadzone(Units.degreesToRadians(0.5))
        ;
        public static drivetrain_controller.configuration strafe_config = new drivetrain_controller.configuration()
            .with_pid(14.0, 0.0, 0.2)
            .with_pid_threshold(0.1)
            .with_max_vel(constants.swerve.max_speed_mps)
            .with_max_accel(7.5)
            .with_epsilon(0.07)
        ;
        public static drivetrain_controller.configuration sttbolls_auton_x_config = new drivetrain_controller.configuration()
            .with_pid(5.5, 0.0, 0.2)
            .with_pid_threshold(0.1)
            .with_max_vel(max_speed_mps)
            .with_max_accel(3.0)
            .with_epsilon(0.07)
        ;
        public static drivetrain_controller.configuration sttbolls_auton_y_config = new drivetrain_controller.configuration()
            .with_pid(6.0, 0.0, 0.3)
            .with_pid_threshold(0.1)
            .with_max_vel(max_speed_mps)
            .with_max_accel(3.0)
            .with_epsilon(0.07)
        ;
        public static drivetrain_controller.configuration sttbolls_tele_config = new drivetrain_controller.configuration()
            .with_pid(5.5, 0.0, 0.2)
            .with_pid_threshold(0.15)
            .with_max_vel(4.0)
            .with_max_accel(3.0)
            .with_epsilon(0.08)
        ;

        public static final double pid_line_y_weight = 0.95;

        public static final class module_config {
            public final String name;
            public final can can_drive, can_turn;
            public final int dio_abs;
            public final InvertedValue drive_inverted;
            public final InvertedValue turn_inverted;
            public final double comp_abs_offset, prac_abs_offset;

            public module_config(String name, can can_drive, can can_turn, int dio_abs, 
                    InvertedValue drive_inverted, InvertedValue turn_inverted, 
                    double comp_abs_offset, double prac_abs_offset) {
                this.name = name;
                this.can_drive = can_drive;
                this.can_turn = can_turn;
                this.dio_abs = dio_abs;
                this.drive_inverted = drive_inverted;
                this.turn_inverted = turn_inverted;
                this.comp_abs_offset = comp_abs_offset;
                this.prac_abs_offset = prac_abs_offset;
            }
        }

        public static final module_config[] module_configs = {
            new module_config("fr", swerve_fr_drive, swerve_fr_turn, dio_swerve_fr_abs,
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive,
                1.7340399017720274, 2.6765),

            new module_config("fl", swerve_fl_drive, swerve_fl_turn, dio_swerve_fl_abs,
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive,
                2.658128809795839, 3.8414),

            new module_config("br", swerve_br_drive, swerve_br_turn, dio_swerve_br_abs,
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive,
                0.3974268720077226, 0.6872),

            new module_config("bl", swerve_bl_drive, swerve_bl_turn, dio_swerve_bl_abs,
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive,
                5.791394594994436, 2.7687),
        };
    }

    public final class leds {
        public static final int length = is_comp ? 13 : 13;
    }

    public static final class LL {
        public final String name;
        public final Translation3d mount_offset; // from center of robot, height from ground
        public final Rotation2d mount_angle; // from vertical to lense normal

        public LL(String name, Translation3d mount_offset, Rotation2d mount_angle) {
            this.name = name;
            this.mount_offset = mount_offset;
            this.mount_angle = mount_angle;
        }
    }

    public static class can {
        public int can_id;
        public String canbus;

        public can(int can_id, String canbus) {
            this.can_id = can_id;
            this.canbus = canbus;
        }
    }
}
