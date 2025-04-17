package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.swerve;
import frc.robot.utils.LL_connected_check;
import frc.robot.utils.dave_led;
import frc.robot.utils.math_utils;
import frc.robot.utils.voltage_warning;

public class leds extends dave_led {

    private final LL_connected_check LL_connected_left = new LL_connected_check(config.LL_left_name);
    private final LL_connected_check LL_connected_right = new LL_connected_check(config.LL_right_name);
    private final swerve swerve;

    public final flag flag_configuring = new flag();

    public static final flag flag_intake_flash = new flag();
    public static final flag flag_auto_intake = new flag();
    public final flag flag_operator_flash = new flag();
    public final flag flag_current_draw = new flag();
    public final flag flag_LL_no_target = new flag();
    public final flag flag_flash_zero = new flag();
    public static final flag flag_flash_climb = new flag();
    public static final flag flag_solid_climb = new flag();

    public leds(TimedRobot robot, swerve swerve) {
        super(config.pwm_leds, config.leds.length, robot);
        this.swerve = swerve;

        robot.addPeriodic(this::periodic, constants.control_dts);
    }

    private void handle_flags() {
        if(flag_flash_climb.get()) {
            set_fast_flicker(clr_white);
            return;
        }

        if(flag_solid_climb.get()) {
            set_color(clr_white);
            return;
        }

        if(flag_flash_zero.get()) {
            set_fast_flicker(clr_cyan);
            return;
        }

        if(flag_LL_no_target.get()) {
            set_fast_flicker(clr_purple);
            return;
        }

        if(flag_operator_flash.get()) {
            set_fast_flicker(clr_yellow);
            return;
        }

        if(DriverStation.isDisabled()) {
            return;
        }

        if(flag_intake_flash.get()) {
            set_fast_flicker(bindings.abxy_color);
            return;
        }

        if(flag_auto_intake.get()) {
            set_color(clr_white);
            return;
        }

        if (flag_current_draw.get()) {
            set_color(clr_orange);
            return;
        }

        set_color(bindings.abxy_color);
    }

    @Override
    protected void periodic() {
        boolean disabled = DriverStation.isDisabled();
        if(flag_configuring.get() || !robot.zerod.get()) {
            set_color(0, length/2, clr_blue);
            set_color(length/2, length, clr_purple);
            return;
        }

        if(disabled) {
            set_color(0, length, clr_off);

            Color8Bit disabled_color = clr_purple;
            if(DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
                switch(DriverStation.getAlliance().get()) {
                    case Blue: disabled_color = clr_blue; break;
                    case Red:  disabled_color = clr_red; break;
                }
            }

            if(DriverStation.isTest()) {
                rainbow(5);
            } else {
                set_trail(disabled_color, 0, length-4);
            }
        }

        handle_flags();

        if(bindings.manual_intake) {
            set_color(length-5, length, clr_orange);
        }


        // most important, override other led codes
        if(RobotBase.isReal()) {
            boolean error = false;
            error = error || !voltage_warning.check();
            SmartDashboard.putBoolean("estopped", !DriverStation.isEStopped());
            error = error || DriverStation.isEStopped();
            error = error || !swerve.has_abs();
            boolean joysticks = DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1);
            SmartDashboard.putBoolean("joysticks", joysticks);
            error = error || !joysticks;
            boolean LL_connected = LL_connected_left.connected() && LL_connected_right.connected();
            error = error || !LL_connected;
            boolean toasty = swerve.toasty_motors();
            error = error || toasty;
            if(error) {
                set_checker(0, length / 4, clr_cyan, clr_orange);
            }
        }

        if(disabled) {
            set_single(length-1, LimelightHelpers.getTV(config.LL_left_name) ? clr_green : clr_red);
            set_single(length-2, LimelightHelpers.getTV(config.LL_right_name) ? clr_green : clr_red);

            boolean lineup = swerve.close_to_any_reef_angle();
            set_single(length - 4, lineup ? clr_green : clr_red);
            set_single(length - 3, lineup ? clr_green : clr_red);
            SmartDashboard.putBoolean("auto lineup", lineup);
        }
    }

}
