package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class voltage_warning {

    private static double threshold_comp = 12.6;
    private static double threshold_prac = 12.1;
    private static PowerDistribution pd;
    private static boolean ok = true;

    public static void set_thresholds(double voltage_warning_threshold_comp, double voltage_warning_threshold_prac) {
        threshold_comp = voltage_warning_threshold_comp;
        threshold_prac = voltage_warning_threshold_prac;
    }

    public static void add_nt_listener(TimedRobot robot, PowerDistribution pd, int hz) {
        voltage_warning.pd = pd;
        robot.addPeriodic(() -> {
            double voltage = (pd == null) ? RobotController.getBatteryVoltage() : pd.getVoltage();
            double voltage_warning_threshold = DriverStation.isFMSAttached() ? threshold_comp : threshold_prac;
            if(ok && voltage < voltage_warning_threshold) {
                ok = false;
            }
            if(!ok && voltage > voltage_warning_threshold + 0.1) { // hysteresis
                ok = true;
            }
            SmartDashboard.putBoolean("voltage_status", check());
            SmartDashboard.putNumber("pdh temperature C", pd.getTemperature());
            SmartDashboard.putNumber("pdh current draw", pd.getTotalCurrent());
            SmartDashboard.putNumber("pdh voltage", pd.getVoltage());
        }, 1.0 / hz);
    }

    public static boolean check() {
        return ok || DriverStation.isEnabled(); // dont give errors when enabled, voltage drops frequently while robot is running, totally ok
    }
}
