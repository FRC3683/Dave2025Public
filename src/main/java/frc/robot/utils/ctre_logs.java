package frc.robot.utils;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ctre_logs {
    public static void init() {
        SignalLogger.enableAutoLogging(false);
        if(RobotBase.isReal()) {
            var file = new File("/media/sda/ctre_logs/"); // path to first USB found on rio
            SmartDashboard.putBoolean("usb_present", file.exists());
            long free_space_bytes = file.getFreeSpace();
            SignalLogger.setPath(file.getAbsolutePath());
            var logging_enabled = free_space_bytes > 1_000_000_000; // 1 GB
            SmartDashboard.putBoolean("logging_enabled", logging_enabled);
            SignalLogger.enableAutoLogging(logging_enabled);
        }
    }
}
