package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.field_util;
import frc.robot.utils.oi;
import frc.robot.utils.swerve_setup_offsets;

public final class debug {
    public static void add_dashboard_commands(robot robot) {
        SmartDashboard.putData("setup swerve offsets", swerve_setup_offsets.make(robot.swerve.modules));

        SmartDashboard.putData("reset pose 5,5", Commands.runOnce(() -> {
            robot.swerve.reset_pose(new Pose2d(5, 5, robot.swerve.get_heading()));
        }).ignoringDisable(true));

        SmartDashboard.putData("reset pose lob", Commands.runOnce(() -> {
            robot.swerve.reset_pose(new Pose2d(field_util.field_size_m.getX()/2, 0.813, robot.swerve.get_heading()));
        }).ignoringDisable(true));

        SmartDashboard.putData("coast stuff", Commands.runOnce(() -> {
            robot.swerve.coast();
        }).ignoringDisable(true));

        SmartDashboard.putData("zero elevator", Commands.runOnce(() -> {
            robot.elevator.zero();
            robot.elevator.home_pivot();
        }).ignoringDisable(true));

        SmartDashboard.putData("tele current limits", Commands.runOnce(() -> {
            robot.swerve.apply_current_limits(false);
            robot.elevator.apply_current_limits(false);
        }).ignoringDisable(true));

        SmartDashboard.putData("auto current limits", Commands.runOnce(() -> {
            robot.swerve.apply_current_limits(true);
            robot.elevator.apply_current_limits(true);
        }).ignoringDisable(true));

        SmartDashboard.putData("reset climb", Commands.runOnce(() -> {
            robot.climb.reset();
        }).ignoringDisable(true));
    }
}
