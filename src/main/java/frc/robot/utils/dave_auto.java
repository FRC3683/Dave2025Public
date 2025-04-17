package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robot;
import frc.robot.utils.dave_path.dave_point;
import frc.robot.utils.dave_path.dave_waypoint;

public class dave_auto {

    public static final String version = "1.0.0";
    public static final Map<dave_path.event, Supplier<Command>> events = new HashMap<>();
    private static final Map<String, Boolean> event_map = new HashMap<>();
    private static swerve swerve;

    private final Queue<Command> commands;
    private final Pose2d starting_pose;

    private dave_auto(dave_path json, boolean reset_odometry) {
        if(!json.version.equals(version)) {
            DriverStation.reportWarning("versions don't match", false);
        }

        starting_pose = reset_odometry ? field_util.fix(new Pose2d(t(json.start_pos.position), Rotation2d.fromDegrees(json.start_pos.angle_degrees)))
            : null;

        commands = new LinkedList<>();
        for(var waypoint : json.waypoints) {
            boolean is_last = (waypoint == json.waypoints.get(json.waypoints.size() - 1));
            Command to_point = build(waypoint, is_last);
            commands.add(to_point);
        }
    }

    public Pose2d get_starting_pose() {
        return starting_pose;
    }

    private dave_auto(dave_path json) { this(json, true); }

    public Command next(int num) {
        var command = Commands.none();
        while(!commands.isEmpty() && num > 0) {
            --num;
            command = command.andThen(commands.poll());
        }
        return command;
    }

    public Command next() {
        return next(1);
    }

    public Command rest() {
        return next(commands.size());
    }

    // path relative to deploy directory
    public static dave_auto from(Path path) {
        try {
            var str = Files.readString(Filesystem.getDeployDirectory().toPath().resolve(path));
            return dave_auto.from(str);
        } catch(IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    public static dave_auto from(String json) {
        var object_mapper = new ObjectMapper();
        try {
            var data = object_mapper.readValue(json, dave_path.class);
            var auto = dave_auto.from(data);
            return auto;
        } catch(JsonMappingException e) {
            e.printStackTrace();
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
        return new dave_auto(new dave_path());
    }

    public static dave_auto from(dave_path json) {
        return new dave_auto(json);
    }

    public static void config(swerve swerve) {
        dave_auto.swerve = swerve;
    }

    public static void add_event(dave_path.event name, Supplier<Command> event) {
        events.put(name, event);
    }

    public static void add_event(dave_path.event name, Runnable runnable) {
        add_event(name, () -> { runnable.run(); });
    }

    public static void add_event(dave_path.event name, Command to_schedule) {
        add_event(name, () -> { to_schedule.schedule(); });
    }

    private static boolean flip() {
        return robot.is_red();
    }


    Translation2d t(dave_point point) {
        return new Translation2d(point.x, point.y);
    }

    void flip(dave_point point) {
        var flipped = field_util.flip(t(point));
        point.x = flipped.getX();
        point.y = flipped.getY();
    }

    
    private Command theta(dave_waypoint wp) {
        // switch(wp.theta_type) {
        //     case "heading":     return swerve.snap(snap_heading_deg);
        //     case "point":       return swerve.snap_trans(snap_point.t(), snap_heading_deg);
        //     case "none":        return Commands.none();
        // }
        return swerve.snap(wp.heading);
    }

    private Command events(dave_waypoint wp) {
        var cmd = Commands.none();
        if(events == null) {
            return cmd;
        }
        for(var event : wp.events) {
            cmd = cmd.alongWith(events.get(event).get());
        }
        return cmd;
    }

    private Command strafe(dave_waypoint wp) {
        switch(wp.type) {
        case line:
        case assisted: // TODO
            return swerve.strafe_line(t(wp.point), Rotation2d.fromDegrees(wp.line_angle_degrees()), wp.max_vel, wp.tolerance_m, wp.through_vel);
        case arc:
            return Commands.none();
        case point:
        default:
            return swerve.strafe_to_point(t(wp.point), wp.max_vel, wp.tolerance_m, wp.through_vel);
        }
    }

    private Command build(dave_waypoint wp, boolean last) {
        if(last) {
            wp.tolerance_m = -1;
        }
        if(flip()) {
            flip(wp.point);
            // if(snap_point != null) {
            //     snap_point.flip();
            // }
            // if(!theta_type.equals("point")) {
            //     snap_heading_deg = field_util.fix(snap_heading_deg);
            // }
        }
        return Commands.parallel(
            strafe(wp),
            events(wp),
            Commands.runOnce(() -> {
                theta(wp).schedule();
            })
        );
    }

    public static interface swerve {
        Command strafe_to_point(final Translation2d target, final double max_vel, final double tolerance, final double through_vel);
        Command strafe_line(Translation2d point, Rotation2d direction, double max_vel, double tolerance, final double through_vel);
        Command snap(double theta_deg);
        Command snap_trans(Translation2d look_at, double offset_deg);
        Pose2d get_pose();
        void reset_pose(Pose2d pose);
    }
}
