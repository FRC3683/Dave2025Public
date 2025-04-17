package frc.robot.utils;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

import com.fasterxml.jackson.annotation.JsonIgnore;

public class dave_path {
    public static enum waypoint_type {
        line, arc, point, assisted
    }

    public static enum direction {
        clockwise, anti_clockwise
    }

    public static enum event {
        BEGIN_INTAKE,
        END_INTAKE,
        BEGIN_SCORE_SEQUENCE,
        CANCEL_SCORE_SEQUENCE
        // etc... add during season
    }

    public String version = "1.0.0";
    public boolean use_start_pose = true;
    public dave_pose start_pos = new dave_pose(1, 1, 0);
    public List<dave_waypoint> waypoints = new ArrayList<>();

    public dave_path() {} // empty constructor required for jackson parsing

    public dave_path(boolean use_start_pose) {
       this.use_start_pose = use_start_pose; 
    }

    public dave_path(dave_pose start_pos) {
        this.start_pos = start_pos;
        use_start_pose = true;
    }

    public void copy(dave_path other) {
        version = other.version;
        use_start_pose = other.use_start_pose;
        start_pos = other.start_pos;
        waypoints = other.waypoints;
    }

    public dave_waypoint find(dave_point point) {
        for(var waypoint : waypoints) {
            if(waypoint.point == point) {
                return waypoint;
            }
            if(waypoint.type == waypoint_type.arc && waypoint.point == point) {
                return waypoint;
            }
            if((waypoint.type == waypoint_type.assisted || waypoint.type == waypoint_type.line) && waypoint.line_point == point) {
                return waypoint;
            }
        }
        return null;
    }

    public dave_point closest_within(dave_point to, double within_m) {
        if(to == null) {
            return null;
        }
        dave_point closest = null;
        double closest_m = Double.MAX_VALUE;
        var points = new ArrayList<dave_point>();
        if (use_start_pose) {
            points.add(start_pos.position);
        }
        for(var wp : waypoints) {
            points.add(wp.point);
            if(wp.type == waypoint_type.line || wp.type == waypoint_type.assisted) {
                points.add(wp.line_point);
            }
            if(wp.type == waypoint_type.arc) {
                points.add(wp.arc_center);
            }
        }
        points.removeIf((dave_point p) -> p == null);
        for(var point : points) {
            double dist = point.dist_to(to);
            if(dist < within_m && dist < closest_m) {
                closest = point;
                closest_m = dist;
            }
        }
        return closest;
    }

    public dave_point last() {
        if(waypoints.isEmpty()) {
            return start_pos.position;
        }
        return waypoints.get(waypoints.size() - 1).point;
    }

    public void add_waypoint(dave_waypoint waypoint) {
        waypoints.add(waypoint);
    }

    public void insert_waypoint(dave_waypoint to_insert, int index) {
        waypoints.add(index, to_insert);
    }

    public int index_of(dave_waypoint waypoint) {
        return waypoints.indexOf(waypoint);
    }

    public dave_point prev_point_of(dave_point point) {
        for (int i = 0; i < waypoints.size(); ++i) {
            if (waypoints.get(i).point == point && i > 0) {
                return waypoints.get(i - 1).point;
            }
        }
        return null;
    }

    public void remove_by_point(dave_point point) {
        for (int i = 0; i < waypoints.size(); ++i) {
            var wp = waypoints.get(i);
            if (wp.point == point) {
                waypoints.remove(i);
                break;
            }
        }
    }

    public static class dave_point {
        public double x = 0;
        public double y = 0;
        public String link = null;

        public dave_point() {} // empty constructor required for jackson parsing

        public dave_point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public dave_point(double x, double y, String link) {
            this.x = x;
            this.y = y;
            this.link = link;
        }

        public dave_point(dave_point other) {
            x = other.x;
            y = other.y;
        }

        public static dave_point from_direction(double angle_degrees, double length) {
            var angle_rad = Math.toRadians(angle_degrees);
            var x = Math.cos(angle_rad);
            var y = Math.sin(angle_rad);
            return new dave_point(x * length, y * length);
        }

        public dave_point plus(dave_point other) {
            return new dave_point(x + other.x, y + other.y);
        }

        public dave_point unary_minus() {
            return new dave_point(-x, -y);
        }

        public dave_point minus(dave_point other) {
            return this.plus(other.unary_minus());
        }

        public dave_point times(double scalar) {
            return new dave_point(x * scalar, y * scalar);
        }

        public dave_point hat() {
            double l = hypot();
            return new dave_point(x / l, y / l);
        }

        public dave_point round(int decimals) {
            double d = Math.pow(10, decimals);
            x = Math.round(x * d) / d;
            y = Math.round(y * d) / d;
            return this;
        }

        public double hypot() {
            return Math.hypot(x, y);
        }

        public double dist_to(dave_point other) {
            return Math.hypot(other.x - x, other.y - y);
        }

        public double dist_to_sq(dave_point other) {
            double dx = other.x - x;
            double dy = other.y - y;
            return dx * dx + dy * dy;
        }

        public double angle_degrees() {
            return Math.toDegrees(Math.atan2(y, x));
        }

        @Override
        public String toString() {
            return "(" + x + ", " + y + ": " + link + ")";
        }
    }

    public static class dave_waypoint {
        public String name = null;
        public waypoint_type type = waypoint_type.line;
        public dave_point point = new dave_point(0, 0);
        public double through_vel = 0;
        public double tolerance_m = 0.1;
        public double max_vel = 4.5;
        public double heading = 0;
        public dave_point line_point = new dave_point(0, 0);
        public dave_point arc_center = new dave_point(0, 0);;
        public direction arc_direction = direction.anti_clockwise;
        public Set<event> events = EnumSet.of(event.BEGIN_INTAKE); 

        public dave_waypoint() {} // empty constructor required for jackson parsing

        public dave_waypoint(dave_point point) {
            this.point = point;
        }

        public dave_waypoint(dave_point point, dave_point prev) {
            this.point = point;
            line_point = point.plus(prev).times(0.5);
        }

        @JsonIgnore
        public void set_point(dave_point to) {
            point.x = to.x;
            point.y = to.y;
        }

        @JsonIgnore
        public void set_line_angle_degrees(double angle_degrees) {
            var diff = line_point.minus(point);
            var old_dist = diff.hypot();
            var new_diff = dave_point.from_direction(angle_degrees, old_dist);
            line_point.x = point.x - new_diff.x;
            line_point.y = point.y - new_diff.y;
        }

        @JsonIgnore
        public void set_line_angle_degrees(dave_point p) {
            line_point.x = p.x;
            line_point.y = p.y;
        }

        @JsonIgnore
        public double line_angle_degrees() {
            return point.minus(line_point).angle_degrees();
        }
    }

    public static class dave_pose {
        public dave_point position = new dave_point(0, 0);
        public double angle_degrees = 0;

        public dave_pose() {} // empty constructor required for jackson parsing

        public dave_pose(double x, double y, double angle) {
            position.x = x;
            position.y = y;
            angle_degrees = angle;
        }

        public dave_pose(dave_point position, double angle_degrees) {
            this.position = position;
            this.angle_degrees = angle_degrees;
        }

        public void set_position(dave_point position) {
            this.position.x = position.x;
            this.position.y = position.y;
        }
    }
}