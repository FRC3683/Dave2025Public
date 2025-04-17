package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants;

public class elevator_mech2d {
    private MechanismLigament2d target_height, actual_height, target_wrist, actual_wrist,
        target_roller, actual_roller,
        target_ramp, actual_ramp;

    public static final elevator_mech2d sim = new elevator_mech2d();

    private elevator_mech2d() {
        var mech = new Mechanism2d(3, 3);
        var root = mech.getRoot("elevator", 1.3, 0);
        var ramp_root = mech.getRoot("ramp", 0.1, 1);
        target_height = root.append(new MechanismLigament2d("elevator_target", 1, 90));
        target_wrist = target_height.append(new MechanismLigament2d("wrist_target", 1, -90));
        target_roller = target_wrist.append(new MechanismLigament2d("roller_target", 0.1, 0));
        target_ramp = ramp_root.append(new MechanismLigament2d("ramp_target", 0.1, 0));
        target_height.setColor(new Color8Bit(Color.kAqua));
        target_wrist.setColor(new Color8Bit(Color.kAqua));
        target_roller.setColor(new Color8Bit(Color.kAqua));
        target_ramp.setColor(new Color8Bit(Color.kAqua));
        actual_height = root.append(new MechanismLigament2d("elevator_actual", 1, 90));
        actual_wrist = actual_height.append(new MechanismLigament2d("wrist_actual", 1, -90));
        actual_roller = actual_wrist.append(new MechanismLigament2d("roller_actual", 0.1, 0));
        actual_ramp = ramp_root.append(new MechanismLigament2d("ramp_actual", 0.1, 0));
        SmartDashboard.putData("elevator", mech);
    }

    public void set_elevator_pos(Distance target, Distance actual) {
        target_height.setLength(target.in(Meters));
        actual_height.setLength(actual.in(Meters));
    }

    public void set_wrist_angle(Angle target, Angle actual) {
        target_wrist.setAngle(target.in(Degrees)-90);
        actual_wrist.setAngle(actual.in(Degrees)-90);
    }

    public void update_roller(Angle target, Angle actual) {
        target_roller.setAngle(target.in(Degrees));
        actual_roller.setAngle(actual.in(Degrees));
    }

    public void update_roller(AngularVelocity target_speed, AngularVelocity actual_speed) {
        target_roller.setAngle(target_roller.getAngle() + target_speed.in(DegreesPerSecond) * constants.control_dts);
        actual_roller.setAngle(actual_roller.getAngle() + actual_speed.in(DegreesPerSecond) * constants.control_dts);
    }

    public void update_ramp(AngularVelocity target_speed, AngularVelocity actual_speed) {
        target_ramp.setAngle(target_ramp.getAngle() + target_speed.in(DegreesPerSecond) * constants.control_dts);
        actual_ramp.setAngle(actual_ramp.getAngle() + actual_speed.in(DegreesPerSecond) * constants.control_dts);
    }
}
