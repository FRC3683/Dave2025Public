package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class climb_mech2d {
    private Mechanism2d mech;
    private MechanismLigament2d servo_target, climb_target, climb_actual;

    public climb_mech2d() {
        mech = new Mechanism2d(3, 3);
        var servo_root = mech.getRoot("servo", 2, 2);
        var root = mech.getRoot("climb", 1, 1);
        servo_target = servo_root.append(new MechanismLigament2d("climb_servo", 1, 0));
        climb_target = root.append(new MechanismLigament2d("climb_target", 1, 0));
        climb_actual = root.append(new MechanismLigament2d("climb_actual", 1, 0));
        climb_target.setColor(new Color8Bit(Color.kAqua));

        SmartDashboard.putData("climb", mech);
    }

    public void update_servo(Angle target) {
        var current_degrees = servo_target.getAngle();
        servo_target.setAngle(MathUtil.interpolate(current_degrees, target.in(Degrees), 0.1));
    }

    public void update_climb(Angle target, Angle actual) {
        climb_target.setAngle(target.in(Degrees));
        climb_actual.setAngle(actual.in(Degrees));
    }
}
