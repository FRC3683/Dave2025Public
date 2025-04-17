package frc.robot.utils;

import static edu.wpi.first.units.Units.Celsius;

import java.util.concurrent.atomic.AtomicInteger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class talon_temps_safety {

    public static final int CHILLIN = 0;
    public static final int TOASTY = 1;
    public static final int CUTOFF = 2;

    public talon_temps_safety(TalonFX device, String dashboard_name, Temperature warning, Temperature error) {
        this.dashboard_name = dashboard_name;
        device_temp_signal = device.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(4, device_temp_signal);

        this.warning_temp = warning;
        this.error_temp = error;
        state.set(CHILLIN);
    }

    public void periodic() {
        if(RobotBase.isSimulation()) {
            return;
        }

        var device_temp = device_temp_signal.refresh().getValue();

        boolean chillin = device_temp.lte(warning_temp);

        boolean error = device_temp.gte(error_temp);

        state.set(error ? CUTOFF : (chillin ? CHILLIN : TOASTY));

        SmartDashboard.putNumber(dashboard_name + "device temp", device_temp.in(Celsius));
    }

    public int get() {
        return state.get();
    }

    public boolean cutoff() {
        return state.get() == CUTOFF;
    }

    public boolean toasty() {
        int state = this.state.get();
        return state == TOASTY || state == CUTOFF;
    }

    private final StatusSignal<Temperature> device_temp_signal;
    private final Temperature warning_temp, error_temp;
    private final String dashboard_name;
    private final AtomicInteger state = new AtomicInteger();
}
