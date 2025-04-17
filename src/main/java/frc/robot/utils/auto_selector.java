package frc.robot.utils;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot;

public final class auto_selector {

    private auto[] autos_bank1;
    private auto[] autos_bank2;

    private auto selected_auto;
    private Command auto_command;

    private boolean selected_shift = false;
    private int selected_index = 0;
    private Optional<Alliance> alliance = Optional.empty();
    private static Consumer<Pose2d> reset_pose;
    
    private final int[] select_buttons = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    private final int bank_size = select_buttons.length;
    private final int shift_button = bank_size+1;
    private final boolean default_shift = false;
    private final int default_auto = 0;

    public auto_selector(auto[] bank1, auto[] bank2, Consumer<Pose2d> reset_pose) {
        auto_selector.reset_pose = reset_pose;
        if(bank1.length > bank_size) {
            System.err.println("too many autos in bank1! only the first " + bank_size + " will be available!");
        }
        if(bank2.length > bank_size) {
            System.err.println("too many autos in bank2! only the first " + bank_size + " will be available!");
        }

        autos_bank1 = Arrays.copyOf(bank1, bank_size);
        autos_bank2 = Arrays.copyOf(bank2, bank_size);
        for(int i = bank1.length; i < bank_size; ++i) {
            autos_bank1[i] = auto.nothing();
        }
        for(int i = bank2.length; i < bank_size; ++i) {
            autos_bank2[i] = auto.nothing();
        }
    }

    public void init() {
        select(default_shift, default_auto);

        for(int i = 0; i < bank_size; ++i) {
            final int index = i;
            oi.cmd_xkeys.button(select_buttons[i]).onTrue(Commands.runOnce(() -> {
                if(!DriverStation.isDisabled()) {
                    return;
                }
                select(oi.cmd_xkeys.button(shift_button).getAsBoolean(), index);
            }).ignoringDisable(true));
        }

        new Trigger(() -> DriverStation.getAlliance().isPresent() && alliance.isEmpty())
            .or(() -> DriverStation.getAlliance().isPresent() && alliance.isPresent() && DriverStation.getAlliance().get() != alliance.get()).onTrue(Commands.runOnce(() -> {
            select(selected_shift, selected_index);
        }).ignoringDisable(true));
    }

    private void select(boolean shift, int index) {
        if(DriverStation.getAlliance().isEmpty()) {
            return;
        }
        selected_index = index;
        selected_shift = shift;
        var bank = shift ? autos_bank2 : autos_bank1;
        selected_auto = bank[index];
        var pair = selected_auto.supplier.get();
        auto_command = pair.getFirst();
        var pose = pair.getSecond();
        if(pose != null) {
            reset_pose.accept(pose);
        }
        alliance = DriverStation.getAlliance();
        // one time the robot generated the auto before connecting to FMS, so it dispayed the correct auto on elastic but 
        // had generated for the wrong alliance color. lmao. displaying generated alliance.get().name() was the hotfix.
        SmartDashboard.putString("auto", alliance.get().name() + " " + selected_auto.shuffleboard_string);
        SmartDashboard.putString("auto_c1", selected_auto.led_color1.toHexString());
        SmartDashboard.putString("auto_c2", selected_auto.led_color2.toHexString());
        System.gc();
    }

    public Command get_auto_command() {
        return auto_command;
    }

    public Pair<Color8Bit, Color8Bit> get_auto_color() {
        return Pair.of(selected_auto.led_color1, selected_auto.led_color2);
    }



    private static final Color[] colors = {
        Color.kPink, Color.kRed, Color.kYellow, Color.kGreen, Color.kBlue, Color.kPurple, Color.kWhite
    };

    private static int c1=0, c2=1;

    public static class auto {
        private Supplier<Pair<Command, Pose2d>> supplier;
        private String shuffleboard_string;
        private Color8Bit led_color1, led_color2;

        public auto(String shuffleboard_string, Supplier<Pair<Command, Pose2d>> supplier) {
            this.supplier = supplier;
            if(this.supplier == null) {
                System.err.println("no auto command supplier for auto " + shuffleboard_string);
                this.supplier = () -> Pair.of(Commands.none(), null);
            }
            try {
                this.supplier.get();
            } catch(Exception e) {
                System.err.println("error generating auto " + shuffleboard_string);
            }
            this.shuffleboard_string = shuffleboard_string;
            led_color1 = new Color8Bit(colors[c1]);
            led_color2 = new Color8Bit(colors[c2]);
            c2++;
            if(c2 >= colors.length) {
                c1++;
                c2 = c1+1;
            }
        }

        public static auto from(String shuffleboard_string, Supplier<Command> red, Supplier<Command> blue) {
            return new auto(shuffleboard_string, () -> {
                var alliance = DriverStation.getAlliance().orElse(Alliance.Red);
                var command = alliance == Alliance.Red ? red.get() : blue.get();
                return Pair.of(command, null);
            });
        }

        public static auto from(String shuffleboard_string, Supplier<Command> supplier) {
            return new auto(shuffleboard_string, () -> Pair.of(supplier.get(), null));
        }

        public static auto nothing() {
            return new auto("do nothing auto", () -> Pair.of(Commands.runOnce(() -> { reset_pose.accept(new Pose2d(0, 0, Rotation2d.fromDegrees(robot.is_red() ? 0 : 180))); }), null));
        }
    }
}
