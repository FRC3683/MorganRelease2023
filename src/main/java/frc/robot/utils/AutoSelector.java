package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.SSGKob3Blue;
import frc.robot.commands.SSGKob3Red;
import frc.robot.commands.SSGShaqBlue;
import frc.robot.commands.SSGShaqGOATBlue;
import frc.robot.commands.SSGShaqGOATRed;
import frc.robot.commands.SSGShaqPracticeRed;
import frc.robot.commands.SSGShaqRed;
import frc.robot.commands.Auto_Balance;
import frc.robot.utils.GlobalState.ScoringLevel;

public class AutoSelector {
    private static AutoSelector instance;

    public static AutoSelector getInstance() {
        if (instance == null) {
            instance = new AutoSelector();
        }
        return instance;
    }

    enum Auto {
        SHAQ(0, 255, 255), // cyan
        SHAQ_BALANCE(255, 255, 255), // white
        KOB3(255, 255, 0), // yellow
        KOB3_BALANCE(255, 0, 255), // magenta
        // STRONG_SIDE_3_PC(0, 255, 255), // cyan
        // STRONG_SIDE_2_9(255, 100, 0), // orange ish
        // STRONG_SIDE_2_5(255, 255, 0), // yellow
        // WEAK_SIDE_VANILLA(255, 255, 190), // off white ?
        // CENTER(255, 0, 255), // magenta
        ;

        final Color8Bit color;

        Auto(int R, int G, int B) {
            color = new Color8Bit(R / 10, G / 10, B / 10);
        }
    }

    // new Auto(new Auto_Balance(false, false), new Auto_Balance(false, false),
    // "balance", 0, 0, 255), // blue
    // new Auto(new Auto_StrongSideRed(true), new Auto_StrongSideBlue(true), "Strong
    // side balance", 255, 0, 255), // magenta
    // new Auto(new Auto_StrongSideRed(false), new Auto_StrongSideBlue(false),
    // "Strong side 3 pc", 0, 255, 255), // cyan
    // new Auto(new Auto_StrongSideHighRed(), new Auto_StrongSideHighBlue(), "Strong
    // side high", 255, 50, 0), // orange
    // new Auto(new Auto_2PcCblbmpRed(true), new Auto_2PcCblbmpBlue(true), "Weak
    // side balance", 0, 255, 0), // green
    // new Auto(new Auto_2PcCblbmpRed(false), new Auto_2PcCblbmpBlue(false), "Weak
    // side 2.5", 255, 255, 0), // yellow
    // new Auto(new Auto_3PcCblbmpRed(), new Auto_3PcCblbmpBlue(), "3pc cable bump",
    // 255, 0, 0) // red

    private final Xkey60 xkeys;

    private int index;
    private Auto auto;
    private Auto generatedAuto;
    private CommandBase command;

    public AutoSelector() {
        xkeys = Xkey60.getInstance();
        index = 0;
        auto = Auto.values()[index];
        SmartDashboard.putString("auto cmd", auto.name());
        generatedAuto = null;
        command = null;
    }

    public void disabledPeriodic() {
        if (xkeys.autoCycle()) {
            ++index;
            index %= Auto.values().length;
            auto = Auto.values()[index];
            SmartDashboard.putString("auto cmd", auto.name());
            // System.out.printf("auto cmd: %s\n", auto.name());
        }
        if (generatedAuto == null || xkeys.autoGenerate()) {
            boolean red = (GlobalState.getInstance().alliance() != Alliance.Blue);
            switch (auto) {
                case SHAQ:
                    command = red ? new SSGShaqGOATRed(false) : new SSGShaqGOATBlue(false);
                    break;
                case SHAQ_BALANCE:
                    command = red ? new SSGShaqGOATRed(true) : new SSGShaqGOATBlue(true);
                    break;
                case KOB3:
                    command = red ? new SSGKob3Red(false) : new SSGKob3Blue(false);
                    break;
                case KOB3_BALANCE:
                    command = red ? new SSGKob3Red(true) : new SSGKob3Blue(true);
                    break;
                // case STRONG_SIDE_3_PC:
                // command = red ? null : null;
                // break;
                // case STRONG_SIDE_2_9:
                // command = red ? null : null;
                // break;
                // case STRONG_SIDE_2_5:
                // command = red ? null : null;
                // break;
                // case WEAK_SIDE_VANILLA:
                // command = red ? null : null;
                // break;
                // case CENTER:
                // command = red ? null : null;
                // break;
            }
            generatedAuto = auto;
            SmartDashboard.putString("auto cmd", auto.name() + " generated");
            // System.out.printf("generated auto cmd: %s\n", auto.name());
            System.gc(); // encourages garbage collector to run
        }
    }

    public boolean generated() {
        return command != null;
    }

    public CommandBase getAutoCommand() {
        return command;
    }

    public Color8Bit generatedColor() {
        return generatedAuto.color;
    }

    public Color8Bit color() {
        return auto.color;
    }

    public String name() {
        return auto == null ? "none" : auto.name();
    }

    public int getIndex() {
        return index;
    }
}
