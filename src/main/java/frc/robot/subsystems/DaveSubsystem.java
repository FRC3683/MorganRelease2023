// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.utils.GlobalTimer;
import frc.robot.utils.MathUtils;
import frc.robot.utils.StateLogger;

public abstract class DaveSubsystem extends SubsystemBase {

    public abstract class State {
        private String name;
        private boolean ranConstruct;
        private boolean isActive;
        private Timer timer;
        private DaveSubsystem subsystem;

        public State(String name) {
            this.subsystem = DaveSubsystem.this;
            this.name = name;
            ranConstruct = false;
            isActive = false;
            timer = new Timer();
        }

        public void construct() {
        }

        public abstract void init();

        public abstract void periodic();

        public void onComplete() {
        }

        public final void baseInit() {
            if (!ranConstruct) {
                construct();
                ranConstruct = true;
            }
            init();
            timer.reset();
            timer.start();
            isActive = true;
        }

        public final void basePeriodic() {
            periodic();
        }

        public double time() {
            if (isActive) {
                return timer.get();
            } else {
                return -1.0;
            }
        }

        public void reset() {
            if (isActive) {
                timer.reset();
            } else {
            }
        }

        public boolean after(double seconds) {
            if (isActive) {
                return timer.hasElapsed(seconds);
            } else {
                return false;
            }
        }

        public boolean before(double seconds) {
            if (isActive) {
                return !timer.hasElapsed(seconds);
            } else {
                return true;
            }
        }

        public boolean between(double earlier, double later) {
            if (isActive) {
                double t = timer.get();
                return earlier < t && t < later;
            } else {
                return false;
            }
        }

        public double alpha(double earlier, double later) {
            if (isActive) {
                double t = timer.get();
                if (t <= earlier)
                    return 0.0;
                if (t >= later)
                    return 1.0;
                return MathUtils.unlerp(earlier, later, t);
            } else {
                return 0.0;
            }
        }

        public String getName() {
            return name;
        }

        public boolean isActive() {
            return isActive;
        }

        public void deactivate() {
            onComplete();
            isActive = false;
            timer.stop();
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == null) {
                return false;
            }

            if (obj.getClass() != this.getClass()) {
                return false;
            }

            final State other = (State) obj;
            if ((this.name == null) ? (other.name != null) : !this.name.equals(other.name)) {
                return false;
            }

            return true;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    private State currentState;

    protected GlobalTimer gtimer;

    public State getCurrentState() {
        return currentState;
    }

    public void setCurrentState(State state) {
        if (state.subsystem != this) {
            System.err.printf("error, cannot set the state of <%s> to <%s>.<%s>\n", this.getName(),
                    state.subsystem.getName(), state.getName());
        }
        if (currentState == null) {
            currentState = state;
            currentState.baseInit();
        } else if (!currentState.equals(state)) {
            currentState.deactivate();
            currentState = state;
            currentState.baseInit();
        }
    }

    public boolean isState(String state) {
        return currentState.name.equals(state);
    }

    public boolean isState(State state) {
        return currentState.equals(state);
    }

    private ShuffleboardTab tab;

    interface WriteDashboardFunc {
        Object write();
    }

    private class DashboardEntryWrite {
        GenericEntry entry;
        WriteDashboardFunc func;

        private DashboardEntryWrite(String title, Object defaultValue, WriteDashboardFunc func) {
            entry = tab.add(title, defaultValue).getEntry();
            this.func = func;
        }

        void write() {
            entry.setValue(func.write());
        }
    }

    interface ReadDashboardFunc {
        void read(NetworkTableValue value);
    }

    class DashboardEntryRead {
        GenericEntry entry;
        ReadDashboardFunc func;

        DashboardEntryRead(String title, Object defaultValue, ReadDashboardFunc func) {
            entry = tab.add(title, defaultValue).getEntry();
            this.func = func;
        }

        void read() {
            func.read(entry.get());
        }
    }

    protected double asDouble(NetworkTableValue o) {
        return ((NetworkTableValue) o).getDouble();
    }

    private ArrayList<DashboardEntryWrite> entriesWriteable = new ArrayList<>();
    private ArrayList<DashboardEntryRead> entriesReadable = new ArrayList<>();

    public void AddDashboardEntryWrite(String title, Object defaultValue, WriteDashboardFunc func) {
        entriesWriteable.add(new DashboardEntryWrite(title, defaultValue, func));
    }

    public void AddDashboardEntryState(State defaultValue) {
        AddDashboardEntryWrite("State", defaultValue.getName(), () -> {
            return currentState.getName();
        });
    }

    public void AddDashboardEntryEncoderPosition(String title, Encoder encoder) {
        AddDashboardEntryWrite(title, 0, () -> {
            return encoder.getDistance();
        });
    }

    public void AddDashboardEntryEncoderRate(String title, Encoder encoder) {
        AddDashboardEntryWrite(title, 0, () -> {
            return encoder.getRate();
        });
    }

    public void AddDashboardEntryEncoderRate(String title, Encoder encoder, double scale) {
        AddDashboardEntryWrite(title, 0, () -> {
            return encoder.getRate() * scale;
        });
    }

    public void AddDashboardEntrySparkEncoderRate(String title, RelativeEncoder encoder) {
        AddDashboardEntryWrite(title, 0, () -> {
            return encoder.getVelocity();
        });
    }

    public void AddDashboardEntrySparkEncoderPosition(String title, RelativeEncoder encoder) {
        AddDashboardEntryWrite(title, 0, () -> {
            return encoder.getPosition();
        });
    }

    public void AddDashboardEntryRawSolenoid(String title, Solenoid solenoid) {
        AddDashboardEntryWrite(title, 0, () -> {
            return solenoid.get();
        });
    }

    public void AddDashboardEntryRead(String title, double defaultValue, ReadDashboardFunc func) {
        entriesReadable.add(new DashboardEntryRead(title, defaultValue, func));
    }

    private void sendToDashboard() {
        for (var entry : entriesWriteable) {
            entry.write();
        }
    }

    private void getFromDashboard() {
        for (var entry : entriesReadable) {
            entry.read();
        }
    }

    protected Config cfg;

    /** Creates a new ExampleSubsystem. */
    public DaveSubsystem(String name) {
        tab = Shuffleboard.getTab(name);
        gtimer = GlobalTimer.getInstance();
        cfg = Config.getInstance();
    }

    public abstract void subsystemPeriodic();

    @Override
    public final void periodic() {
        // This method will be called once per scheduler run
        getFromDashboard(); // TODO: Check performance
        // currentState.basePeriodic();
        // subsystemPeriodic();
        sendToDashboard();
    }

    public final void controlsPeriodic() {
        currentState.basePeriodic();
        subsystemPeriodic();
    }

    @Override
    public final void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
