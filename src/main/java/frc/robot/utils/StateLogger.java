// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.TimeZone;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.DaveSubsystem.State;

/** Add your docs here. */
public class StateLogger {

    private enum DataType {
        LONG(0),
        INT(1),
        BOOL(2),
        DOUBLE(3),
        STRING(4),
        STATE(5),
        POSE2D(6),
        EESTATE(7);

        public int id;

        private DataType(int id) {
            this.id = id;
        }
    }

    public static final Pose2d badPose = new Pose2d(-3683, -3683, new Rotation2d(-3683));

    private final ArrayList<String> keylist;
    private final HashMap<String, DataType> typemap;
    private final HashMap<String, IntSupplier> intmap;
    private final HashMap<String, BooleanSupplier> boolmap;
    private final HashMap<String, LongSupplier> longmap;
    private final HashMap<String, DoubleSupplier> doublemap;
    private final HashMap<String, Supplier<String>> stringmap;
    private final HashMap<String, Supplier<State>> statemap;
    private final HashMap<String, Supplier<Pose2d>> posemap;
    private final HashMap<String, Supplier<EEState>> estatemap;

    private StringBuilder sb;
    private BufferedWriter log_file;

    public StateLogger() {
        keylist = new ArrayList<>();
        typemap = new HashMap<>();
        longmap = new HashMap<>();
        doublemap = new HashMap<>();
        intmap = new HashMap<>();
        stringmap = new HashMap<>();
        statemap = new HashMap<>();
        posemap = new HashMap<>();
        boolmap = new HashMap<>();
        estatemap = new HashMap<>();

        sb = new StringBuilder();
    }

    private void addKeysAndType(String key, DataType type) {
        keylist.add(key);
        typemap.put(key, type);
    }

    public void log(String key, LongSupplier val) {
        if (typemap.containsKey(key)) {
            System.err.println(key + " already exists ya dummy");
            return;
        }
        addKeysAndType(key, DataType.LONG);
        longmap.put(key, val);
    }

    public void log(String key, DoubleSupplier val) {
        if (typemap.containsKey(key)) {
            System.err.println(key + " already exists ya dummy");
            return;
        }
        addKeysAndType(key, DataType.DOUBLE);
        doublemap.put(key, val);
    }

    public void log(String key, IntSupplier val) {
        if (typemap.containsKey(key)) {
            System.err.println(key + " already exists ya dummy");
            return;
        }
        addKeysAndType(key, DataType.INT);
        intmap.put(key, val);
    }

    public void log(String key, BooleanSupplier val) {
        if (typemap.containsKey(key)) {
            System.err.println(key + " already exists ya dummy");
            return;
        }
        addKeysAndType(key, DataType.BOOL);
        boolmap.put(key, val);
    }

    public void logString(String key, Supplier<String> val) {
        if (typemap.containsKey(key)) {
            System.err.println(key + " already exists ya dummy");
            return;
        }
        addKeysAndType(key, DataType.STRING);
        stringmap.put(key, val);
    }

    public void logState(String key, Supplier<State> val) {
        if (typemap.containsKey(key)) {
            System.err.println(key + " already exists ya dummy");
            return;
        }
        addKeysAndType(key, DataType.STATE);
        statemap.put(key, val);
    }

    public void logPose(String key, Supplier<Pose2d> val) {
        if (typemap.containsKey(key)) {
            System.err.println(key + " already exists ya dummy");
            return;
        }
        addKeysAndType(key, DataType.POSE2D);
        posemap.put(key, val);
    }

    public void logEEstate(String key, Supplier<EEState> val) {
        if (typemap.containsKey(key)) {
            System.err.println(key + " already exists ya dummy");
            return;
        }
        addKeysAndType(key, DataType.EESTATE);
        estatemap.put(key, val);
    }

    public void startLogging() {
        if (keylist.size() > 0) {
            for (String key : keylist) {
                sb.append(key + "~");
            }
            sb.append("\n");
            for (String key : keylist) {
                sb.append(typemap.get(key).id + "~");
            }
            sb.append("\n");
        }
    }

    public void append() {
        if (keylist.size() > 0) {

            for (String key : keylist) {
                DataType d = typemap.get(key);

                if (d == DataType.LONG) {
                    sb.append(longmap.get(key).getAsLong());
                } else if (d == DataType.BOOL) {
                    sb.append(boolmap.get(key).getAsBoolean() ? 1 : 0);
                } else if (d == DataType.INT) {
                    sb.append(intmap.get(key).getAsInt());
                } else if (d == DataType.DOUBLE) {
                    sb.append(doublemap.get(key).getAsDouble());
                } else if (d == DataType.STRING) {
                    sb.append(stringmap.get(key).get());
                } else if (d == DataType.STATE) {
                    State s = statemap.get(key).get();
                    sb.append(s.getName() + ":" + s.time());
                } else if (d == DataType.POSE2D) {
                    Pose2d p = posemap.get(key).get();
                    sb.append(p.getX());
                    sb.append(",");
                    sb.append(p.getY());
                    sb.append(",");
                    sb.append(p.getRotation().getDegrees());
                } else if (d == DataType.EESTATE) {
                    EEState p = estatemap.get(key).get();
                    sb.append(p.escPos_m);
                    sb.append(",");
                    sb.append(p.lnxPos_m);
                    sb.append(",");
                    sb.append(p.wristPos_deg);
                    sb.append(",");
                    sb.append(p.escVel_mps);
                    sb.append(",");
                    sb.append(p.lnxVel_mps);
                    sb.append(",");
                    sb.append(p.wristVel_degps);
                    sb.append(",");
                    sb.append(p.escAcc_mpsps);
                    sb.append(",");
                    sb.append(p.lnxAcc_mpsps);
                    sb.append(",");
                    sb.append(p.wristAcc_degpsps);
                }
                sb.append("~");
            }
            sb.append("\n");
        }
    }

    public void flush() {
        if (RobotBase.isReal()) {

            System.out.println("FLUSHING");
            String filedir = "/U/";
            String log_name = filedir + "log_" + DriverStation.getEventName() + "_"
                    + DriverStation.getMatchType() + "_"
                    + Integer.toString(DriverStation.getMatchNumber()) + "_"
                    + getDateTimeString() + ".tony";

            File dir = new File(filedir);
            if (!dir.exists())
                return;

            // Confirm file not already opened
            if (log_file != null) {
                System.out.println("Warning: attempt to open an already opened log!");
                return;
            }

            // Open File
            try {

                FileWriter fstream = new FileWriter(log_name, true);
                log_file = new BufferedWriter(fstream);

                log_file.write(sb.toString());
                log_file.flush();
                log_file.close();
            } catch (Exception e) {
                System.err.println("FILE WRITE FAIIIIIIIILED!!!!!! deforested :(");
                e.printStackTrace();
            } finally {

                log_file = null;
            }

            // System.out.println(sb.toString());
        }
    }

    public void clearStateLog() {
        sb = new StringBuilder();
    }

    private String getDateTimeString() {
        DateFormat df = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
        df.setTimeZone(TimeZone.getTimeZone("US/Central"));
        return df.format(new Date());
    }
}
