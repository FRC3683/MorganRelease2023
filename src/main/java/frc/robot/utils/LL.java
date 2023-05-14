package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Constants;
import frc.robot.utils.GlobalState.GamePiece;
import frc.robot.utils.GlobalState.ScoringLevel;

public class LL {
    // Config
    static {
        BOTTOM = new LL("limelight-bottom", "10.36.83.83");
        TOP = new LL("limelight-top", "10.36.83.36");
        SIZE_BOTPOSE = 6;
    }

    public static final LL BOTTOM;
    public static final LL TOP;

    public static final int SIZE_BOTPOSE;

    public final NetworkTable nt;
    public final String ntstring;
    public final String ip;

    public final DoubleSubscriber tvSub;
    public final DoubleSubscriber txSub;
    public final DoubleSubscriber tySub;
    public final DoubleSubscriber taSub;
    public final DoubleSubscriber tsSub;
    public final DoubleArraySubscriber bpRedSub;
    public final DoubleArraySubscriber bpBluSub;

    private boolean isOn;

    public static enum Setting {
        LEDMODE("ledMode"),
        CAMMODE("camMode"),
        PIPELINE("pipeline"),
        STREAM("stream"),
        SNAPSHOT("snapshot"),;

        public String nt;

        Setting(String nt) {
            this.nt = nt;
        }
    }

    private LL(String nt, String ip) {
        this.ntstring = nt;
        this.ip = ip;
        this.nt = NetworkTableInstance.getDefault().getTable(ntstring);
        tvSub = doubleSub("tv");
        txSub = doubleSub("tx");
        tySub = doubleSub("ty");
        taSub = doubleSub("ta");
        tsSub = doubleSub("ts");
        bpRedSub = doubleArraySub("botpose_wpired", 6);
        bpBluSub = doubleArraySub("botpose_wpiblue", 6);

        LEDoff();
    }

    private static double FILLER = 3683.0;

    private DoubleSubscriber doubleSub(String ntt) {
        return nt.getDoubleTopic(ntt)
                .subscribe(FILLER, PubSubOption.keepDuplicates(true),
                        PubSubOption.periodic(Constants.controlDt_s),
                        PubSubOption.sendAll(true));
    }

    private DoubleArraySubscriber doubleArraySub(String ntt, int size) {
        double[] temp = new double[size];
        for (int i = 0; i < size; i++)
            temp[i] = FILLER;
        return nt.getDoubleArrayTopic(ntt)
                .subscribe(temp, PubSubOption.keepDuplicates(true),
                        PubSubOption.periodic(Constants.controlDt_s),
                        PubSubOption.sendAll(true));
    }

    static LinearFilter[] poseFilters = new LinearFilter[] {
            LinearFilter.movingAverage(5),
            LinearFilter.movingAverage(5),
            LinearFilter.movingAverage(5),
            LinearFilter.movingAverage(5),
            LinearFilter.movingAverage(5),
            LinearFilter.movingAverage(5)
    };

    public static int pipeline(GamePiece gamePiece, ScoringLevel scoringLevel) {
        int pipelines[][] = {
                { 1, 1, 0 }, // cone
                { 2, 2, 2 } // cube
        };
        return pipelines[gamePiece.index][scoringLevel.index];
    }

    Debouncer commsDebouncer = new Debouncer(0.5);
    double prevLatency = 0;

    public boolean comms() {
        double latency = nt.getEntry("tl").getDouble(0);
        if (commsDebouncer.calculate(latency == prevLatency)) {
            return false;
        }
        prevLatency = latency;
        return true;
    }

    public boolean validTarget() {
        return tvSub.get() > 0.0;
    }

    public long tv_timestamped() {
        return tvSub.getAtomic().timestamp;
    }

    public double tx() {
        return txSub.get();
    }

    public TimestampedDouble tx_timestamped() {
        return txSub.getAtomic();
    }

    public double ty() {
        return tySub.get();
    }

    public TimestampedDouble ty_timestamped() {
        return tySub.getAtomic();
    }

    public double ta() {
        return taSub.get();
    }

    public TimestampedDouble ta_timestamped() {
        return taSub.getAtomic();
    }

    public double ts() {
        return tsSub.get();
    }

    public TimestampedDouble ts_timestamped() {
        return tsSub.getAtomic();
    }

    public double[] botposeRed() {
        return bpRedSub.get();
    }

    public static Pose2d arrayToPose2d(double[] bp) {
        return new Pose2d(bp[0], bp[1], Rotation2d.fromDegrees(bp[5]));
    }

    public Pose2d botposeRedPose2d() {
        return arrayToPose2d(botposeRed());
    }

    public TimestampedDoubleArray botposeRed_timestamped() {
        return bpRedSub.getAtomic();
    }

    public double[] botposeBlub() {
        return bpBluSub.get();
    }

    public Pose2d botposeBlubPose2d() {
        return arrayToPose2d(botposeBlub());
    }

    public TimestampedDoubleArray botposeBlu_timestamped() {
        return bpBluSub.getAtomic();
    }

    public static long delta_us(long t) {
        return Math.abs(RobotController.getFPGATime() - t);
    }

    public static double delta_s(long t) {
        return (RobotController.getFPGATime() - t) / 1000000.0;
    }

    public static void portforward() {
        PortForwarder.add(5800, TOP.ip, 5800);
        PortForwarder.add(5801, TOP.ip, 5801);
        PortForwarder.add(5802, TOP.ip, 5802);
        PortForwarder.add(5803, TOP.ip, 5803);
        PortForwarder.add(5804, TOP.ip, 5804);
        PortForwarder.add(5805, TOP.ip, 5805);
        PortForwarder.add(5806, BOTTOM.ip, 5800);
        PortForwarder.add(5807, BOTTOM.ip, 5801);
        PortForwarder.add(5808, BOTTOM.ip, 5802);
        PortForwarder.add(5809, BOTTOM.ip, 5803);
        PortForwarder.add(5810, BOTTOM.ip, 5804);
        PortForwarder.add(5811, BOTTOM.ip, 5805);
    }

    public void setPipeline(int pipeline) {
        sendDouble(pipeline, Setting.PIPELINE);
    }

    public void LEDpipeline() {
        sendDouble(0, Setting.LEDMODE);
    }

    public void LEDoff() {
        sendDouble(1, Setting.LEDMODE);
        isOn = false;
    }

    public void LEDblink() {
        sendDouble(2, Setting.LEDMODE);
    }

    public void LEDon() {
        sendDouble(3, Setting.LEDMODE);
        isOn = true;
    }

    public void CAMvision() {
        sendDouble(0, Setting.CAMMODE);
    }

    public void CAMdriver() {
        sendDouble(1, Setting.CAMMODE);
    }

    public void STREAMstandard() {
        sendDouble(0, Setting.STREAM);
    }

    public void STREAMprimary() {
        sendDouble(1, Setting.STREAM);
    }

    public void STREAMsecondary() {
        sendDouble(2, Setting.STREAM);
    }

    public void snapshot() {
        sendDouble(1, Setting.SNAPSHOT);
    }

    public void resetSnapshot() {
        sendDouble(0, Setting.SNAPSHOT);
    }

    public void sendDouble(double out, Setting nto) {
        nt.getEntry(nto.nt).setDouble(out);
    }

    public void sendInt(int out, Setting nto) {
        nt.getEntry(nto.nt).setInteger(out);
    }

    public boolean isOn() {
        return isOn;
    }
}
