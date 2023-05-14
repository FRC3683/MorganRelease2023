package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utils.GlobalState.ScoringLevel;

public class DaveLED {
    private AddressableLED mLED;
    private AddressableLEDBuffer mLEDBuffer;
    private int mRainbowFirstPixelHue;

    public final Color8Bit clr_off = new Color8Bit(0, 0, 0);
    public final Color8Bit clr_err_0 = new Color8Bit(255, 40, 0);
    public final Color8Bit clr_err_1 = new Color8Bit(0, 255, 255);

    public final int length;

    public DaveLED(int port, int length) {
        this.length = length;
        mLEDBuffer = new AddressableLEDBuffer(length);
        if (RobotBase.isReal()) {
            mLED = new AddressableLED(port);
            mLED.setLength(mLEDBuffer.getLength());
            mLED.setData(mLEDBuffer);
            mLED.start();
            mRainbowFirstPixelHue = 0;
        }
    }

    public void drawLEDs() {
        if (RobotBase.isReal())
            mLED.setData(mLEDBuffer);
    }

    public void setColor(int start, int end, Color8Bit color) {
        for (var i = Math.max(0, start); i < Math.min(length, end); ++i) {
            mLEDBuffer.setRGB(i, color.red, color.green, color.blue);
        }
    }

    public void wipe() {
        setColor(0, length, clr_off);
    }

    public void setChecker(int start, int end, Color8Bit c0, Color8Bit c1, boolean alternate) {
        for (var i = Math.max(0, start); i < Math.min(length, end); ++i) {
            if ((i % 2 == 0) ^ alternate) {
                mLEDBuffer.setRGB(i, c0.red, c0.green, c0.blue);
            } else {
                mLEDBuffer.setRGB(i, c1.red, c1.green, c1.blue);
            }
        }
    }

    public void setCheckerErr(int start, int end, boolean alternate) {
        setChecker(start, end, clr_err_0, clr_err_1, alternate);
    }

    public void setColor(double percent, Color8Bit c) {
        percent = MathUtils.clamp(0, 1, percent);
        int end = (int) Math.round((length - 1) * percent);
        setColor(0, end, c);
    }

    public void setSingle(int index, Color8Bit c) {
        if (index < 0 || index >= length)
            return;
        mLEDBuffer.setRGB(index, c.red, c.green, c.blue);
    }

    public Color8Bit getSingle(int index) {
        return new Color8Bit(mLEDBuffer.getLED(MathUtils.clampi(0, length - 1, index)));
    }

    public static Color8Bit add(Color8Bit c0, Color8Bit c1) {
        return new Color8Bit(c0.red + c1.red, c0.green + c1.green, c0.blue + c1.blue);
    }

    public static Color8Bit mult(Color8Bit c, double v) {
        return new Color8Bit((int) Math.round(c.red * v), (int) Math.round(c.green * v), (int) Math.round(c.blue * v));
    }

    public void setTrail(int index, Color8Bit color) {
        setSingle(index - 2, add(getSingle(index - 2), mult(color, 0.01)));
        setSingle(index - 1, add(getSingle(index - 1), mult(color, 0.1)));
        setSingle(index, add(getSingle(index), color));
        setSingle(index + 1, add(getSingle(index + 1), mult(color, 0.1)));
        setSingle(index + 2, add(getSingle(index + 2), mult(color, 0.01)));
    }

    public void setColor(Color8Bit c) {
        setColor(0, length, c);
    }

    public void setColorScoringLevel(ScoringLevel scoringLevel, Color8Bit c) {
        double[] ends = { 0.5, 0.75, 1 };
        setColor(ends[scoringLevel.index], c);
    }

    public void rainbow() {
        for (var i = 0; i < length; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (mRainbowFirstPixelHue + (i * 180 / length)) % 180;
            // Set the value
            mLEDBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        mRainbowFirstPixelHue += 3;
        // Check bounds
        mRainbowFirstPixelHue %= 180;
    }

}