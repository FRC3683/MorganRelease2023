package frc.robot.utils;

import edu.wpi.first.wpilibj.I2C;

public class AdafruitI2C {

    I2C imu;
    private byte TCS34725_COMMAND_BIT = (byte)0x80;
    private Device device;
    public enum Device{
        BNO055,
        TCS34725
    }

    public AdafruitI2C(I2C.Port port, byte address, Device device){
        imu = new I2C(port, address);
        this.device = device;
    }

    /**
     * Writes an 8 bit value over I2C
     * @param reg the register to write the data to
     * @param value a byte of data to write
     * @return whatever I2CJNI.i2CWrite returns. It's not documented in the wpilib javadocs!
     */
    boolean write8(int reg, int value) {
        return write8(reg, (byte) value);
    }

    /**
     * Writes an 8 bit value over I2C
     * @param reg the register to write the data to
     * @param value a byte of data to write
     * @return whatever I2CJNI.i2CWrite returns. It's not documented in the wpilib javadocs!
     */
    boolean write8(int reg, byte value) {
        boolean retVal = false;

        if(device == Device.TCS34725) {
            retVal = imu.write(reg | TCS34725_COMMAND_BIT, value & 0xFF);
        }
        else{
            retVal = imu.write(reg, value);
        }

        return retVal;
    }

    /**
     * Reads an 8 bit value over I2C via BNO
     * @param reg the register to read from.
     * @return
     */
    byte read8(BNO055.reg_t reg) {
        return read8(reg.getVal());
    }

    /**
     * Reads an 8 bit value over I2C
     * @param reg the register to read from.
     * @return
     */
    byte read8(int reg) {
        byte[] vals = new byte[1];
        readLen(reg, vals);
        return vals[0];
    }

    /**
     * Reads a 16 bit value over I2C
     * @param reg the register to read from.
     * @return
     */

    short read16(int reg) {
        byte[] vals = new byte[2];
        readLen(reg, vals);
        return (short) (vals[0]<<8 | vals[1]);
    }

    /**
     * Reads the specified number of bytes over I2C
     *
     * @param reg the address to read from
     * @param buffer the size of the data to read
     * @return true on success
     */
    boolean readLen(int reg, byte[] buffer) {
        boolean retVal = true;

        if (buffer == null || buffer.length < 1) {
            return false;
        }

        if(device == Device.TCS34725) {
            retVal = !imu.read(reg | TCS34725_COMMAND_BIT, buffer.length, buffer);
        }
        else {
            retVal = !imu.read(reg, buffer.length, buffer);
        }

        return retVal;
    }
}