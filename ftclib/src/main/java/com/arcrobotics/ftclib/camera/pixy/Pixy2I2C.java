package com.arcrobotics.ftclib.camera.pixy;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceImpl;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.Arrays;

public class Pixy2I2C {
    private final static int PIXY_I2C_DEFAULT_ADDR = 0x54;
    private final static int PIXY_I2C_MAX_SEND = 16; // don't send any more than 16 bytes at a time

    private I2cDeviceSynch i2c = null;

    /**
     * Opens I2C port
     *
     *
     * @return Returns 0
     */
    public int open(HardwareMap hw, String deviceName, int address) {
        i2c = hw.i2cDeviceSynch.get(deviceName);
        i2c.setI2cAddress(I2cAddr.create7bit(address));
        i2c.engage();
        return 0;
    }

    public int open(HardwareMap hw, String deviceName) {
        i2c = hw.i2cDeviceSynch.get(deviceName);
        i2c.setI2cAddress(I2cAddr.create7bit(PIXY_I2C_DEFAULT_ADDR));
        i2c.engage();
        return 0;
    }

    /**
     * Closes I2C port
     */
    public void close() {
        i2c.close();
    }

    /**
     * Receives and reads specified length of bytes from I2C
     *
     * @param buffer Byte buffer to return value
     * @param length Length of value to read
     * @param cs     Checksum
     *
     * @return Length of value read
     */
    public int receive(byte[] buffer, int length, Pixy2.Checksum cs) {
        int i, n;
        if (cs != null)
            cs.reset();
        for (i = 0; i < length; i += n) {
            // n is the number read -- it most likely won't be equal to length
            n = 0;
            byte[] read = new byte[length - i];
            read = i2c.read(i2c.getI2cAddress().get7Bit(), (length - i));
            for (int k = 0; k < read.length; k++) {
                n++;
                byte b = read[k];
                if (cs != null) {
                    int csb = b & 0xff;
                    cs.updateChecksum(csb);
                }
                buffer[k + i] = b;
            }
        }
        return length;
    }

    /**
     * Receives and reads specified length of bytes from I2C
     *
     * @param buffer Byte buffer to return value
     * @param length Length of value to read
     *
     * @return Length of value read
     */
    public int receive(byte[] buffer, int length) {
        return receive(buffer, length, null);
    }

    /**
     * Writes and sends buffer over I2C
     *
     * @param buffer Byte buffer to send
     * @param length Length of value to send
     *
     * @return Length of value sent
     */
    public int send(byte[] buffer, int length) {
        int i, packet;
        for (i = 0; i < length; i += PIXY_I2C_MAX_SEND) {
            if (length - i < PIXY_I2C_MAX_SEND)
                packet = (length - i);
            else
                packet = PIXY_I2C_MAX_SEND;
            byte[] send = Arrays.copyOfRange(buffer, i, packet);
            i2c.write(i2c.getI2cAddress().get7Bit(), send);
        }
        return length;
    }
}
