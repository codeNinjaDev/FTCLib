package com.arcrobotics.ftclib.camera.pixy;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.Arrays;

public class Pixy2Lego {
    private final static int PIXY_I2C_DEFAULT_ADDR = 0x54;
    private final static int PIXY_I2C_MAX_SEND = 16; // don't send any more than 16 bytes at a time

    private I2cDeviceSynch i2c = null;

    public Pixy2Lego(HardwareMap hw, String deviceName) {
        open(hw, deviceName);
    }

    public Pixy2Lego(HardwareMap hw, String deviceName, int address) {
        open(hw, deviceName, address);
    }
    /**
     * Opens I2C port
     *
     *
     * @return Returns 0
     */
    private int open(HardwareMap hw, String deviceName, int address) {
        i2c = hw.i2cDeviceSynch.get(deviceName);
        i2c.setI2cAddress(I2cAddr.create7bit(address));
        i2c.engage();
        return 0;
    }

    private int open(HardwareMap hw, String deviceName) {
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
     *
     * @return Length of value read
     */
    public Block getLargestBlock(int sig) {
        int address = 0x50 + sig;
        byte[] datai2c = i2c.read(address, 5);
        int x = (int) datai2c[1];
        int y = (int) datai2c[2];
        int width = (int) datai2c[3];
        int height = (int) datai2c[4];
        return new Block(sig, x, y, width, height);
    }


    public class Block {

        private int signature, x, y, width, height, area = 0;

        /**
         * Constructs signature block instance
         *
         * @param signature Block signature
         * @param x         X value
         * @param y         Y value
         * @param width     Block width
         * @param height    Block height

         */
        public Block(int signature, int x, int y, int width, int height) {
            this.signature = signature;
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.area = width * height;
        }

        /**
         * Prints signature block data to console
         */
        public void print() {
            System.out.println(toString());
        }

        /**
         * Returns a string of signature block data
         *
         * @return String of signature block data
         */
        public String toString() {
            int i, j;
            int[] sig = new int[6];
            int d;
            boolean flag;
            String out = "";
            if (signature > 7) {
                // Color code! (CC)
                // Convert signature number to an octal string
                for (i = 12, j = 0, flag = false; i >= 0; i -= 3) {
                    d = (signature >> i) & 0x07;
                    if (d > 0 && !flag)
                        flag = true;
                    if (flag)
                        sig[j++] = d + '0';
                }
                sig[j] = '\0';
                out = "CC block sig: " + Arrays.toString(sig) + " (" + signature + " decimal) x: " + x + " y: " + y
                        + " width: " + width + " height: " + height;

            } else // Regular block. Note, angle is always zero, so no need to print
                out = "sig: " + signature + " x: " + x + " y: " + y + " width: " + width + " height: " + height;
            return out;
        }

        /**
         * @return Block signature
         */
        public int getSignature() {
            return signature;
        }

        /**
         * @return Block X value
         */
        public int getX() {
            return x;
        }

        /**
         * @return Block Y value
         */
        public int getY() {
            return y;
        }

        /**
         * @return Block width
         */
        public int getWidth() {
            return width;
        }

        /**
         * @return Block height
         */
        public int getHeight() {
            return height;
        }

        public int getArea() {
            return area;
        }

    }
}
