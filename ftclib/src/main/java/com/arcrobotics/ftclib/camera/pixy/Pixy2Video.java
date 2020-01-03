package com.arcrobotics.ftclib.camera.pixy;

import java.util.concurrent.TimeUnit;
public class Pixy2Video {
    public final static byte VIDEO_REQUEST_GET_RGB = 0x70;

    private final Pixy2 pixy;

    /**
     * Constructs Pixy2 video getter
     *
     * @param pixy Pixy2 instance
     */
    protected Pixy2Video(Pixy2 pixy) {
        this.pixy = pixy;
    }

    /**
     * Gets average RGB value at 5x5 area around specified coordinates in the image
     *
     * @param x        X value
     * @param y        Y value
     * @param rgb      RGB container to return values in
     * @param saturate Whether or not to scale all RGB values to maximize the
     *                 greatest value at 255
     *
     * @return Pixy2 error code
     */
    public int getRGB(int x, int y, RGB rgb, boolean saturate) {
        long start = System.currentTimeMillis();

        while (true) {
            pixy.bufferPayload[0] = (byte) (x & 0xff);
            pixy.bufferPayload[1] = (byte) ((x >> 8) & 0xff);
            pixy.bufferPayload[2] = (byte) (y & 0xff);
            pixy.bufferPayload[3] = (byte) ((y >> 8) & 0xff);
            pixy.bufferPayload[4] = (byte) (saturate == true ? 1 : 0);
            pixy.length = 5;
            pixy.type = VIDEO_REQUEST_GET_RGB;
            pixy.sendPacket();
            if (pixy.receivePacket() == 0) {
                if (pixy.type == Pixy2.PIXY_TYPE_RESPONSE_RESULT && pixy.length == 4) {
                    rgb.setRGB(pixy.buffer[0], pixy.buffer[1], pixy.buffer[2]);
                    return 0; // Success
                } else if (pixy.type == Pixy2.PIXY_TYPE_RESPONSE_ERROR
                        && pixy.buffer[0] == Pixy2.PIXY_RESULT_PROG_CHANGING) {
                    // Deal with program changing by waiting
                    try {
                        TimeUnit.MICROSECONDS.sleep(500);
                    } catch (InterruptedException e) {
                    }
                    continue;
                }
            }
            if (System.currentTimeMillis() - start > 500) {
                return Pixy2.PIXY_RESULT_ERROR; // Timeout to prevent lockup
            }
            return Pixy2.PIXY_RESULT_ERROR;
        }
    }

    public class RGB {

        int r, g, b;

        /**
         * Constructs RGB container
         *
         * @param r R value
         * @param g G value
         * @param b B value
         */
        public RGB(int r, int g, int b) {
            // Limits rgb values between the min and max
            this.r = (r >= 255 ? 255 : (r <= 0 ? 0 : r));
            this.g = (g >= 255 ? 255 : (g <= 0 ? 0 : g));
            this.b = (b >= 255 ? 255 : (b <= 0 ? 0 : b));
        }

        /**
         * @return Color object containing RGB
         */
        public int[] getColor() {
            return new int[] {r, g, b};
        }

        /**
         * @return R value
         */
        public byte getR() {
            return (byte) r;
        }

        /**
         * @return G value
         */
        public byte getG() {
            return (byte) g;
        }

        /**
         * @return B value
         */
        public byte getB() {
            return (byte) b;
        }

        /**
         * Sets R value between 0-255
         *
         * @param r R value
         */
        public void setR(int r) {
            // Limits r value between the min and max
            this.r = (r >= 255 ? 255 : (r <= 0 ? 0 : r));
        }

        /**
         * Sets G value between 0-255
         *
         * @param g G value
         */
        public void setG(int g) {
            // Limits g value between the min and max
            this.g = (g >= 255 ? 255 : (g <= 0 ? 0 : g));
        }

        /**
         * Sets B value between 0-255
         *
         * @param b B value
         */
        public void setB(int b) {
            // Limits b value between the min and max
            this.b = (b >= 255 ? 255 : (b <= 0 ? 0 : b));
        }

        /**
         * Sets RGB value from Color
         *
         * @param color Color
         */
        public void setRGB(int[] color) {
            this.r = color[0];
            this.g = color[1];
            this.b = color[2];
        }

        /**
         * Sets RGB value
         *
         * @param rgb RGB value
         */
        public void setRGB(int rgb) {
            this.r = (rgb >> 16) & 0xff;
            this.g = (rgb >> 8) & 0xff;
            this.b = rgb & 0xff;
        }

        /**
         * Sets RGB values between 0-255
         *
         * @param r R value
         * @param g G value
         * @param b B value
         */
        public void setRGB(int r, int g, int b) {
            // Limits r value between the min and max
            this.r = (r >= 255 ? 255 : (r <= 0 ? 0 : r));
            this.g = (g >= 255 ? 255 : (g <= 0 ? 0 : g));
            this.b = (b >= 255 ? 255 : (b <= 0 ? 0 : b));
        }

    }

}
