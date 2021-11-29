package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Size;

public class BarcodeDetection extends Detection{
    public BarcodeDetection(Size frameSize, double minAreaFactor) {
        super(frameSize, minAreaFactor);
    }

    public BarcodeDetection(Size frameSize, double minAreaFactor, double maxAreaFactor) {
        super(frameSize, minAreaFactor, maxAreaFactor);
    }

    public Barcode getBarcode() {
        double totalWidth = this.maxSizePx.width;
        double center = this.getCenter().x;
        double windowWidth = totalWidth / 3;

        if (center <= windowWidth) {
            return Barcode.LEFT;
        } else if (center <= windowWidth * 2) {
            return Barcode.CENTER;
        } else if (center <= windowWidth * 3) {
            return Barcode.RIGHT;
        }

        return Barcode.UNKNOWN;
    }

    public enum Barcode {
        UNKNOWN, LEFT, CENTER, RIGHT
    }
}
