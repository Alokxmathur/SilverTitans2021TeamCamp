package org.firstinspires.ftc.teamcode.robot.components.camera;

import android.graphics.Bitmap;

import org.firstinspires.ftc.teamcode.game.Match;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class RingDetector {
    double ONE_DISK_HEIGHT = 20;
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(12.5,25,25,0);
    MatOfPoint largestContour = null;
    double contourArea = 0;
    int minY = 9999, minX = 9999;
    int maxY = 0, maxX = 0;

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();
    Mat inputMat = new Mat();

    public RingDetector() {}

    public RingDetector(Scalar ringColor) {
        setHsvColor(ringColor);
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 255) ? hsvColor.val[0]+mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

        mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;
    }

    public void process(Bitmap bitMap) {
        Utils.bitmapToMat(bitMap, inputMat);
        Imgproc.pyrDown(inputMat, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        List<MatOfPoint> contoursFound = new ArrayList<>();
        Imgproc.findContours(mDilatedMask, contoursFound, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        minX = minY = 9999;
        maxX = maxY = 0;
        contourArea = 0;
        for (MatOfPoint contour: contoursFound) {
            double area = Imgproc.contourArea(contour);
            if (area > this.contourArea) {
                this.contourArea = area;
                largestContour = contour;
                Core.multiply(contour, new Scalar(4,4), contour);
                Rect boundingRectangle = Imgproc.boundingRect(contour);
                minX = boundingRectangle.x;
                maxX = boundingRectangle.x + boundingRectangle.width;
                minY = boundingRectangle.y;
                maxY = boundingRectangle.y + boundingRectangle.height;
            }
        }
    }

    /**
     * Returns the number of rings seen on the launch stack
     * The determination is based upon the height of the stack seen.
     * The team uses our ColorBlobDetection app to determine the thresholds for the
     * hue, saturation and value readings for seeing the orange disks
     * @param bitMap: the bitmap of the camera image
     * @return number of rings seen
     */
    public int getNumberOfRings(Bitmap bitMap) {
        Match.log("Rings height=" + getHeight() + ", area: " + contourArea);
        process(bitMap);
        if (contourArea == 0) {
            return 0;
        }
        else if (getHeight() > ONE_DISK_HEIGHT) {
            return 1;
        }
        else {
            return 4;
        }
    }

    public double getHeight() {
        return maxY - minY;
    }
    public double getWidth() {
        return maxX - minX;
    }
}