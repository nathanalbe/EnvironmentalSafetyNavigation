package org.firstinspires.ftc.teamcode;

import org.opencv.imgproc.Imgproc;

public enum ColorSpace {
    /*
     * Define our "conversion codes" in the enum
     * so that we don't have to do a switch
     * statement in the processFrame method.
     */
    RGB(Imgproc.COLOR_RGBA2RGB),
    HSV(Imgproc.COLOR_RGB2HSV),
    YCrCb(Imgproc.COLOR_RGB2YCrCb),
    Lab(Imgproc.COLOR_RGB2Lab);

    //store cvtCode in a public var
    public int cvtCode = 0;

    //constructor to be used by enum declarations above
    ColorSpace(int cvtCode) {
        this.cvtCode = cvtCode;
    }
}
