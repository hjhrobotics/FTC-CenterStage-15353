package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCvColorPipeline extends OpenCvPipeline {

    private String MarkerLocation = "";
    private String allianceColor = "";

    private static final int imageWidth = 320;
    private static final int imageHeight = 240;
    Mat region1, region2, region3;

    private static final int REGION_WIDTH = imageWidth/3;
    private static final int REGION_HEIGHT = imageHeight;

    private static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,0);
    private static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(REGION_WIDTH,0);
    private static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(2*REGION_WIDTH,0);

    Point region1_pointA, region1_pointB, region2_pointA, region2_pointB, region3_pointA, region3_pointB;

    /*
     * Some color constants
     */
    static final Scalar submatOutline = new Scalar(242, 250, 172); //Light Yellow Color
    static final Scalar GREEN = new Scalar(0, 255, 0);
    Mat recoloredImage = new Mat();
    Mat singleColorChannel = new Mat();
    int avg1, avg2, avg3;

    public OpenCvColorPipeline(String color, HardwareMap h) {
        allianceColor = color;

        region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


    }




    @Override
    public Mat processFrame(Mat input) {
        //Analyze the image--Divide Screen in 3 parts, draw a rect around each submat area,
        //Take the average of each, then find the area with the most color (Where the marker is)

        //Step 1, recolor image to YCrCb
        Imgproc.cvtColor(input, recoloredImage, Imgproc.COLOR_RGB2YCrCb);

        //Step 2: Extract the color channel based on Alliance
        if(allianceColor == "RED") {
            //Extract the red channel
            Core.extractChannel(recoloredImage, singleColorChannel, 1);

        } else {
            //Extract the blue channel
            Core.extractChannel(recoloredImage, singleColorChannel, 2);
        }

        //Divide recolored image into 3 subsections
        region1 = singleColorChannel.submat(new Rect(region1_pointA, region1_pointB));
        region2 = singleColorChannel.submat(new Rect(region2_pointA, region2_pointB));
        region3 = singleColorChannel.submat(new Rect(region3_pointA, region3_pointB));

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avg1 = (int) Core.mean(region1).val[0];
        avg2 = (int) Core.mean(region2).val[0];
        avg3 = (int) Core.mean(region3).val[0];

        /*
         * Draw a rectangle showing region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         * Light yellow outlines
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                submatOutline, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                submatOutline, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                submatOutline, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        /*
         * Find the max of the 3 averages
         */
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == avg1) // Was it from region 1?
        {
            MarkerLocation = "LEFT"; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg2) // Was it from region 2?
        {
            MarkerLocation = "CENTER"; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg3) // Was it from region 3?
        {
            MarkerLocation = "RIGHT"; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */

        return input;

    }

    public String getMarkerLocation() {
        return MarkerLocation;
    }
    public String getExpectedAlliance() {
        return allianceColor;
    }


}
