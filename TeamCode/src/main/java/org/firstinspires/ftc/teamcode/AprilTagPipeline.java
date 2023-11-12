package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagPipeline  {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private String cameraName = "";
    private boolean visionEnabled = true;

    public AprilTagPipeline(HardwareMap h, String camName) {
        cameraName = camName;
        buildVisionPortal(h);

    }

    public void buildVisionPortal(HardwareMap h) {

        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(h.get(WebcamName.class, cameraName));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, visionEnabled);
    }



    //Use this method to run the pipeline and get detetcions
    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    public void toggleVisionPortal() {
        //Enabled by default on init
        visionEnabled = !visionEnabled;
        visionPortal.setProcessorEnabled(aprilTag, visionEnabled);
    }
    //Use for Telemetry to get status of vision
    public boolean isVisionEnabled() {
        return  visionEnabled;
    }
    public void closeVisionPortal() {
        visionPortal.close();
    }

}
