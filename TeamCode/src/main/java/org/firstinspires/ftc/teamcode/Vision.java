package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Vision {
    //private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //AprilTags
    private String aprilTagCamera = "Webcam1";
    private AprilTagPipeline atPipeline;
    private  AprilTagProcessor atProcessor;

    //OpenCV
    private OpenCvWebcam cvWebcam;
    private String cvCameraName = "Webcam1";
    private OpenCvColorPipeline cvPipeline;
    private String markerLocation = "";

    public Vision() {

    }

    public void cteateAprilTagPipeline(HardwareMap h) {
        atPipeline = new AprilTagPipeline(h, aprilTagCamera);
        atProcessor = atPipeline.getAprilTagProcessor();
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return atProcessor;
    }
    public void toggleAprilTagProcessing() {
        atPipeline.toggleVisionPortal();
    }

    public void closeAprilTagPipeline() {
        atPipeline.closeVisionPortal();
    }

    public void createOpenCvPipeline(HardwareMap h, String allianceColor) {
        cvPipeline = new OpenCvColorPipeline(allianceColor, h);
        int cameraMonitorViewId = h.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", h.appContext.getPackageName());
        cvWebcam = OpenCvCameraFactory.getInstance().createWebcam(h.get(WebcamName.class, cvCameraName), cameraMonitorViewId);
        cvWebcam.setPipeline(cvPipeline);
    }

    public void startCvStream () {
        cvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                cvWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }
    public  void stopCvStream() {
        cvWebcam.stopStreaming();

    }
    public void closeCvCamera() {
        cvWebcam.closeCameraDevice();
    }

    public String getCvMarkerLocation() {
        markerLocation = cvPipeline.getMarkerLocation();
        return markerLocation;
    }

    public String getExpectedAlliance() {
        return cvPipeline.getExpectedAlliance();
    }






}
