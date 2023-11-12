package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class Sensors {
    private BNO055IMU gyro = null;
    private Orientation angles = null;

    /*
    private RevColorSensorV3 sideColor = null;
    private RevColorSensorV3  backColor = null;

    private Rev2mDistanceSensor sideWall  = null;
    private Rev2mDistanceSensor backWall  = null;
*/
    public Sensors(HardwareMap h) {
        gyro = h.get(BNO055IMU.class, "gyro");
        /*
        backColor = h.get(RevColorSensorV3.class, "backColor");
        sideColor = h.get(RevColorSensorV3.class, "sideColor");
        sideWall = h.get(Rev2mDistanceSensor.class,"sideWall");
        backWall = h.get(Rev2mDistanceSensor.class,"backWall");
*/
    }

    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = false;
        //parameters.loggingTag          = "Gyro";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        try {
            gyro.initialize(parameters);
        } catch(Exception e){

        }

    }

    public Orientation readGyroAngle() {
        angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles;
    }

    //X and Y are not needed
    public int getGyroZ(Orientation angles) {
        return (int)angles.firstAngle;
    }

    private String[] printGyroAngles(Orientation angles) {
        String a[] = {formatDegrees(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)), formatDegrees(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle)), formatDegrees(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle))};
        return a;
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
/*

    public int[] getSideGroundColors() {
        int colors[] = {sideColor.red(), sideColor.green(), sideColor.blue()};
        return colors;
    }

    public int getSideGroundRed() {
        return sideColor.red();
    }

    public int getSideGroundGreen() {
        return sideColor.green();
    }

    public int getSideGroundBlue() {
        return sideColor.blue();

    }
    public int[] getBackGroundColors() {
        int colors[] = {backColor.red(), backColor.green(), backColor.blue()};
        return colors;
    }

    public int getBackGroundRed() {
        return backColor.red();
    }

    public int getBackGroundGreen() {
        return backColor.green();
    }

    public int getBackGroundBlue() {
        return backColor.blue();

    }

    public double getSideDistance(){
        return sideWall.getDistance(DistanceUnit.INCH);
    }

    public double getBackDistance() { return backWall.getDistance(DistanceUnit.INCH); }


 */
}
