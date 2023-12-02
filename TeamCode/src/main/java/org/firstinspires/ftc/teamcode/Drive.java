package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {
    private DcMotor lf_drive = null;
    private DcMotor lr_drive  = null;
    private DcMotor rf_drive  = null;
    private DcMotor rr_drive  = null;

    public Drive(HardwareMap h) {

        lf_drive = h.get(DcMotor.class, "lf_drive");
        lr_drive = h.get(DcMotor.class, "lr_drive");
        rf_drive = h.get(DcMotor.class, "rf_drive");
        rr_drive = h.get(DcMotor.class, "rr_drive");

        lf_drive.setDirection(DcMotor.Direction.REVERSE);
        lr_drive.setDirection(DcMotor.Direction.REVERSE);
        rf_drive.setDirection(DcMotor.Direction.FORWARD);
        rr_drive.setDirection(DcMotor.Direction.FORWARD);

        //Disable the encoders
        lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        runUsingEncoders();


    }



    //Encoders--Use the rear encoders only
    public void runUsingEncoders() {
       rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public  void disableEncoders() {
        lf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Left Motor Encoder Stuff
    public void leftEncoderToPosition(int position) { lr_drive.setTargetPosition(lr_drive.getCurrentPosition() + position); }
    public void leftMotorRunWithEncoders() { lr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public boolean leftMotorBusy() {
        return lr_drive.isBusy();
    }
    public String getLeftMotorMode() {
        return lr_drive.getMode().toString();
    }
    public void resetLeftEncoder() {
        lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public int getLeftEncoderValue() {
        return  lr_drive.getCurrentPosition();
    }

    //Right Encoder Stuff
    public void rightEncoderToPosition(int position) { rr_drive.setTargetPosition(rr_drive.getCurrentPosition() + position); }
    public void rightMotorRunWithEncoders() { rr_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public boolean rightMotorBusy() {
        return rr_drive.isBusy();
    }
    public String getRightMotorMode() {
        return rr_drive.getMode().toString();
    }
    public void resetRightEncoder() {
        rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public int getRightEncoderValue() {
        return  rr_drive.getCurrentPosition();
    }

    //Drive Methods
    public void straightDrive(double speed) {
        lf_drive.setPower(speed);
        lr_drive.setPower(speed);
        rf_drive.setPower(speed);
        rr_drive.setPower(speed);
    }

    public void strafe(double speed) {
        lf_drive.setPower(speed);
        lr_drive.setPower(-speed);
        rf_drive.setPower(-speed);
        rr_drive.setPower(speed);
    }

    public void teleopDrive(double x, double y) {
        double leftPower;
        double rightPower = x;
        leftPower    = Range.clip(x + y, -1.0, 1.0) ;
        rightPower    = Range.clip(x - y, -1.0, 1.0) ;

        lf_drive.setPower(leftPower);
        lr_drive.setPower(leftPower);
        rf_drive.setPower(rightPower);
        rr_drive.setPower(rightPower);
    }

    public  void tankDrive(double left, double right) {
        double leftPower;
        double rightPower;
        leftPower    = Range.clip(left, -1.0, 1.0) ;
        rightPower    = Range.clip(right, -1.0, 1.0) ;

        lf_drive.setPower(leftPower);
        lr_drive.setPower(leftPower);
        rf_drive.setPower(rightPower);
        rr_drive.setPower(rightPower);
    }

    public void printDriveTelemetry(Telemetry t) {
        t.addData("LR Position", lr_drive.getCurrentPosition());
        t.addData("LR Mode", lr_drive.getMode().toString());

        t.addData("RR Position", rr_drive.getCurrentPosition());
        t.addData("RR Mode", rr_drive.getMode().toString());

    }

}
