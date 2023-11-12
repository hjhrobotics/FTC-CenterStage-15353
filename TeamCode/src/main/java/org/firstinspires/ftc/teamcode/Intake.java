package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private Servo gripper = null;

    private Servo rotategripper = null;

    private boolean gripperOpen = false;

    private int liftLevel = 0;

    private DcMotor lifted = null;

    private DcMotor climber = null;

    private DcMotor liftclimber = null;

    private int liftUpperLimit = 4130;
    private int liftLowerLimit = 100;

    private int level0target = -25;

    private int level1target = -4000;

    private int level2target = -7750;

    private int level3target = -10500;

    private int auton = -6000;

    // private DigitalChannel armLimitSwitch = null;
    // private DigitalChannel armLowerLimit = null;


    public Intake(HardwareMap h) {

        gripper = h.get(Servo.class, "gripper");
        rotategripper = h.get(Servo.class, "rotategripper");

        lifted = h.get(DcMotor.class, "lifted");
        climber = h.get(DcMotor.class, "climber");
        liftclimber = h.get(DcMotor.class, "liftclimber");

        lifted.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftclimber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lifted.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftclimber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      //  gripper.setPosition(.5);
        //rotategripper.setPosition(.27);

    }

    public void removeLiftLimits() {
        lifted.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public String getMotorRunmode() {
        return lifted.getMode().toString();
    }

    public int getLiftLevel() {
        return liftLevel;
    }

    public void setLiftLevel(int level) {
        liftLevel = level;
    }


    //Open and close the gripper
    public void openGripper() {
        gripper.setPosition(.5);
        gripperOpen = true;
    }

    public void closeGripper() {
        gripper.setPosition(.75);
        gripperOpen = false;
    }

    public void gripperUp() {
        rotategripper.setPosition(0);
    }

    public void gripperDown() {
        rotategripper.setPosition(1);
    }


    public void toggleGripper() {
        if (gripperOpen) {
            closeGripper();
        } else {
            openGripper();
        }
        gripperOpen = !gripperOpen;
    }

    public void moveLift(double speed) {
        if (speed > .8 || speed < -.8) {
            lifted.setPower(speed);
        } else {
            lifted.setPower(0);
        }

    }

    public void lowerLift(double speed) {
        if (lifted.getCurrentPosition() > liftLowerLimit) {
            lifted.setPower(speed);
        } else {
            lifted.setPower(0);
        }


    }

    public void raiseLift(double speed) {
        if (lifted.getCurrentPosition() < liftUpperLimit) {
            lifted.setPower(speed);
        } else {
            lifted.setPower(0);
        }


    }

    public void stopLift() {
        lifted.setPower(0);
    }

    public void runClimber(double speed) {
        double s = Range.clip(speed, -1, 1);
        climber.setPower(s);
    }

    public void setLiftclimber(double speed) {
        liftclimber.setPower(speed * .5);
    }

    public double getEncodedLift(){
        return lifted.getCurrentPosition();
    }

    public void liftToBottom() {
        lifted.setTargetPosition(level0target);
        setLiftLevel(0);
        lifted.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void liftLevel1() {
        lifted.setTargetPosition(level1target);
        setLiftLevel(1);
        lifted.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void liftLevel2() {
        lifted.setTargetPosition(level2target);
        setLiftLevel(2);
        lifted.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void liftLevel3() {
        lifted.setTargetPosition(level3target);
        setLiftLevel(3);
        lifted.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void AUTONlevel() {
        lifted.setTargetPosition(auton);
        setLiftLevel(auton);
        lifted.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public  boolean liftBusy() { return  lifted.isBusy(); }


    public int getLevel3Target(){
        return level3target;

    }

    public int getLevel2Target(){
        return level2target;

    }

    public int getLevel1Target(){
        return level1target;

    }

    public int getBottomTarget(){
        return level0target;

    }

    public int getauton(){
        return auton;

    }

    public void printIntakeTelemetry(Telemetry t) {
        t.addData("Lift Encoder Value", lifted.getCurrentPosition());
        t.addData("Climber Encoder Value", climber.getCurrentPosition());
        t.addData("Climb Lifter Encoder Value", liftclimber.getCurrentPosition());
        t.addData("Lift Target Position", liftLevel);
        t.addData("Gripper Open", gripperOpen);
    }


}
