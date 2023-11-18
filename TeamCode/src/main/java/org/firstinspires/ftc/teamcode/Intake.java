package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    // --- Declare DC Motors here ---
    private DcMotor lifted = null;
    private DcMotor climber = null;
    private DcMotor liftclimber = null;

    // --- Declare Servos Here ---
    private Servo gripper = null;
    private Servo rotategripper = null;

    // -- Subsystem Variables ---

    private boolean gripperOpen = false;

    private int currentLiftTarget = 0;
    private String  liftRunmode = "manual"; //expect manual or encoder
    private String liftTargetDirecton = "up"; //up or down
    private boolean liftAvailable = true;



    // Encoder Values used to limit lift from going too high or too low
    //2023 Values, 2 stages, 117rpm motor
    private int liftUpperLimit = -4130;
    private int liftLowerLimit = -90;

    //Encoder values for adjusting lift
    private int liftGroundEncoderValue = -22;
    private int liftMiddleLevelEncoderValue = -2750;

    private int pixelEncoderValueFlat = -75;
    private int pixelEncoderValueUpright = -750;

    private int climberTopPosition = 63200;
    private int climberLiftTopPosition = -300;


    private int auton = -7500;

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
       // liftclimber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lifted.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  liftclimber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gripperDown();
        closeGripper();

    }


    // --- Gripper Methods ---
    //Open and close the gripper
    public void closeGripper() {
        gripper.setPosition(.7);
        gripperOpen = true;
    }

    public void openGripper() {
        gripper.setPosition(.4);
        gripperOpen = false;
    }

    public void gripperUp() {
        rotategripper.setPosition(0);
    }

    public void gripperDown() {
        rotategripper.setPosition(1);
    }

    // --- Lift Methods ---

    public void toggleLiftOperationMode() {
        //expect manual or encoder
        if(liftRunmode == "encoder") {
            liftRunmode = "manual";
        } else {
            liftRunmode = "encoder";
        }

    }
    public void setLiftModeToEncoder() { liftRunmode = "encoder"; }
    public void setLiftModeToManual() { liftRunmode = "manual"; }

    public String getLiftOperationMode() {
        return liftRunmode;
    }

    public void moveLift(double speed) {
        lifted.setPower(speed);
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

    //Use after running to position to reset runmode
    public void liftRunWithEncoders() {
        lifted.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getEncodedLift(){
        return lifted.getCurrentPosition();
    }
    public int getCurrentLiftTarget() { return currentLiftTarget; }
    public String getLiftMotorRunmode() {
        return lifted.getMode().toString();
    }
    public String getLiftTargetDirecton() {
        return liftTargetDirecton;
    }
    public void liftToBottom(boolean right_bumper) {
        liftTargetDirecton = "down";
        currentLiftTarget = liftGroundEncoderValue;
        lifted.setTargetPosition(currentLiftTarget);
        lifted.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void liftToMiddle() {
        liftTargetDirecton = "up";
        currentLiftTarget = liftMiddleLevelEncoderValue;
        lifted.setTargetPosition(currentLiftTarget);
        lifted.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void upOnePixelFlat() {
        liftTargetDirecton = "up";
        currentLiftTarget = lifted.getCurrentPosition() + pixelEncoderValueFlat;
        lifted.setTargetPosition(currentLiftTarget);
        lifted.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void upOnePixelUpright() {
        liftTargetDirecton = "up";
        currentLiftTarget = lifted.getCurrentPosition() + pixelEncoderValueUpright;
        lifted.setTargetPosition(currentLiftTarget);
        lifted.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    public  boolean liftBusy() { return  lifted.isBusy(); }

    public void runClimber(double speed) {
        double s = Range.clip(speed, -1, 1);
        climber.setPower(s);
    }
   /* public void setLiftclimber(double speed) {
        liftclimber.setPower(speed * .5);
    }
*/
    public void runClimberToTop() {
        climber.setTargetPosition(climberTopPosition);
        climber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void climberRunWithEncoders() {
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runLiftClimberToTop() {
        liftclimber.setTargetPosition(climberLiftTopPosition);
        liftclimber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void climberLiftRunWithEncoders() { liftclimber.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }

    public int getClimberTopPosition() { return climberTopPosition; }
    public int getClimberLiftTopPosition() { return climberLiftTopPosition; }
    public int getClimberPosition() { return climber.getCurrentPosition(); }
    public int getLiftClimberPosition() { return liftclimber.getCurrentPosition(); }


    public void printIntakeTelemetry(Telemetry t) {
        t.addData("Lift Encoder Value", lifted.getCurrentPosition());
        t.addData("Lift Runmode", getLiftMotorRunmode());
        t.addData("Lift Op Mode", liftRunmode);
        t.addData("Lift Target Position", currentLiftTarget);
        t.addData("Lift Target Direction", liftTargetDirecton);

        t.addData("Climber Encoder Value", climber.getCurrentPosition());
        t.addData("Climb Lifter Encoder Value", liftclimber.getCurrentPosition());

        t.addData("Gripper Open", gripperOpen);
    }


    public String getLiftLevel() {
        return null;
    }
}
