package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    // --- Declare DC Motors here ---
    private DcMotor actuator = null;
    private DcMotor chain = null;

    // --- Declare Servos Here ---
    private Servo gripper = null; //Left Servo
    private Servo gripper2 = null; //RightServo
    private Servo intakeRotateServo = null;
    private Servo PlaneShooter = null;

    // -- Subsystem Variables ---

    private boolean gripperOpen = false;

    private int currentActuatorTarget = 0;
    private int currentChainTarget = 0;

    private String chainTargetDirecton = "UP"; //up or down
    private String actuatorTargetDirection = "OUT";

    // Encoder Values used to limit lift from going too high or too low
    //2023 Values, 2 stages, 117rpm motor
    private int actuatorMiddlePosition = 2000;
    private int actuatorInnerLimit = 175;
    private int actuatorOuterLimit = 2500;

    private int chainGroundPosition = 0;
    private int chainPlacePosition = 2000;
    private int chainClimbPosition = 3609;

    private int chainUpperLimit = 0;
    private int chainLowerLimit = 0;

    private double leftServoGripPosition = .7;
    private double leftServoReleasePosition = 1;
    private double rightServoGripPosition = 1;
    private double rightServoReleasePosition = .7;
    private double rotateServoInPosition = .66;
    private double rotateServoGrabPosition = .68;
    private double rotateServoPlacePosition = .58;





    public Intake(HardwareMap h) {

        gripper = h.get(Servo.class, "gripper");
        gripper2 = h.get(Servo.class, "Gripper2");
        intakeRotateServo = h.get(Servo.class, "intakeRotate");
        PlaneShooter = h.get(Servo.class, "PlaneShooter");

        actuator = h.get(DcMotor.class, "actuator");
        chain = h.get(DcMotor.class, "chain");

        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // liftclimber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        closeGripper();
        gripperGrabPosition();

    }


    // --- Gripper Methods ---
    //Open and close the gripper
    public void closeGripper() {
        gripper.setPosition(leftServoGripPosition);
        gripper2.setPosition(rightServoGripPosition);
        gripperOpen = false;
    }

    public void openGripper() {
        gripper.setPosition(leftServoReleasePosition);
        gripper2.setPosition(rightServoReleasePosition);
        gripperOpen = true;
    }
    public void openLeftGripper() {
        gripper.setPosition(leftServoReleasePosition);
        gripperOpen = true;
    }
    public void openRightGripper() {
        gripper2.setPosition(rightServoReleasePosition);
        gripperOpen = true;
    }
    public void ShootPlane() {
        PlaneShooter.setPosition(0);
    }
    public void ResetPlane() {
        PlaneShooter.setPosition(.5);
    }

    public void gripperPlacePosition() {
        intakeRotateServo.setPosition(rotateServoPlacePosition);
    }

    public void gripperGrabPosition() {
        intakeRotateServo.setPosition(rotateServoGrabPosition);
    }

    public void gripperInnerPosition() {
        intakeRotateServo.setPosition(rotateServoInPosition);
    }

    // --- Actuator Methods ---
    //Move method oly for testing/resetting
    public void moveActuator(double speed) {
        actuator.setPower(speed);
    }
    public void moveActuatorIn(double speed) {
        if (actuator.getCurrentPosition() > actuatorInnerLimit) {
            actuator.setPower(speed);
        } else {
            actuator.setPower(0);
        }
    }
    public void moveActuatorOut(double speed) {
        if (actuator.getCurrentPosition() < actuatorOuterLimit) {
            actuator.setPower(speed);
        } else {
            actuator.setPower(0);
        }
    }
    public void actuatorFullRetract(){
        currentActuatorTarget = actuatorInnerLimit;
        actuator.setTargetPosition(actuatorInnerLimit);
        actuatorTargetDirection = "IN";
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void actuatorFullExtend(){
        currentActuatorTarget = actuatorOuterLimit;
        actuator.setTargetPosition(actuatorOuterLimit);
        actuatorTargetDirection = "OUT";
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void actuatorToMiddle(){
        currentActuatorTarget = actuatorMiddlePosition;
        actuator.setTargetPosition(actuatorMiddlePosition);
        if(actuator.getCurrentPosition() > actuatorMiddlePosition) {
            actuatorTargetDirection = "IN";
        } else {
            actuatorTargetDirection = "OUT";
        }
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void stopActuator() {
        actuator.setPower(0);
    }

    public void runActuatorWithEncoder(){actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

    public int getActuatorPosition() { return actuator.getCurrentPosition(); }
    public int getCurrentActuatorTarget() { return currentActuatorTarget; }
    public String getActuatorTargetDirecton() {
        return actuatorTargetDirection;
    }

    //Chain Methods
    //Only in test mode, no limit
    public void runChain(double speed) {
        double s = Range.clip(speed, -1, 1);
        chain.setPower(s);
    }
    public void moveChainDown(double speed) {
        if (chain.getCurrentPosition() > chainLowerLimit) {
            chain.setPower(speed);
        } else {
            chain.setPower(0);
        }
    }
    public void moveChainUp(double speed) {
        if (chain.getCurrentPosition() < chainUpperLimit) {
            chain.setPower(speed);
        } else {
            chain.setPower(0);
        }
    }

    public void runChainWithEncoder(){chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

    public void moveToGrabPosition() {
        openGripper();
        gripperGrabPosition();
        currentChainTarget = chainGroundPosition;
        chain.setTargetPosition(chainGroundPosition);
        if(chain.getCurrentPosition() > chainGroundPosition) {
            chainTargetDirecton = "DOWN";
        } else {
            chainTargetDirecton = "UP";
        }
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuatorFullExtend();

    }

    public void moveToTransitPosition() {
        closeGripper();
        gripperInnerPosition();
        currentChainTarget = chainGroundPosition + 50;
        chain.setTargetPosition(chainGroundPosition + 50);
        if(chain.getCurrentPosition() > (chainGroundPosition + 50)) {
            chainTargetDirecton = "DOWN";
        } else {
            chainTargetDirecton = "UP";
        }
        actuatorFullRetract();
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void moveToTransitPositionAuto() {
        closeGripper();
        gripperGrabPosition();
        currentChainTarget = chainGroundPosition + 55;
        chain.setTargetPosition(chainGroundPosition + 55);
        if(chain.getCurrentPosition() > (chainGroundPosition + 55)) {
            chainTargetDirecton = "DOWN";
        } else {
            chainTargetDirecton = "UP";
        }
        actuatorFullRetract();
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void moveToPlacePosition() {
        currentChainTarget = chainPlacePosition;
        chain.setTargetPosition(chainPlacePosition);
        if(chain.getCurrentPosition() > chainPlacePosition) {
            chainTargetDirecton = "DOWN";
        } else {
            chainTargetDirecton = "UP";
        }
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuatorToMiddle();
        gripperPlacePosition();
        closeGripper();
    }

    public void moveToClimbPosition() {
        closeGripper();
        gripperInnerPosition();
        chain.setTargetPosition(chainClimbPosition);
        currentChainTarget = chainClimbPosition;
        if(chain.getCurrentPosition() > chainClimbPosition) {
            chainTargetDirecton = "DOWN";
        } else {
            chainTargetDirecton = "UP";
        }
        actuatorFullExtend();
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public int getChainPosition() { return chain.getCurrentPosition(); }
    public int getCurrentChainTarget() { return currentChainTarget; }
    public String getChainTargetDirecton() {
        return chainTargetDirecton;
    }


    public void printIntakeTelemetry(Telemetry t) {
        t.addData("Actuator Encoder Value", actuator.getCurrentPosition());
        t.addData("Actuator Target Position", currentActuatorTarget);
        t.addData("Actuator Target Direction", actuatorTargetDirection);

        t.addData("Chain Encoder Value", chain.getCurrentPosition());
        t.addData("Chain Target Position", currentChainTarget);
        t.addData("Chain Target Direction", chainTargetDirecton);

        t.addData("Gripper Open", gripperOpen);
    }

}
