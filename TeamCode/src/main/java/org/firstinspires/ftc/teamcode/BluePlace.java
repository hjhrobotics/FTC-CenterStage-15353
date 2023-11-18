/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Place", group="Iterative Opmode")
//@Disabled
public class BluePlace extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;
    private Sensors sensors;
    private Intake intake;

    private Orientation angles = null;

    public Vision vision = null;

    private int autoCase = 1;
    private int rightEncoderTarget = 0;
    private int leftEncoderTarget = 0;
    private int gyroTarget = 0;
    private String markerLocation = "";

    private double commandStartTime = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        drive = new Drive(hardwareMap);
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision();

        intake.closeGripper();

        try {
            sensors.initGyro();
            telemetry.addData("Gyro", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Gyro", "Error: " + e.toString());
        }

        try {
            vision.createOpenCvPipeline(hardwareMap, "BLUE");
            vision.startCvStream();
            telemetry.addData("Vision", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Vision", "Error: " + e.toString());
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("Marker Location", vision.getCvMarkerLocation());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        //markerLocation = vision.getCvMarkerLocation();
        markerLocation = "CENTER";
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Get gyro reading
        angles = sensors.readGyroAngle();

        switch (autoCase) {
            case 1:
                //Drive forward to prep to place

                leftEncoderTarget = 900;
                rightEncoderTarget = 900;
                drive.leftEncoderToPosition(leftEncoderTarget);
                drive.rightEncoderToPosition(rightEncoderTarget);
                autoCase = 2;
                break;
            case 2:
                drive.straightDrive(.5);
                if(drive.getRightEncoderValue() >= rightEncoderTarget && drive.getLeftEncoderValue() >= leftEncoderTarget) {
                    drive.straightDrive(0);
                    commandStartTime = runtime.seconds() + 1;
                    autoCase = 3;
                }
                break;
            case 3:
                //Select the starting location
                if(runtime.seconds() > commandStartTime) {

                    if (markerLocation == "CENTER") {
                        //Already at the center, continue
                        intake.openGripper();
                        intake.upOnePixelFlat();
                        autoCase = 4;

                    } else if (markerLocation == "RIGHT") {
                        //Left and right go here
                        autoCase = 20;

                    } else {
                        autoCase = 30;
                    }
                }
                break;
            case 4:
                //Enable the lift so it moves
                intake.moveLift(.4);
                if (intake.getEncodedLift() <= intake.getCurrentLiftTarget()) {
                    intake.stopLift();
                    //Close the gripper on the Gold


                    autoCase = 401;
                }

                break;
            case 401:
                //chill for a sec
                if(runtime.seconds() > commandStartTime) {
                    commandStartTime = runtime.seconds() + 1;
                    autoCase = 5;
                }
                break;
            case 5:
                intake.closeGripper();
                if(runtime.seconds() > commandStartTime) {
                    intake.gripperUp();
                    autoCase = 6;
                }
                break;
            case 106:
                intake.closeGripper();
                if(runtime.seconds() > commandStartTime) {
                    autoCase = 6;
                }
                break;
            case 6:
                //Back up a bit
                leftEncoderTarget = drive.getLeftEncoderValue() - 300;
                rightEncoderTarget = drive.getRightEncoderValue() - 300;
                drive.leftEncoderToPosition(leftEncoderTarget);
                drive.rightEncoderToPosition(rightEncoderTarget);
                autoCase = 7;
                break;
            case 7:
                drive.straightDrive(-.5);
                if(drive.getRightEncoderValue() <= rightEncoderTarget && drive.getLeftEncoderValue() <= leftEncoderTarget) {
                    drive.straightDrive(0);
                    autoCase = 999;
                }
                break;
            case  8:
                //Turn to face the board
                //Set gyro Target
                gyroTarget = 55;
                drive.tankDrive(-.4, .4);
                if(sensors.getGyroZ(angles) > gyroTarget) {
                    //case 100 is the finish of the process
                    autoCase = 100;
                }
                break;
        //************************************** Start of Right ********************************
            case 20:
                autoCase = 21;
                break;
        //************************************** Start of Left  ********************************
            case 30:
                autoCase = 31;
                break;
        //************************************** Start of Final process to place ********************************
            case 100:
                //set board target, square up to board
                leftEncoderTarget = drive.getLeftEncoderValue() + 1200;
                rightEncoderTarget = drive.getRightEncoderValue() + 1600;
                drive.leftEncoderToPosition(leftEncoderTarget);
                drive.rightEncoderToPosition(rightEncoderTarget);
                autoCase = 101;
                break;
            case 101:
                //Drive to the board, set the lift target for middle when done
                drive.tankDrive(.4, .45);
                if(drive.getRightEncoderValue() >= rightEncoderTarget && drive.getLeftEncoderValue() >= leftEncoderTarget) {
                    drive.straightDrive(0);
                    intake.liftToMiddle();
                    autoCase = 102;
                }
                break;
            case 102:
                intake.moveLift(.6);
                if (intake.getEncodedLift() <= intake.getCurrentLiftTarget()) {
                    intake.stopLift();
                    intake.gripperUp();
                    commandStartTime = runtime.seconds() + 2;

                    autoCase = 103;
                }
                break;
            case 103:
                //Wait for a bit to let the gripper get up
                if(runtime.seconds() >  commandStartTime) {
                    autoCase = 124;
                }
                break;
            case 124:
                //Move to board -- set targets
                leftEncoderTarget = drive.getLeftEncoderValue() + 13;
                rightEncoderTarget = drive.getRightEncoderValue() + 15;
                drive.leftEncoderToPosition(leftEncoderTarget);
                drive.rightEncoderToPosition(rightEncoderTarget);
                autoCase = 125;
                break;
            case 125:
                //Move the robot to the set targets
                drive.straightDrive(.3);
                if(drive.getRightEncoderValue() >= rightEncoderTarget && drive.getLeftEncoderValue() >= leftEncoderTarget) {
                    drive.straightDrive(0);
                    commandStartTime = runtime.seconds() + 2;

                    autoCase = 126;
                }
                break;
            case 126:
                //Open the gripper to release the pixel
                intake.openGripper();
                //Wait a bit to avoid pixel going odd places
                if(runtime.seconds() >  commandStartTime) {
                    autoCase = 127;
                }
                break;
            case 127:
                //Move away from board -- set targets
                leftEncoderTarget = drive.getLeftEncoderValue() - 13;
                rightEncoderTarget = drive.getRightEncoderValue() - 15;
                drive.leftEncoderToPosition(leftEncoderTarget);
                drive.rightEncoderToPosition(rightEncoderTarget);
                autoCase = 128;
                break;
            case 128:
                //Move back slowly
                drive.straightDrive(-.3);
                if(drive.getRightEncoderValue() <= rightEncoderTarget && drive.getLeftEncoderValue() <= leftEncoderTarget) {
                    drive.straightDrive(0);
                    commandStartTime = runtime.seconds() + 1;
                    autoCase = 129;
                }
                break;
            case 129:
                //Lift and gripper down to start teleop
                intake.gripperDown();
                //Set the target for the lift to the bottom
                intake.liftToBottom(gamepad2.right_bumper);
                autoCase = 130;
                break;
            case 130:
                intake.moveLift(-.5);
                if (intake.getEncodedLift() >= intake.getCurrentLiftTarget()) {
                    intake.stopLift();
                    autoCase = 131;
                }
                break;
            case 131:
                drive.tankDrive(0, -.6);
                if(sensors.getGyroZ(angles) <= -67) {
                    drive.tankDrive(0, 0);
                    autoCase = 132;
                }
                break;
            default:
                drive.teleopDrive(0, 0);
                intake.moveLift(0);
                break;

        }

        telemetry.addData("Auto Case", autoCase);
        telemetry.addData("Gyro Z", sensors.getGyroZ(angles));
        telemetry.addData("Marker Location", markerLocation);

        telemetry.addData("Left Encoder Target", leftEncoderTarget);
        telemetry.addData("Right Encoder Target", rightEncoderTarget);
        telemetry.addData("Gyro Target", gyroTarget);

      //  drive.printDriveTelemetry(telemetry);
        intake.printIntakeTelemetry(telemetry);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}


