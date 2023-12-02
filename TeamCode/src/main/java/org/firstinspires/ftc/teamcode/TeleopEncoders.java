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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Autonomous(name="Auto Test", group="Iterative Opmode")
@TeleOp(name="Teleop-Encoders", group="Iterative Opmode")
//@Disabled
public class TeleopEncoders extends OpMode {
    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;
    private Sensors sensors;
   private Intake intake;
    private Vision vision;

    private Orientation angles = null;

    private int intakeCase = 1;


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

        try {
            sensors.initGyro();
            telemetry.addData("Gyro", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Gyro", "Error: " + e.toString());
        }


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //GTA Drive Code
        if (gamepad1.right_bumper) {

            drive.strafe(-1);
        } else if (gamepad1.left_bumper) {
            drive.strafe(1);
        } else if (gamepad1.left_trigger > .2) {
            drive.teleopDrive(-gamepad1.left_trigger, gamepad1.right_stick_x);
        } else {
            drive.teleopDrive(gamepad1.right_trigger, gamepad1.right_stick_x);
        }
        //Fine control robot
        if (gamepad1.a) {
            drive.straightDrive(.3);
        } else if (gamepad1.b) {
            drive.strafe(-.3);
        } else if (gamepad1.x) {
            drive.strafe(.3);
        } else if (gamepad1.y) {
            drive.straightDrive(-.3);
        }

       if(gamepad2.a) {
            intake.openGripper();

        }
        if(gamepad2.b) {
            intake.closeGripper();
        }
        if(gamepad2.x){
            intake.gripperGrabPosition();
        }
        if(gamepad2.y) {
            intake.gripperPlacePosition();
        }

        switch(intakeCase) {
            case 1:
                //Pick Up
                if(gamepad2.dpad_right) {
                    intake.moveToGrabPosition();
                    intakeCase = 2;
                }
                //Place position
                if(gamepad2.dpad_up) {
                    intake.moveToPlacePosition();
                    intakeCase = 2;
                }
                //collapse
                if(gamepad2.dpad_down) {
                    intake.moveToTransitPosition();
                    intakeCase = 2;
                }
                //climb
                if(gamepad2.left_bumper) {
                    intake.moveToClimbPosition();
                    intakeCase = 2;
                }

                intake.runChain(-gamepad2.right_stick_y);
                if(gamepad2.left_stick_y > .1) {
                    //actuator in
                    intake.moveActuatorIn(-gamepad2.left_stick_y);

                } else if(gamepad2.left_stick_y < -.1) {
                    //actuator out
                    intake.moveActuatorOut(-gamepad2.left_stick_y);
                } else {
                    intake.stopActuator();
                }
                break;
            case 2:
                //move actuator
                if(intake.getActuatorTargetDirecton() == "IN") {
                    intake.moveActuatorIn(-.5);
                    if(intake.getActuatorPosition() < intake.getCurrentActuatorTarget()) {
                        intake.stopActuator();
                        intakeCase = 3;
                    }
                } else if(intake.getActuatorTargetDirecton() == "OUT"){
                    intake.moveActuatorOut(.5);
                    if(intake.getActuatorPosition() > intake.getCurrentActuatorTarget()) {
                        intake.stopActuator();
                        intakeCase = 3;
                    }
                } else {
                    intake.stopActuator();
                    intakeCase = 3;
                }



                break;
            case 3:
                if(intake.getChainTargetDirecton() == "UP") {
                    intake.runChain(.5);
                    if(intake.getChainPosition() > intake.getCurrentChainTarget()) {
                        intake.runChain(0);
                        intakeCase = 4;
                    }
                } else if(intake.getChainTargetDirecton() == "DOWN") {
                    intake.runChain(-.5);
                    if(intake.getChainPosition() < intake.getCurrentChainTarget()) {
                        intake.runChain(0);
                        intakeCase = 4;
                    }
                } else {
                    intake.runChain(0);
                    intakeCase = 4;
                }

                break;
            case 4:
                intake.runActuatorWithEncoder();
                intake.runChainWithEncoder();
                intakeCase = 1;
                break;
            default:
                break;

        }







        //Get gyro reading
        angles = sensors.readGyroAngle();


        intake.printIntakeTelemetry(telemetry);
        telemetry.addData("Intake Case", intakeCase);
        telemetry.addData("Gyro", sensors.getGyroZ(angles));
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}


