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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Autonomous(name="Auto Test", group="Iterative Opmode")
@TeleOp(name="Teleop-v2", group="Iterative Opmode")
//@Disabled
public class TeleopV2 extends OpMode {
    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;
    private Sensors sensors;
   private Intake intake;
    private Vision vision;

    private Orientation angles = null;


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
        if(gamepad2.b){
            intake.closeGripper();
        }
        if(gamepad2.x) {
            intake.gripperDown();

        }
        if(gamepad2.y){
            intake.gripperUp();
        }


        intake.runClimber(-gamepad2.right_stick_y);

        intake.moveLift(-gamepad2.left_stick_y);

        //intake.liftToBottom(gamepad2.dpad_right);
/*
       if(gamepad2.left_stick_y < -.15) {
           intake.raiseLift(-gamepad2.left_stick_y);
       } else if(gamepad2.left_stick_y > .15) {
           intake.lowerLift(-gamepad2.left_stick_y);
       } else {
           intake.moveLift(0);
       }
/*
        if(gamepad2.left_trigger > .08) {
            intake.setLiftclimber(-gamepad2.left_trigger);
        } else if(gamepad2.right_trigger > .08) {
            intake.setLiftclimber(gamepad2.right_trigger);
        } else {
            intake.setLiftclimber(0);
        }
*/


        //Get gyro reading
        //angles = sensors.readGyroAngle();





        telemetry.addData("LR Position", drive.getLeftEncoderValue());
        telemetry.addData("LR Drive Busy", drive.leftMotorBusy());
        telemetry.addData("LR Position", drive.getRightEncoderValue());
        telemetry.addData("RR Drive Busy", drive.rightMotorBusy());

        telemetry.addData("Trigger Value - r", gamepad2.right_trigger);
        telemetry.addData("Trigger Value - l", gamepad2.left_trigger);
        telemetry.addData("Slide", intake.getEncodedLift());


        //intake.printIntakeTelemetry(telemetry);
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}


