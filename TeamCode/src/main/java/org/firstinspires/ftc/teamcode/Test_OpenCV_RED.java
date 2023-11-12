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

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="RED OpenCV Test", group="Iterative Opmode")
//@Disabled
public class Test_OpenCV_RED extends OpMode {
    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;
    private Sensors sensors;
    private Intake intake;
    private Vision vision;

    private Orientation angles = null;

    private int autoCase = 1;
    private int encoderTarget = 0;

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

        try {
            vision.createOpenCvPipeline(hardwareMap, "RED");
            vision.startCvStream();
            telemetry.addData("Vision", "Initialized");
            //set in opmode when pipeline is built
            telemetry.addData("Expected Alliance", vision.getExpectedAlliance());
        } catch (Exception e) {
            telemetry.addData("Vision", "Error: " + e.toString());
        }
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


        telemetry.addData("Expected Alliance", vision.getExpectedAlliance());
        telemetry.addData("Marker Location", vision.getCvMarkerLocation().toString());
        telemetry.update();
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
        //Get gyro reading
        angles = sensors.readGyroAngle();

        switch (autoCase) {
            case 1:
                drive.leftEncoderToPosition(1000);
                encoderTarget = drive.getLeftEncoderValue() + 1000;
                autoCase = 2;
                break;
            case 2:
                drive.teleopDrive(.4, 0);
                if(drive.getLeftEncoderValue() >= encoderTarget) {
                    drive.runUsingEncoders();
                    autoCase = 3;
                }
                break;

            default:
                drive.teleopDrive(0, 0);
                intake.moveLift(0);
                break;

        }




        telemetry.addData("Gyro Z", sensors.getGyroZ(angles));

        telemetry.addData("Lift Pos", intake.getEncodedLift());
        telemetry.addData("Lift Busy", intake.liftBusy());
        telemetry.addData("Lift Level", intake.getLiftLevel());
        telemetry.addData("LR Position", drive.getLeftEncoderValue());
        telemetry.addData("LR Drive Busy", drive.leftMotorBusy());
        telemetry.addData("LR Position", drive.getRightEncoderValue());
        telemetry.addData("RR Drive Busy", drive.rightMotorBusy());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}


