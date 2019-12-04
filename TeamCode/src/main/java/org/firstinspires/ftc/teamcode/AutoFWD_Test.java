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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This program is used in the autonomous period
 *  * The file is for the red or blue alliance.
 *  * The robot starts against the wall and drives to the center line under the skybridge.
 */
// Code to move the foundation into the building zone during Autonomous Mode

@Autonomous(name="Test FWD", group="Pushbot")
//@Disabled
public class AutoFWD_Test extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot2        robot   = new HardwarePushbot2();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private Auto_FoundationMove     robotmotion = new Auto_FoundationMove();

    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;         // REV HD HEX 40:1 motors
    private static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;         // This is < 1.0 if geared UP 20 teeth drive 10 teeth driven
    private static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;   // 90mm wheels. For figuring circumference its a 90 millimeter wheel
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     DRIVE_SPEED             = 1;
    private static final double     TURN_SPEED              = 0.5;
    private static final double     COUNTS_PER_ARM_MOTOR_REV    = 280 ;         // REV HD HEX 40:1 motors
    private static final double     ARM_SPEED             = 0.8;
    private static final double     ARM_GEAR_REDUCTION    = 4.0 ;   // This should be 1.0 or more for an arm. Count teeth and calculate
    private static final double     Ticks_Per_Degree        = COUNTS_PER_ARM_MOTOR_REV * ARM_GEAR_REDUCTION/360;



    @Override
    public void runOpMode() {

        //Local variables//



        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftFront.getCurrentPosition(),
                          robot.rightFront.getCurrentPosition());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //sleep(1000);     //add stall time here if needed

        robotmotion.encoderDrive(DRIVE_SPEED, 25, 25, 5.);  // S3: Forward 30 Inches with 10 Sec timeout have to confirm
        //robotmotion.armDrive(15,0,3);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }



}


