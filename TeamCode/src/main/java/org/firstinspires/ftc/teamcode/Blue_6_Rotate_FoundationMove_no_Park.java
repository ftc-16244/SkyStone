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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
// Code to move the foundation into the building zone during Autonomous Mod



@Autonomous(name="#6B Blue Foundation no Park", group="Pushbot")
//@Disabled
public class Blue_6_Rotate_FoundationMove_no_Park extends Auto_FoundationMove {

    // There are no methods in this class. That is whay we extendes the Auto_FoundationMove class
    /* Declare OpMode members. */
    HardwarePushbot2        robot   = new HardwarePushbot2();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    Auto_FoundationMove     motion = new Auto_FoundationMove();


    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;         // REV HD HEX 40:1 motors
    private static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;         // This is < 1.0 if geared UP 20 teeth drive 10 teeth driven
    private static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;   // 90mm wheels. For figuring circumference its a 90 millimeter wheel
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = 0.4;
    private static final double     TURN_SPEED              = 0.5;
    private static final double     TRACK_WIDTH             =   15;
    private static final double     COUNTS_PER_ARM_MOTOR_REV    = 280 ;         // REV HD HEX 40:1 motors
    public static final double     ARM_SPEED             = 0.8;
    private static final double     ARM_GEAR_REDUCTION    = 4.0 ;   // This should be 1.0 or more for an arm. Count teeth and calculate
    private static final double     Ticks_Per_Degree        = COUNTS_PER_ARM_MOTOR_REV * ARM_GEAR_REDUCTION/360;

    private static final double     FOUNDATION_UP =0.4;
    private static final double     FOUNDATION_DOWN  =0.65;


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
        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftFront.getCurrentPosition(),
                          robot.rightFront.getCurrentPosition());
        telemetry.update();
        robot.foundationleft.setPosition(FOUNDATION_UP);
        robot.foundationright.setPosition(1-FOUNDATION_UP);

        sleep(250);     // pause for servos to move

        // Wait for the game to start (driver presses PLAY)
        waitForStart(); //once press start, everything below will happen
        //armDrive(ARM_SPEED,  2, 1.);  // S1: 180 degrees counterclockwise
        encoderDrive(DRIVE_SPEED, -12, -12, 4);
        encoderDrive(DRIVE_SPEED, -7, 7, 3); //first turn
        encoderDrive(DRIVE_SPEED, -14, -14, 3);
        encoderDrive(DRIVE_SPEED, 7.5, -7.5, 3);
        encoderDrive(DRIVE_SPEED, -13, -13, 4);
        // grab foundation with servos
        robot.foundationleft.setPosition(FOUNDATION_DOWN); //lift them so they don't get destroyed
        robot.foundationright.setPosition(1-FOUNDATION_DOWN);
        //pause for servos to capture foundation
        sleep(500);

        // Drive forward in a left arc to rotate foundation 90 degrees
        leftArcDrive(DRIVE_SPEED,22,115,5);
        // backup and push foundation inot the corner
        encoderDrive(DRIVE_SPEED, -18, -18, 4);
        // release foundation
        robot.foundationleft.setPosition(FOUNDATION_UP); //lift them so they don't get destroyed
        robot.foundationright.setPosition(1-FOUNDATION_UP);
        // pause for servos
        sleep(500);//
        // do not park in this opmode
        //encoderDrive(DRIVE_SPEED, 38, 38, 4);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }




}
