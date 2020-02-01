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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This program is used to test sharing methods from other opmodes by extending a class with all the necessary methods.
 */
// Code to move the foundation into the building zone during Autonomous Mode

@Autonomous(name="X02 5B Alternate Test Only", group="Experimental")
@Disabled

// extend AutoFoundation becasue it has all the Fields (constants) and methods we need to reuse here
public class X02_5B_Test_only extends Auto_FoundationMove {

    /* Declare OpMode members. */

    @Override
    public void runOpMode() {


        robot.init(hardwareMap); // don't forget this line
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
        // drive to center parking spot
        encoderDrive(DRIVE_SPEED, 38, 38, 4);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }



}


