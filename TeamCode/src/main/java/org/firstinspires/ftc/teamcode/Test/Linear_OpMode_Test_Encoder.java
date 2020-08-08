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

package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain_Encoder;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;

import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.DRIVE_SPEED;


@TeleOp(name="Linear OpMode Test #2", group="Linear Opmode")
@Disabled
public class Linear_OpMode_Test_Encoder extends LinearOpMode {

    // Declare OpMode members.
    ElapsedTime     runtime          =    new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        FoundationMover     foundationMover  =    new FoundationMover();
        Arm                 arm              =    new Arm();
        Gripper             gripper          =    new Gripper();
        Drivetrain_Encoder  drivetrain       =    new Drivetrain_Encoder(false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize HW
        foundationMover. init(hardwareMap);
        arm.init(hardwareMap);
        gripper.init(hardwareMap);
        drivetrain.init(hardwareMap);
        telemetry.addData("Hardware is Initiaized ", "Complete ");
        //position robot into start position - for example the 18x18x18 inch dimensions
        gripper.moveToStartPsn();
        sleep(500);

        arm.armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.armLeft.setPower(-.3);
        arm.armRight.setPower(-.3);
        runtime.reset();

        while  (runtime.seconds() < 3.0) {
            telemetry.addData("Arm Resetting", "Leg 1: %2.5f S Elapsed", runtime.seconds());

        }

        arm.armLeft.setPower(0);
        arm.armRight.setPower(0);

        waitForStart();

        runtime.reset();
        drivetrain.driveByEncoder(DRIVE_SPEED, -12, -12, 4);
        drivetrain.driveByEncoder(DRIVE_SPEED, -7, 7, 3); //first turn
        drivetrain.driveByEncoder(DRIVE_SPEED, -14, -14, 3);
        drivetrain.driveByEncoder(DRIVE_SPEED, 7.5, -7.5, 3);
        drivetrain.driveByEncoder(DRIVE_SPEED, -13, -13, 4);
        //grab foundation - both servos
        foundationMover.moveToGrab();

        //pause to let servos get to their position
        sleep(500);     // pause for servos to move
        // now pull foundation straight into building zone
        drivetrain.driveByEncoder(DRIVE_SPEED, 40, 40, 4);
        // release foundation to be ready for teleop
        foundationMover.moveToStore(); //lift them so they don't get destroyed
        sleep(500);
        // run until the end of the match (driver presses STOP)

    }




}
