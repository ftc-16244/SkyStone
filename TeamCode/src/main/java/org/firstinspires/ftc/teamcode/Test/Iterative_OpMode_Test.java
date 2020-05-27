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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enums.ArmState;
import org.firstinspires.ftc.teamcode.Enums.DriveState;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;

import static org.firstinspires.ftc.teamcode.Enums.ArmState.CONTINUOUS;
import static org.firstinspires.ftc.teamcode.Enums.ArmState.DISCRETE;
import static org.firstinspires.ftc.teamcode.Enums.DriveState.STATE_FAST;
import static org.firstinspires.ftc.teamcode.Enums.DriveState.STATE_SLOW;


@TeleOp(name="Iterative OpMode Test ", group="Teleop")
//@Disabled
public class Iterative_OpMode_Test extends OpMode{


    // Create instances for all of the subsystem components for this opmode.
    // because this is a teleop opmode we need all of the systems

    private FoundationMover foundationMover  =    new FoundationMover();
    private Arm             arm              =    new Arm();
    private Gripper         gripper          =    new Gripper();
    private Drivetrain      drivetrain       =    new Drivetrain(true);
    private DriveState      currDriveState;
    private ArmState        currArmMode;

    @Override
    public void init() {
        // Call init methods in all needed system classes
       foundationMover. init(hardwareMap);
       arm.init(hardwareMap);
       gripper.init(hardwareMap);
       drivetrain.init(hardwareMap);
       telemetry.addData("Hardware is Initiaized ", "Complete ");
       //position robot into start position - for example the 18x18x18 inch dimensions
        gripper.moveToStartPsn();

        arm.resetArmPosn();

       telemetry.addData("Arm and Gripper Reset", "Complete ");
       currDriveState = DriveState.STATE_FAST;
       currArmMode =ArmState.DISCRETE;

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    // vision code to scan for objects would go here. Possibly encoder resets as well
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // move implements to "game ready position" can unfold or move outside the 18 in cube.


        foundationMover.moveToStore(); // start match with foundation mover in the "up" position
        arm.moveToCarryStone();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double speedfactor = 0.5;  //multiplier for SLOW speed


        //========================================
        // GAME PAD 1
        //========================================
        // left joystick is assigned to drive speed
        // left joystick is assigned to drive speed
        drive = -gamepad1.left_stick_y;
        // right joystick is for turning
        turn  =  gamepad1.right_stick_x;
        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max; // does this to stay within the limit and keeps the ratio the same
            right /= max;
        }

        // foundation moving servo assignment to drivers gamepad
        if (gamepad1.a) {
            foundationMover.moveToStore();
        }

        if (gamepad1.b) {
            foundationMover.moveToGrab();

        }

        // set-up drive speed states on bumpers
        if (gamepad1.left_bumper)
        {
            currDriveState = STATE_FAST;
        }
        if (gamepad1.right_bumper)
        {
            currDriveState = STATE_SLOW;
        }

        //========================================
        // GAME PAD 2
        //========================================

        // gripper assignment to X and Y buttons on implement gamepad

        if (gamepad2.x) {
            gripper.moveToClose();
        }

        if (gamepad2.y) {
            gripper.moveToOpen();
        }



        // set-up arm mode states on bumpers
        if (gamepad2.left_bumper)
        {
            currArmMode =    CONTINUOUS;


        }
        if (gamepad2.right_bumper)
        {
            currArmMode = DISCRETE;
        }

        // switch case for the drive speed state

        switch(currDriveState) {

            case STATE_FAST:
                telemetry.addData("Drive Speed",currDriveState);
                drivetrain.leftFront.setPower(left);
                drivetrain.rightFront.setPower(right);

                // Send telemetry message to signify robot running;
                telemetry.addData("left",  "%.2f", left);
                telemetry.addData("right", "%.2f", right);
                break;

            case STATE_SLOW: // power reduced with speedfactor variable
                telemetry.addData("Drive Speed",currDriveState);
                drivetrain.leftFront.setPower(left*speedfactor);
                drivetrain.rightFront.setPower(right*speedfactor);

                // Send telemetry message to signify robot running;
                telemetry.addData("left",  "%.2f", left);
                telemetry.addData("right", "%.2f", right);
                break;
        }

        switch(currArmMode) {

            case DISCRETE://gamepad buttons go to preset arm positions
                telemetry.addData("Arm Mode",currArmMode);
                if (gamepad2.a) {
                    arm.moveToPickupStone();
                }

                if (gamepad2.b) {
                    arm.moveToCarryStone();
                }
                break;

            case CONTINUOUS: //joystick controls arm
                telemetry.addData("Arm Mode",currArmMode);
                arm.moveByJoystick(-gamepad2.left_stick_y);
                break;
        }




    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
