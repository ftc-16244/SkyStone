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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Iterative Test", group="Teleop")
//@Disabled
public class Teleop_Test extends OpMode{

    private enum State {
        STATE_DISCRETE,
        STATE_INFINITE,
    } // Enums to choose which mode the arm will operate in. Preset discrete of infinite via joystick


    /* Declare OpMode members. */
    HardwarePushbot2 robot       = new HardwarePushbot2(); // use the class created to define a Pushbot's hardware

    private State    currentState;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double ARM_SPEED = .8;
        double lift;



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // using ghe armrotator method from our instance of Auto_FoundationMove created above.
        //You have to create a separate instance in this case to get access to "armDrive"

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// in case robot gets bumped
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        newState(State.STATE_DISCRETE);
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
        double ARM_SPEED = .8;
        double lift;

        // note isBusy() applies to the Run_USING ENCODER mods
        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;
        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Output the safe vales to the motor drives.
        robot.leftFront.setPower(left);
        robot.rightFront.setPower(right);

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.update();

        // gamepad 2 control of arm using Y joysticsk
        //lift = -gamepad2.left_stick_y;
        //robot.arm.setPower(lift);
        // state change based on gampad bumpers
        if (gamepad2.left_bumper)
        {
            newState(State.STATE_DISCRETE);
        }
        if (gamepad2.right_bumper)
        {
            newState(State.STATE_INFINITE);
        }
        switch (currentState)
        {
            case STATE_DISCRETE: // push button
                robot.arm.setTargetPosition(400);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(Math.abs(ARM_SPEED));
                while (robot.arm.isBusy())
                {
                telemetry.addData("Discrete","Mode");
                telemetry.update();
                }

            case STATE_INFINITE:
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift = -gamepad2.left_stick_y;
                robot.arm.setPower(lift);
        }

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    //===================================================================
    // Helper Methods
    //==================================================================
    private void newState(State newState) // method to change states
    {
        currentState = newState;
    }
}
