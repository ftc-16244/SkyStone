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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="Meet 3 Teleop", group="Teleop")
//@Disabled
public class Meet_3_Teleop extends OpMode{


    //set up states to change how the arm operates. Pre-sets or variable.
    private enum State {
        STATE_DISCRETE,
        STATE_CONTINUOUS
    } // Enums to choose which mode the arm will operate in. Preset discrete of continuous via joystick
    private enum Drive_State {
        STATE_FAST,
        STATE_SLOW
    }

    /* Declare OpMode members. */
    HardwarePushbot2 robot       = new HardwarePushbot2(); // use the class created to define a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    Auto_FoundationMove robotmotion = new Auto_FoundationMove();// adds arm drive and encoder drive methods


    private State                   currentState;
    private Drive_State                   currentDRIVE_STATE;
    private static final double     GRIPPER_START    = 1 ; //optional to make sure it starts inside 18 inches
    private static final double     GRIPPER_READY    = 0.5; //open gripper such that spatual touched inside frame when arm is on top of inside rail
    private static final double     GRIPPER_CLOSE    = 0.75;   // larger number grips tighter. 0.7 for sprocket is a good start
    private static final int        ARM_STONE_READY  = 20; // encoder counts where arm is ready to grab stone
    private static final int        ARM_STONE_CARRY  = 115; // encoder counts where arm is ready to grab stone
    private static final double     FOUNDATION_UP     = 0.3; // reference the left servo for position
    private static final double     FOUNDATION_DOWN   = 0.65;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //newState(State.STATE_CONTINUOUS);
        newState(State.STATE_DISCRETE);
        currentDRIVE_STATE = Drive_State.STATE_FAST;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.addData("CONTINUOUS","Mode");//
        //telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // using ghe armrotator method from our instance of Auto_FoundationMove created above.
        //You have to create a separate instance in this case to get access to "armDrive"


        robot.foundationright.setPosition(1-FOUNDATION_UP);
        robot.foundationleft.setPosition(FOUNDATION_UP);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// in case robot gets bumped
        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// in case robot gets bumped

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    robot.arm.setTargetPosition(ARM_STONE_READY);// lift up arm to allow gripper to open
    robot.arm2.setTargetPosition(ARM_STONE_READY);// lift up arm to allow gripper to open
    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.arm.setPower(.75);
    robot.arm2.setPower(.75);
    //robot.closey.setPosition(GRIPPER_READY);// open ready to grab a stone.
     // empty for now
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
        double ARM_SPEED = .25;
        double lift;
        double speedfactor = 2;

    //double-many decimal places
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

        // note isBusy() applies to the Run_USING ENCODER mods
        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        // Combine drive and turn for blended motion.


        // Output the safe vales to the motor drives.

        // Send telemetry message to signify robot running;
       // telemetry.addData("left",  "%.2f", left);
      //  telemetry.addData("right", "%.2f", right);

        // gripper assignment to X and Y buttons on implement gamepad
        if (gamepad2.x) {
            robot.closey.setPosition(GRIPPER_CLOSE);
        }

        if (gamepad2.y) {
            robot.closey.setPosition(GRIPPER_READY);
        }

        // foundation moving servo assignment to drivers gampad1
       if (gamepad1.a) {
            robot.foundationleft.setPosition(FOUNDATION_DOWN); // a is up
            robot.foundationright.setPosition(1-FOUNDATION_DOWN); // how we "mirror" a servo since we can use a negative sign
        }
        if (gamepad1.b) {
            robot.foundationleft.setPosition(FOUNDATION_UP); // b is down
            robot.foundationright.setPosition(1-FOUNDATION_UP); // how we "mirror" a servo since we can use a negative sign
       }

        // turn on and off he accumulator motor
        if (gamepad2.dpad_down) {
            robot.accumulator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //turns the accumulator on
            robot.accumulator.setPower(ARM_SPEED);
        }
        if ((gamepad2.dpad_left) |(gamepad2.dpad_right))  {
        robot.accumulator.setPower(0); //turns the accumulator off
        }
        if (gamepad2.dpad_up) {
            robot.accumulator.setPower(-ARM_SPEED); //turns the accumulator off
        }

        // set-up arm states on bumpers of implement gampad
        if (gamepad1.left_bumper)
        {
            currentDRIVE_STATE = Drive_State.STATE_FAST; //did this to make it continuos
        }
        if (gamepad1.right_bumper)
        {
            currentDRIVE_STATE = Drive_State.STATE_SLOW; //preset points
        }
        // switch case to determine what mode the arm needs to operate in.

        
        switch (currentState) {
            case STATE_DISCRETE: // push button
                telemetry.addData("Arm Mode", currentState);
                if (gamepad2.a) {
                    robot.arm.setTargetPosition(ARM_STONE_READY);
                    robot.arm2.setTargetPosition(ARM_STONE_READY);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(Math.abs(ARM_SPEED));
                    robot.arm2.setPower(Math.abs(ARM_SPEED));
                    telemetry.addData("Arm Target", "Ready to get stone");

                }
                if (gamepad2.b) {
                    robot.arm.setTargetPosition(ARM_STONE_CARRY);
                    robot.arm2.setTargetPosition(ARM_STONE_CARRY);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(Math.abs(ARM_SPEED));
                    robot.arm2.setPower(Math.abs(ARM_SPEED));
                    telemetry.addData("Carry Position", robot.arm.getTargetPosition());

                }


                break;
            case STATE_CONTINUOUS: {
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                lift = (-gamepad2.left_stick_y) / 2; //divides the power by 2 to reduce power
                robot.arm.setPower(lift);
                robot.arm2.setPower(lift);
                telemetry.addData("Arm Mode", currentState);
                telemetry.addData("Arm Power", "%.2f", lift);
                break;
            }

        }
              switch (currentDRIVE_STATE) {
                   case STATE_FAST:
                          robot.leftFront.setPower(left);
                          robot.rightFront.setPower(right);
                          break;

                      case STATE_SLOW:
                          robot.leftFront.setPower(left / speedfactor);
                          robot.rightFront.setPower(right / speedfactor);
                          break;

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
