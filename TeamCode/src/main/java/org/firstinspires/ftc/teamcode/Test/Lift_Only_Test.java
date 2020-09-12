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

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.ArmState;
import org.firstinspires.ftc.teamcode.Enums.DriveState;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Lift_Demo;

import static org.firstinspires.ftc.teamcode.Enums.ArmState.CONTINUOUS;
import static org.firstinspires.ftc.teamcode.Enums.ArmState.DISCRETE;
import static org.firstinspires.ftc.teamcode.Enums.DriveState.STATE_FAST;
import static org.firstinspires.ftc.teamcode.Enums.DriveState.STATE_SLOW;


@TeleOp(name="Lift Only Test ", group="Teleop")
//@Disabled
public class Lift_Only_Test extends OpMode{


    // Create instance of the lift subsystem. This opmode tests the lift motor and the limit switch

    Lift lift             =    new Lift();

    //private ElapsedTime     runtime         =       new ElapsedTime();

    @Override
    public void init() {
        // Call init methods for all implements needed in this opmode. Usually it will be all
       lift.init(hardwareMap);
       telemetry.addData("Hardware is Initiaized ", "Complete ");

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
        // empty for this example
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //========================================
        // GAME PAD 2 Only for this test opMode
        //========================================

        // gripper assignment to X and Y buttons on implement gamepad
        // does not work 5/28. wires are in correct port too
        if (gamepad2.x) {
            lift.moveToPickup();
            telemetry.addData("Button X Pushed", "Complete ");
            telemetry.addData("Count",  "Running at %7d",
                   lift.liftmtr.getCurrentPosition());
        }

        if (gamepad2.y) {
            lift.moveToStackLevel1();
            telemetry.addData("Button Y Pushed", "Complete ");
        }
        if (gamepad2.a) {
            lift.moveToStackLevel2();
            telemetry.addData("Button A Pushed", "Complete ");
        }

        if (gamepad2.b) {
            lift.resetLift(telemetry);
            telemetry.addData("Reset", "Complete ");
        }



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
