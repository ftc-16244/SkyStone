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

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;




@TeleOp(name="Iterative OpMode Test ", group="Teleop")
//@Disabled
public class Iterative_OpMode_Test extends OpMode{



    // Create instances for all of the subsystem components for this opmode.
    // because this is a teleop opmode we need all of the systems

    FoundationMover foundationMover = new FoundationMover();
    Arm arm = new Arm();
    Gripper gripper = new Gripper();



    @Override
    public void init() {
        // Initialize hardware of all sub-systems
       foundationMover. init(hardwareMap);
       arm.init(hardwareMap);
       gripper.init(hardwareMap);

       //position robot into start position - for example the 18x18x18 inch dimensions
       gripper.moveToStartPsn();
       telemetry.addData("Fdn Mover Init ", "Complete ");
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

        //========================================
        // GAME PAD 1
        //========================================
        // left joystick is assigned to drive speed


        // foundation moving servo assignment to drivers gamepad
        if (gamepad1.a) {
            foundationMover.moveToStore();
        }

        if (gamepad1.b) {
            //foundationMover.moveToGrab();
        }


        //========================================
        // GAME PAD 2
        //========================================

        // gripper assignment to X and Y buttons on implement gamepad
        /*
        if (gamepad2.x) {
            gripper.moveToClose();
        }

        if (gamepad2.y) {
            gripper.moveToOpen();
        }

        if (gamepad2.a) {
            arm.moveToPickupStone();
        }

        if (gamepad2.b) {
            arm.moveToCarryStone();
        }
        */

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
