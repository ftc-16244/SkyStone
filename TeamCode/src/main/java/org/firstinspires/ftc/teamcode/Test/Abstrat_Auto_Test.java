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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.MotionLib;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationMover;

@Autonomous(name="Autonomous Test", group="Linear Opmode")
//@Disabled
public class Abstrat_Auto_Test extends MotionLib {

    // Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain = new Drivetrain(false);
    FoundationMover foundationMover = new FoundationMover();

    @Override
    public void runOpMode() {

        drivetrain.init(hardwareMap); // some issues here still
        foundationMover.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();
        driveByEncoder(DRIVE_SPEED, -12, -12, 4);
        driveByEncoder(DRIVE_SPEED, -7, 7, 3); //
        driveByEncoder(DRIVE_SPEED, -14, -14, 3);
        driveByEncoder(DRIVE_SPEED, 7.5, -7.5, 3);
        driveByEncoder(DRIVE_SPEED, -13, -13, 4);
        //grab foundation - both servos
        foundationMover.moveToGrab();

        //pause to let servos get to their position
        sleep(500);     // pause for servos to move
        // now pull foundation straight into building zone
        //encoderDrive(DRIVE_SPEED, 40, 40, 4);
        // release foundation to be ready for teleop
        foundationMover.moveToStore(); //lift them so they don't get destroyed
        sleep(500);
        // run until the end of the match (driver presses STOP)

        armDriveByEncoder(1,20,3);

    }


}
