package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

abstract class EncoderDrive extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain = new Drivetrain(false);

    private static final double COUNTS_PER_MOTOR_REV = 1120;         // REV HD HEX 40:1 motors
    private static final double DRIVE_GEAR_REDUCTION = 0.5;         // This is < 1.0 if geared UP 20 teeth drive 10 teeth driven
    private static final double WHEEL_DIAMETER_INCHES = 3.54;   // 90mm wheels. For figuring circumference its a 90 millimeter wheel
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 0.4;
    private static final double TURN_SPEED = 0.5;
    private static final double TRACK_WIDTH = 15;



    /* Constructor */
    public EncoderDrive() {

    }



        public void encoderDrive ( double speed,
        double leftInches, double rightInches,
        double timeoutS){
            int newLeftTarget;
            int newRightTarget;
        drivetrain.init(hardwareMap);

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = drivetrain.leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = drivetrain.rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                drivetrain.leftFront.setTargetPosition(newLeftTarget);
                drivetrain.rightFront.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                drivetrain.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drivetrain.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                drivetrain.leftFront.setPower(Math.abs(speed));
                drivetrain.rightFront.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (drivetrain.leftFront.isBusy() && drivetrain.rightFront.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Target", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Actual_Position", "Running at %7d :%7d", drivetrain.leftFront.getCurrentPosition(), drivetrain.rightFront.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;

                drivetrain.leftFront.setPower(0);
                drivetrain.rightFront.setPower(0);


                // Turn off RUN_TO_POSITION
                drivetrain.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drivetrain.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                sleep(250);   // optional pause after each move
            }

        }
    }
