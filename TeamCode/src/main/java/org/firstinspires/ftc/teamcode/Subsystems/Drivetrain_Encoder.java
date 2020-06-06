package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Thread.sleep;

public class Drivetrain_Encoder {
    // Define hardware objects
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;


    public static final double COUNTS_PER_MOTOR_REV = 1120;         // REV HD HEX 40:1 motors
    public static final double DRIVE_GEAR_REDUCTION = 0.5;         // This is < 1.0 if geared UP 20 teeth drive 10 teeth driven
    public static final double WHEEL_DIAMETER_INCHES = 3.54;   // 90mm wheels. For figuring circumference its a 90 millimeter wheel
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 1;
    private static final double TURN_SPEED = 0.5;
    private boolean inTeleOp;
    private ElapsedTime runtime = new ElapsedTime();

    HardwareMap hwMap = null;        // create a hardware map object here

    // Contructor for Drivetrain
    public Drivetrain_Encoder(boolean inTeleOp) {
    this.inTeleOp = inTeleOp;

    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        leftFront = hwMap.get(DcMotor.class, "Left_front");
        rightFront = hwMap.get(DcMotor.class, "Right_front");


        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // not in teleop means autonomous which meansd we need encoders
        if (!inTeleOp) {
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {
        // for InTeleop we don't need encoders
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }
    public void driveByEncoder(double speed,
                                  double leftInches, double rightInches,
                                  double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;


        // Ensure that the opmode is still active


            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy())) {
                // put telemetry here when it gets sorted out
            }

            // Stop all motion;

            leftFront.setPower(0);
            rightFront.setPower(0);


            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(250);   // optional pause after each move


    }

}