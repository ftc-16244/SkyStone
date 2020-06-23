package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

public class Drivetrain_Encoder {
    // Define hardware objects
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public Telemetry telemetry;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private static final double COUNTS_PER_MOTOR_REV = 1120;         // REV HD HEX 40:1 motors
    public static final double DRIVE_GEAR_REDUCTION = 0.5;         // This is < 1.0 if geared UP 20 teeth drive 10 teeth driven
    public static final double WHEEL_DIAMETER_INCHES = 3.54;   // 90mm wheels. For figuring circumference its a 90 millimeter wheel
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 1;
    private static final double TURN_SPEED = 0.5;
    private boolean inTeleOp;

    private ElapsedTime runtime = new ElapsedTime();
    Orientation lastAngle = new Orientation();

    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private double degrees;
    private double globalAngle;

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

            //sleep(250);   // optional pause after each move


    }

    public void driveByGyro ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active


            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftFront.getCurrentPosition() + moveCounts;
            newRightTarget = rightFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftFront.setPower(speed);
            rightFront.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (leftFront.isBusy() &&  rightFront.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftFront.setPower(leftSpeed);
                rightFront.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (holdTimer.time() < holdTime) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angleDesired     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */


    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */


    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }





    /////////////////// Auto specific methods //////////////////////////////////////////////////////
    public void setDegrees(double degrees){
        this.degrees = degrees;
    }

    public double getDegrees(){
        return degrees;
    }

    public void resetAngle(){
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

    }

    public double getAngle(){

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngle.firstAngle;


        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = angles;

        return globalAngle;

    }

}