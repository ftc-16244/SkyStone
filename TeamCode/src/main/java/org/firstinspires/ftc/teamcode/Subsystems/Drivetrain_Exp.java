package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

public class Drivetrain_Exp {
    // Define hardware objects
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public BNO055IMU imu;

    public static final double COUNTS_PER_MOTOR_REV = 1120;         // REV HD HEX 40:1 motors
    public static final double DRIVE_GEAR_REDUCTION = 0.5;         // This is < 1.0 if geared UP 20 teeth drive 10 teeth driven
    public static final double WHEEL_DIAMETER_INCHES = 3.54;   // 90mm wheels. For figuring circumference its a 90 millimeter wheel
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 1;
    private static final double TURN_SPEED = 0.5;
    private boolean inTeleOp;
    private ElapsedTime runtime = new ElapsedTime();
    Telemetry telemetry;
    public LinearOpMode linearOpMode;

    //HardwareMap hwMap = null;        // create a hardware map object here

    // Contructor for Drivetrain
    public Drivetrain_Exp(boolean inTeleOp) {

        //this.inTeleOp = inTeleOp;


    }

    public void init(HardwareMap hwMap) {

        //hwMap = ahwMap;
        // initialize the imu first

        imu = hwMap.get(BNO055IMU.class, "imu");
        leftFront = hwMap.get(DcMotor.class, "Left_front");
        rightFront = hwMap.get(DcMotor.class, "Right_front");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Calibrate
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!linearOpMode.isStopRequested() && imu.isGyroCalibrated())  {
            linearOpMode.sleep(50);
            linearOpMode.idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();



        // not in teleop means autonomous which means we need encoders
        if (!inTeleOp) {
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        } else {
            // for InTeleop we don't need encoders
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }

    }


}