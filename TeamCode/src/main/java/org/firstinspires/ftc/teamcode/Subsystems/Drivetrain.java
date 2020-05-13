package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    // Define hardware objects
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;


    private static final double COUNTS_PER_MOTOR_REV = 1120;         // REV HD HEX 40:1 motors
    private static final double DRIVE_GEAR_REDUCTION = 0.5;         // This is < 1.0 if geared UP 20 teeth drive 10 teeth driven
    private static final double WHEEL_DIAMETER_INCHES = 3.54;   // 90mm wheels. For figuring circumference its a 90 millimeter wheel
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 1;
    private static final double TURN_SPEED = 0.5;


    HardwareMap hwMap = null;        // create a hardware map object here

    // Contructor for Drivetrain
    public Drivetrain() {


    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        leftFront = hwMap.get(DcMotor.class, "Left_front");
        rightFront = hwMap.get(DcMotor.class, "Right_front");


        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setDrive2Teleop(HardwareMap hwMap) {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDrive2Auto(HardwareMap hwMap) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}