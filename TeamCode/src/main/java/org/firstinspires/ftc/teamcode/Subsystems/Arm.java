package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm
{
   // Define hardware objects
    private DcMotor      armLeft      = null;
    private DcMotor      armRight     = null;
    // adding a comment to demonstarte Git and Github

    private static final int ARM_STONE_LOAD = 250; // Left side reference
    private static final int ARM_STONE_CARRY = 500;// Left side reference
    private static final double ARM_SPEED = .5;

    HardwareMap hwMap           =  null;        // create a hardware map object here

    // Contructor for Arm
    public Arm(){

    }

    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;
        armLeft = hwMap.get(DcMotor.class,"Arm");
        armRight =  hwMap.get(DcMotor.class,"Arm_2");

        armLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if needed
        armRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if needed

        armLeft.setPower(0);
        armRight.setPower(0);

        // Set all motors to run with without encoders.

        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void moveToPickupStone(){
        armLeft.setTargetPosition(ARM_STONE_LOAD);
        armRight.setTargetPosition(ARM_STONE_LOAD);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setPower(Math.abs(ARM_SPEED));
        armRight.setPower(Math.abs(ARM_SPEED));
    }

    public void moveToCarryStone() {
        armLeft.setTargetPosition(ARM_STONE_CARRY);
        armRight.setTargetPosition(ARM_STONE_CARRY);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setPower(Math.abs(ARM_SPEED));
        armRight.setPower(Math.abs(ARM_SPEED));
    }

}