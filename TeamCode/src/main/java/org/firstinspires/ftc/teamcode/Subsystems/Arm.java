package org.firstinspires.ftc.teamcode.Subsystems;


import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Arm
{

    // Define hardware objects
    public DcMotor      armLeft      = null;
    public DcMotor      armRight     = null;
    // adding a comment to demo a git pull command

    private static final int ARM_STONE_LOAD = 50; // Left side reference
    private static final int ARM_STONE_CARRY = 200;// Left side reference
    private static final double ARM_SPEED = .5;
    private static final double ARM_RESET_POWER = .1;

    private ElapsedTime     runtime = new ElapsedTime();

    // Contructor for Arm
    public Arm(){

    }

    public void init(HardwareMap hwMap){

        armLeft = hwMap.get(DcMotor.class,"Arm");
        armRight =  hwMap.get(DcMotor.class,"Arm_2");

        armLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if needed
        armRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if needed

        armLeft.setPower(0);
        armRight.setPower(0);

        // Set all motors to run with encoders.

        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveToPickupStone()  {
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

    public void resetArmPosn(){
        runtime.reset();
        while (runtime.seconds() < 3.0) {
            armLeft.setPower(- ARM_RESET_POWER);
            armRight.setPower(-ARM_RESET_POWER);
        }
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveByJoystick(float y){
        float lift;
        lift = (-y/2); //divides the power by 2 to reduce power
        armLeft.setPower(lift);
        armRight.setPower(lift);

    }
}