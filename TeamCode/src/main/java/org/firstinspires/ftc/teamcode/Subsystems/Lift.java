package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {

    // Define hardware objects
    public DcMotor lift = null; // REV Core HEX motor
    public DigitalChannel liftswitch = null;


    private static final int LIFT_STONE_LOAD = 50; // Left side reference
    private static final int LIFT_ARM_STONE_LEVEL1 = 100;// Left side reference
    private static final int LIFT_ARM_STONE_LEVEL2 = 200;// Left side reference
    private static final double LIFT_SPEED = .5;
    private static final double LIFT_RESET_SPEED = -.2;
    private static final int timeoutS = 2; // time out in seconds
    private ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap hwMap) {

        //motor
        lift = hwMap.get(DcMotor.class, "Lift");
        lift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if needed
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // limit switch
        liftswitch = hwMap.get(DigitalChannel.class, "LiftSwitch");
        // set the digital channel to input.
        liftswitch.setMode(DigitalChannel.Mode.INPUT);

    }

    public void moveToPickup() {
        lift.setTargetPosition(LIFT_STONE_LOAD);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(Math.abs(LIFT_SPEED));


    }

    public void moveToStackLevel1() {
        lift.setTargetPosition(LIFT_ARM_STONE_LEVEL1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(Math.abs(LIFT_SPEED));

    }

    public void moveToStackLevel2() {
        lift.setTargetPosition(LIFT_ARM_STONE_LEVEL2);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(Math.abs(LIFT_SPEED));

    }

    public void resetLift() {
        runtime.reset();
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(-LIFT_SPEED);
        while ((runtime.seconds() < timeoutS) &&
                (liftswitch.getState() == true )) {
            // put telemetry here when it gets sorted out
        }
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}