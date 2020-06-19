package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {

    // Define hardware objects
    public DcMotor liftmtr = null; // REV Core HEX motor
    public DigitalChannel liftswitch;


    private static final int LIFT_STONE_LOAD = 100; // Left side reference
    private static final int LIFT_ARM_STONE_LEVEL1 = 200;// Left side reference
    private static final int LIFT_ARM_STONE_LEVEL2 = 350;// Left side reference
    private static final double LIFT_SPEED = 1.0;
    private static final double LIFT_RESET_SPEED = -.2;
    private static final int timeoutS = 10; // time out in seconds
    private ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap hwMap) {

        //motor
        liftmtr = hwMap.get(DcMotor.class, "Lift");
        liftmtr.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if needed
        liftmtr.setPower(0);
        liftmtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // limit switch
        liftswitch = hwMap.get(DigitalChannel.class, "LiftSwitch");
        // set the digital channel to input.
        liftswitch.setMode(DigitalChannel.Mode.INPUT);

    }

    public void moveToPickup() {
        liftmtr.setTargetPosition(LIFT_STONE_LOAD);
        liftmtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmtr.setPower(Math.abs(LIFT_SPEED));



    }

    public void moveToStackLevel1() {
        liftmtr.setTargetPosition(LIFT_ARM_STONE_LEVEL1);
        liftmtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmtr.setPower(LIFT_SPEED);

    }

    public void moveToStackLevel2() {
        liftmtr.setTargetPosition(LIFT_ARM_STONE_LEVEL2);
        liftmtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmtr.setPower(Math.abs(LIFT_SPEED));

    }

    public void resetLift() {
        runtime.reset();
        liftmtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftmtr.setPower(-LIFT_SPEED);
        while ((runtime.seconds() < timeoutS) &&
                (liftswitch.getState() == true )) {

            // put telemetry here when it gets sorted out
        }
        liftmtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}