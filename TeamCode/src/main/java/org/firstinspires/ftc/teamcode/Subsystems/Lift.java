package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    // Define hardware objects
    public DcMotor liftmtr = null; // REV Core HEX motor
    private DigitalChannel liftswitch;
    public Telemetry telemetry;
    boolean liftAtStartPosn = false;

    private static final int    LIFT_STONE_LOAD         = 150; // Left side reference
    private static final int    LIFT_ARM_STONE_LEVEL1   = 300;// Left side reference
    private static final int    LIFT_ARM_STONE_LEVEL2   = 700;// Left side reference
    private static final double LIFT_SPEED              = 0.50;
    private static final double LIFT_RESET_SPEED        = -.25;
    private static final int    timeoutS                = 4; // time out in seconds keep at 4 or less or opMode will fail because it sees another loop
    private ElapsedTime         runtime                 = new ElapsedTime();

    // Constructor



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

    public void resetLift(Telemetry telemetry) {
        runtime.reset();
        liftmtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftmtr.setPower(LIFT_RESET_SPEED);

        while ((runtime.seconds() < timeoutS) &&
                (liftAtStartPosn() == false)) {


        }
        liftmtr.setPower(0);
        liftmtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Lift Resetting", "Elapsed Time: %2.2f S", runtime.seconds());
    }

    public boolean liftAtStartPosn() {

        if (liftswitch.getState() == true) {
            // when the switch is depressed the blue light come on and getState=false
            // the switch returns a true if it is not pressed and the blue light is NOT on.
            liftAtStartPosn = false;
        } else {
            liftAtStartPosn = true;
        }
        return liftAtStartPosn;

    }
}