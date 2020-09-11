package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift_Demo {

    public DcMotor liftmtr = null;
    public double LIFT_SPEED_FAST = 0.7; // Limit of speed is -1 to 1
    public double LIFT_SPEED_SLOW = 0.3; // Limit of speed is -1 to 1

    public static final int TICKS_PER_IN = 102; //72 mm pulley
    //public static final int FIVE_INCH_LIFT_TICK_COUNT = 5*TICKS_PER_IN;

    public void init(HardwareMap hwMap) {

        //motor
        liftmtr = hwMap.get(DcMotor.class, "Lift"); //"Lift" has to be in the phone's config file
        liftmtr.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if needed
        liftmtr.setPower(0);
        liftmtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void lift5inches(){
        liftmtr.setTargetPosition(5*TICKS_PER_IN);
        liftmtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmtr.setPower(LIFT_SPEED_FAST);

    }

    public void lift10inches(){
        liftmtr.setTargetPosition(10*TICKS_PER_IN);
        liftmtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmtr.setPower(LIFT_SPEED_SLOW);


    }

}
