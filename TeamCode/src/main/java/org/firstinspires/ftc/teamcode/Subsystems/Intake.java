package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    // Define the motor
   public DcMotor intake = null;

   private static final double INTAKE_SPEED = .5;
   private static final double EJECT_SPEED = -.5;
   private static final double STOP_SPEED = 0;

   // Constructor
   public Intake(){

   }

    // Initialize the motor and give it a name
    public void init(HardwareMap hwMap){
    intake = hwMap.get(DcMotor.class,"Intake");
    // Set direction
    intake.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if needed
     // make sure it is off
    intake.setPower(0);
    // Set all motors to run without encoders.
    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


   }


    // turn on the intake to pull in Skystones
    public void intakeStones(){

       intake.setPower(INTAKE_SPEED);

    }

   // turn backwards to throw out stones
    public void ejectStones(){
       intake.setPower(EJECT_SPEED);
    }

    // turn off the intake
    public void  stopIntake(){
       intake.setPower(STOP_SPEED);
    }


}
