package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.ArmState;
import org.firstinspires.ftc.teamcode.Enums.DriveState;

public class SkystoneRobot {

    // Create instances for all of the subsystem components for this opmode.
    // because this is a teleop opmode we need all of the systems

    public FoundationMover     foundationMover  =    new FoundationMover();
    public Arm                 arm              =    new Arm();
    public Gripper             gripper          =    new Gripper();
    public Drivetrain_Encoder  drivetrain       =    new Drivetrain_Encoder(true);
    public DriveState          currDriveState;
    public ArmState            currArmMode;
    public ElapsedTime         runtime          =    new ElapsedTime();

    SkystoneRobot(){

    }
}
