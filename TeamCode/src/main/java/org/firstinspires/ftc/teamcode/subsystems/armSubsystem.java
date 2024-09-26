package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;

public class armSubsystem extends SubsystemBase {

    //Motor used to change the angle of the arm
    private final DcMotorEx extenderMotor;
    private final DcMotorEx angleMotor;
    public double armRad;
    public double armLen;


    public armSubsystem(final HardwareMap hmap, final String extension, final String angle){
        extenderMotor = hmap.get(DcMotorEx.class, extension);
        angleMotor = hmap.get(DcMotorEx.class, angle);
    }

    public void extendIn(int inches){
        //TODO: Some conversion of an input in INCHES to TICKS so that the motor, extender, lifts the arm system up INCHES.
        //this works so well 100% (its 2am)
        extenderMotor.setTargetPosition(inches);

    }
}
