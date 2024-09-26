package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawSubsystem extends SubsystemBase {

    private final Servo angleOfClaw;
    private final Servo driverOfClaw;
    //TODO: Figure out what these values are
    private final float open = 1;
    private final float closed = 0;


    //hMap is understandable, name is the name of the servo used
    public clawSubsystem(final HardwareMap hMap, final String wristName, final String openCloseName){
        angleOfClaw = hMap.get(Servo.class, wristName);
        driverOfClaw = hMap.get(Servo.class, openCloseName);
    }

    public void open(){
        driverOfClaw.setPosition(open);
    }

    public void close(){
        driverOfClaw.setPosition(closed);
    }
    //Using a dorect connection, this should hold up
    public void setToAngle(int degrees){angleOfClaw.setPosition((1/360)*degrees);}
}
