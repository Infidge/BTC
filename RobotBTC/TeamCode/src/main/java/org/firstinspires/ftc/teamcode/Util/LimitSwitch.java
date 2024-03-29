package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class LimitSwitch {

    public DigitalChannel digitalSwitch;
    public String switchName;
    public boolean switchState;

    public LimitSwitch(DigitalChannel limitSwitch, String name){
        this.digitalSwitch = limitSwitch;
        this.switchName = name;
    }

    public boolean isPressed(){
        if (switchState)
            return true;
        else return false;
    }

}
