package org.firstinspires.ftc.finemen;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by drew on 10/6/17.
 */

public class Robot {
    HardwareMap hwMap; 
    FineDcMotor leftMotor;
    FineD
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        leftMotor.isBusy();
    }
}
