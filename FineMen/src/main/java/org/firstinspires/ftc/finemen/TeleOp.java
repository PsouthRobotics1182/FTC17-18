package org.firstinspires.ftc.finemen;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by drew on 10/27/17.
 */

public class TeleOp extends OpMode {
    SlideDrive slide;
    @Override
    public void init() {
        slide = new SlideDrive(hardwareMap);
    }

    @Override
    public void loop() {
        double leftRight = gamepad1.right_stick_x;
        double frontBack = gamepad1.right_stick_y;
        double rotation = gamepad1.left_stick_x;
        slide.setTransformation(leftRight, frontBack, rotation);

        slide.power();
    }
}
