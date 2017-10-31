package org.firstinspires.ftc.finemen;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import FineLib.FineVision;

@TeleOp(name = "vuforia")
public class VuforiaOp extends LinearOpMode {

    FineVision eyes;

    @Override
    public void runOpMode() throws InterruptedException {
        eyes = new FineVision(hardwareMap, 150);
        telemetry.addData("setup complete",null);
        telemetry.update();

        waitForStart();
        eyes.activate();
        //begin tracking targets
        while (opModeIsActive()) {
            eyes.grab();
            //converts the trackable to a vumark so we can get the ID, or whch tower to put glyph in
            RelicRecoveryVuMark vuMark = eyes.getColumn();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {//only if we can identify the target
                telemetry.addData("VuMark", "%s visible", vuMark);

                telemetry.addData("Pose", eyes.toString());//prints the positon data

                if (eyes.getPose() != null) {
                    VectorF trans = eyes.getTranslation();
                    Orientation rot = eyes.getRotaion();

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;

                    float[] ballSetup = eyes.getBallSetup();
                    telemetry.addData("Ball Setup", ballSetup[0] + "," + ballSetup[1]);
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
        eyes.deactivate();
    }
}


