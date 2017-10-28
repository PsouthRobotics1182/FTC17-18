package org.firstinspires.ftc.finemen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

import FineLib.FineMotor;
import FineLib.FineMotorSystem;

/**
 * Created by drew on 10/6/17.
 */
@Autonomous
public class SlideDrive {

    FineMotorSystem leftDrive;
    FineMotorSystem rightDrive;
    FineMotor crossDrive;
    HardwareMap hwMap;

    double horizontal = 0, forward = 0;
    double rotation = 0;
    //                L  R  C
    double[] power = {0, 0, 0};

    public SlideDrive (HardwareMap hwMap) {
        leftDrive = new FineMotorSystem(hwMap, "leftMotor", 2, 2, DcMotorSimple.Direction.FORWARD);
        rightDrive = new FineMotorSystem(hwMap, "rightMotor", 2, 2, DcMotorSimple.Direction.REVERSE);
        crossDrive = new FineMotor(hwMap, "crossMotor", 2, DcMotorSimple.Direction.FORWARD);
    }

    public void setTransformation (double horizontal, double forward, double rotation) {
        this.horizontal = horizontal;
        this.forward = forward;
        this.rotation = rotation;

        power[0] = forward;
        power[1] = forward;
        power[2] = horizontal;

        power[0] += rotation;
        power[1] -= rotation;
        ArrayList<Double> powers = new ArrayList<Double>() {{add(power[0]); add(power[1]); add(power[2]);}};

        powers = airMode(powers);
        setPowers(powers.get(0), powers.get(1), powers.get(2));

    }

    private void setPowers(double leftPower, double rightPower, double crossPower) {
        power[0] = leftPower;
        power[1] = rightPower;
        power[2] = crossPower;
    }

    public void power() {
        leftDrive.setPower(power[0]);
        rightDrive.setPower(power[1]);
        crossDrive.setPower(power[2]);
    }
    public ArrayList<Double> airMode(ArrayList<Double> powers) {
        ArrayList<Double> absPower = powers;
        for (int i = 0; i < absPower.size(); i++) {
            absPower.set(i, Math.abs(powers.get(i)));
        }
        double max = Collections.max(absPower);
        if (max > 1) {
            for (int i = 0; i < powers.size(); i++) {
                powers.set(i, powers.get(i)/max);
            }
        }
        return powers;
    }
}
