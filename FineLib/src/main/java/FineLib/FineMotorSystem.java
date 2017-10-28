package FineLib;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;

public class FineMotorSystem {

    private ArrayList<FineMotor> motors;
    private ArrayList<Integer> positions;
    private double wheelRadius;
    private double linearSpeed = 0;
    private double rotationalSpeed = 0;
    private double power = 0;
    private double desiredRotations = 0;

    //the gear ratio after the output shaft of the motor
    public FineMotorSystem(HardwareMap hwMap, String nameBase, int gearRatio, int num, DcMotorSimple.Direction[] directions) {
        if (num != directions.length) {
            throw new IndexOutOfBoundsException("FineDcMotorSystem must be given equal number of directions and number of motors");
        }
        motors = new ArrayList<>();
        positions = new ArrayList<>();
        for (int i = 0; i < num; i ++) {
            FineMotor motor = new FineMotor(hwMap, nameBase + i, gearRatio, directions[i]);
            motor.setPower(0);
            motors.add(motor);
            positions.add(0);
        }
    }
    public FineMotorSystem(HardwareMap hwMap, String nameBase, int gearRatio, int num, DcMotorSimple.Direction direction) {
        DcMotorSimple.Direction[] directions = new DcMotorSimple.Direction[num];
        for (int i = 0; i < directions.length; i++) {
            directions[i] = direction;
        }
        motors = new ArrayList<>();
        positions = new ArrayList<>();
        for (int i = 0; i < num; i ++) {
            FineMotor motor = new FineMotor(hwMap, nameBase + i, gearRatio, directions[i]);
            motor.setPower(0);
            motors.add(motor);
            positions.add(0);
        }
    }
    public void setLinearSpeed(double linearSpeed) {
        this.linearSpeed = linearSpeed;
        setRotationalSpeed(lin2rot(linearSpeed));
    }
    public void setRotationalSpeed(double rotationalSpeed){
        this.rotationalSpeed = rotationalSpeed;
        setPower(this.rotationalSpeed/(getMaxRotationalSpeed()));
    }
    public void setPower(double power) {
        this.power = power;
        for (FineMotor motor: motors)
            motor.setPower(power);
        resetPositions();
    }
    public void power() {
        for (int i = 1; i < motors.size(); i++) {
            positions.set(i, Math.abs(motors.get(i).getCurrentPosition()));
        }

        int smallest = Collections.min(positions);
        for (FineMotor motor : motors) {
           int diff = Math.abs(motor.getCurrentPosition()) - smallest;
           if (diff > 50) {
                motor.setPower(motor.getPower() * 0.9);
           }
        }

        for (FineMotor motor: motors) {
            motor.power();
        }
    }

    public void setDistance(int mm) {
        double circumfrence = Math.PI * wheelRadius * 2;
        desiredRotations = mm / circumfrence;
        resetPositions();
    }
    public void setDesiredRotations(int desiredRotations) {
        this.desiredRotations = desiredRotations;
        resetPositions();
    }
    public boolean drive(double power) {
        if (getCurrentPosition() < rot2tick(desiredRotations)) {
            power();
            return true;
        } else {
            return false;
        }
    }
    public void stop() {
        setPower(0);
        for (FineMotor motor: motors) {
            motor.stop();
        }
    }
    public int getCurrentPosition() {
        return Math.abs(motors.get(0).getCurrentPosition());
    }
    public double getMaxRotationalSpeed() {
        return motors.get(0).maxRotationalSpeed();
    }
    public void resetPositions() {
        for (FineMotor motor: motors)
            motor.resetPosition();
    }
    private double lin2rot(double linSpeed) {
        double circumfrence = Math.PI * wheelRadius * 2;
        double rotations = linSpeed/circumfrence;
        double rotSpeed = rotations * 60;
        return rotSpeed;
    }
    private double rot2lin(double rotSpeed) {
        double circumfrence = Math.PI * wheelRadius * 2;
        rotSpeed = rotSpeed / 60;
        double linSpeed = rotSpeed * circumfrence;
        return linSpeed;
    }
    private double rot2tick(double rots) {
        return rots * motors.get(0).getMotorType().getTicksPerRev();
    }
    private double stdev(ArrayList<Integer> values) {
        double powerSum1 = 0;
        double powerSum2 = 0;
        double stdev = 0;

        for (int i = 0; i <= values.size(); i++){
            powerSum1 += values.get(i);
            powerSum2 += Math.pow(values.get(i), 2);
            stdev = Math.sqrt(i*powerSum2 - Math.pow(powerSum1, 2))/i;
            //System.out.println(total[i]); // You specified that you needed to print
            // each value of the array
        }
        return stdev;
    }

}
