package org.firstinspires.ftc.teamcode.vikingroboticsteamcode2020.Fall2020.lib;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.vikingroboticsteamcode2020.Fall2020.lib.Category.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class Init extends LinearOpMode {
    public DcMotor motorLeftFront, motorRightFront, motorLeftBack, motorRightBack;
    public List<DcMotor> chassisMotors, leftMotors, rightMotors, frontMotors, backMotors;
    public IntegratingGyroscope gyroScope;
    public ModernRoboticsI2cGyro gyro;
    public PID pidGyro, pidLeftFront, pidRightFront, pidLeftBack, pidRightBack;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime timer = new ElapsedTime();
    private class MotorPID extends BoundedPID {
        private DcMotor motor;
        public MotorPID(DcMotor motor, double kP, double kI, double kD, double tolerance, double infOut, double supOut) {
            super(kP, kI, kD, tolerance, infOut, supOut);
            this.motor = motor;
        }
        @Override void control(double output) {setDriving.of(output);}
        @Override double read() {return motor.getCurrentPosition();}
        @Override void end() {motor.setPower(0);}
    }
    protected Hom2<DcMotor, Double, Unit> setPower = new In2<DcMotor, Double>() {
        @Override void run(DcMotor motor, Double x) {
            motor.setPower(x);
        }
    };
    protected Hom2<DcMotor, Integer, Unit> setTarget = new In2<DcMotor, Integer>() {
        @Override void run(DcMotor motor, Integer x) {
            motor.setTargetPosition(x);
        }
    };
    protected Hom2<DcMotor, DcMotor.RunMode, Unit> setMode = new In2<DcMotor, DcMotor.RunMode>() {
        @Override void run(DcMotor motor, DcMotor.RunMode mode) {
            motor.setMode(mode);
        }
    };
    protected Hom<Double, Unit> setRotating = new In<Double>() {
        @Override void run(Double power) {
            flip(setPower).of(-power).fmap(leftMotors);
            flip(setPower).of(power).fmap(rightMotors);
        }
    };
    protected Hom<Double,Unit> setDriving = new In<Double>() {
        @Override void run(Double power) {
            flip(setPower).of(power).fmap(chassisMotors);
        }
    };
    protected Hom<Integer, Unit> setTargets = new In<Integer>() {
        @Override void run(Integer encoder) {
            flip(setTarget).of(encoder).fmap(chassisMotors);
        }
    };
    protected Hom<DcMotor.RunMode, Unit> setModes = new In<DcMotor.RunMode>() {
        @Override void run(DcMotor.RunMode mode) {
            flip(setMode).of(mode).fmap(chassisMotors);
        }
    };
    protected Hom<DcMotor, Boolean> isBusy = new Hom<DcMotor, Boolean>() {
        @Override Boolean of(DcMotor motor) {
            return motor.isBusy();
        }
    };
    protected void stopAndReset() {
        setModes.of(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriving.of(0.);
    }
    protected Hom<Unit, Unit> stopAndReset = new Eff() {
        @Override public void run() {
            setModes.of(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriving.of(0.);
        }
    };
    public void driveFor(double power, int encoder) {
        stopAndReset();
        setTargets.of(encoder);
        setModes.of(DcMotor.RunMode.RUN_TO_POSITION);
        setDriving.of(power);
        boolean[] flags = {false, false, false, false};
        Hom<DcMotor, String> encoderData = new Hom<DcMotor, String>(){@Override public String of(DcMotor mot) {
            return String.format(Locale.US, "%d%s", mot.getCurrentPosition(), mot.isBusy()?"...":"!");
        }};
        while(opModeIsActive() && joinl(conj, isBusy.fmap(chassisMotors))) {
            telemetry.addData("LF['\\]: ", encoderData.of(motorLeftFront));
            telemetry.addData("RF[/']: ", encoderData.of(motorRightFront));
            telemetry.addData("LB[./]: ", encoderData.of(motorLeftBack));
            telemetry.addData("RB[\\.]: ", encoderData.of(motorRightBack));
            telemetry.update();
            sleep(1);
        }
        stopAndReset();
    }
    @Override public void runOpMode() {
        initChassisMotors();
        initPID();
        // initGyro();
        waitForStart();
    }
    private void initChassisMotors(){
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightFront = hardwareMap.dcMotor.get("RightFront");
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        chassisMotors = new ArrayList<DcMotor>()
            {{add(motorLeftFront);add(motorRightFront);add(motorLeftBack);add(motorRightBack);}};
        leftMotors = new ArrayList<DcMotor>() {{add(motorLeftFront);add(motorLeftBack);}};
        rightMotors = new ArrayList<DcMotor>() {{add(motorRightFront);add(motorRightBack);}};
        frontMotors = new ArrayList<DcMotor>() {{add(motorLeftFront);add(motorRightFront);}};
        backMotors = new ArrayList<DcMotor>() {{add(motorLeftBack);add(motorRightBack);}};
    }
    private void initPID(){
        pidGyro = new CyclicPID(1.0, 1.0, 1.0, 1.0, -1.0, 1.0, -360, 359) {
            @Override public void control(double output) {setRotating.of(output);}
            @Override public double read() {return gyro.getHeading();}
            @Override public void end() {setRotating.of(0.);}
        };
        pidLeftFront = new MotorPID(motorLeftFront, 1.0, 1.0, 1.0, 0.1, -1.0, 1.0);
        pidRightFront = new MotorPID(motorRightFront, 1.0, 1.0, 1.0, 0.1, -1.0, 1.0);
        pidLeftBack = new MotorPID(motorLeftBack, 1.0, 1.0, 1.0, 0.1, -1.0, 1.0);
        pidRightBack = new MotorPID(motorRightBack, 1.0, 1.0, 1.0, 0.1, -1.0, 1.0);
    }
    private void initGyro() {
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyroScope = (IntegratingGyroscope) gyro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        gyro.calibrate();
        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && gyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();
    }
    public void waitForStart() {
        telemetry.addData(">>>", "Press Play to start tracking");
        telemetry.update();
        super.waitForStart();
        telemetry.clear();
    }
}
