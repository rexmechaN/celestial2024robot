package com.teamcelestial;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import java.util.ArrayList;
import java.util.List;

public class Robot extends TimedRobot
{
    /*private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(4);
    private final WPI_VictorSPX leftMaster = new WPI_VictorSPX(2);
    private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(7);
    private final WPI_VictorSPX rightMaster = new WPI_VictorSPX(9);

    private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftSlave);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightSlave);
    private final DifferentialDrive yagsDrive = new DifferentialDrive(leftGroup, rightGroup);

    private final SlewRateLimiter limiter = new SlewRateLimiter(0.8);*/

    Joystick joystick = new Joystick(0);

    private final CANSparkMax leftCim = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightCim = new CANSparkMax(16, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
    private final DoubleTopic desiredMotorSpeedTopic = networkTable.getDoubleTopic("/datatable/motorSpeed");
    private final DoubleSubscriber desiredMotorSpeedSubscription = desiredMotorSpeedTopic.subscribe(0);
    private final DoublePublisher desiredMotorSpeedPublisher = desiredMotorSpeedTopic.publish();

    private long lastRpmPublish = 0;
    private double multiplier = 0.0;

    private SparkMaxPIDController leftPid;
    private SparkMaxPIDController rightPid;

    private double rpmTarget = 0;

    private Shooter shooter;

    @Override
    public void robotInit()
    {
        desiredMotorSpeedPublisher.setDefault(0);
        desiredMotorSpeedPublisher.set(0);

        leftPid = leftCim.getPIDController();
        rightPid = rightCim.getPIDController();

        List<SparkMaxPIDController> controllerList = new ArrayList<>();
        controllerList.add(leftPid);
        controllerList.add(rightPid);

        shooter = new Shooter(controllerList);

        setupPid(leftPid);
        setupPid(rightPid);
    }

    void setupPid(SparkMaxPIDController controller) {
        System.out.println("Motor PID:");
        System.out.println("P: " + controller.getP());
        System.out.println("I: " + controller.getI());
        System.out.println("D: " + controller.getD());
        System.out.println("F: " + controller.getFF());
        System.out.println("Iacc: " + controller.getIAccum());
        System.out.println("======");
        double kP = 0.000000039;
        double kI = 0.00000029;
        double kD = 0;
        double kIz = 0;
        double kFF = 0;
        double kMaxOutput = 1.0;
        double kMinOutput = -1.0;
        double maxRPM = 5700;
        // set PID coefficients
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setIZone(kIz);
        controller.setFF(kFF);
        controller.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    private boolean increaseLatch = false;
    private boolean reduceLatch = false;
    private boolean shooterStarted = false;
    @Override
    public void teleopPeriodic() {
        boolean increase = joystick.getRawButton(5);
        boolean reduce = joystick.getRawButton(3);
        boolean reverse = joystick.getRawButton(12);

        if(increase && reduce) {
            rpmTarget = 0;
        } else if(increase && !increaseLatch) {
            rpmTarget = rpmTarget + 500;
        } else if(reduce && !reduceLatch) {
            rpmTarget = rpmTarget - 500;
        }

        increaseLatch = increase;
        reduceLatch = reduce;

        rpmTarget = Math.min(Math.max(0, rpmTarget), 5500);

        if(joystick.getRawButton(1) || reverse) {
            if(!shooterStarted) {
                shooter.start(3.95, 0.0, 23.0);
                shooterStarted = true;
            }
            //setMotor(reverse ? -rpmTarget : rpmTarget);
        } else {
            if(shooterStarted) {
                shooter.stop();
                shooterStarted = false;
            }
        }

        shooter.tick();

        double leftRpm = leftCim.getEncoder().getVelocity();
        double rightRpm = rightCim.getEncoder().getVelocity();

        if(lastRpmPublish + 500 < System.currentTimeMillis()) {
            lastRpmPublish = System.currentTimeMillis();
            //System.out.println("Left RPM: " + leftRpm + " \tRight RPM: " + rightRpm + " \tTarget RPM: " + rpmTarget);
        }
    }

    double reference = -1;

    void setMotor(double rpm) {
        if(true) return;
        if(reference == rpm) return;
        reference = rpm;
        System.out.println("Set motor: " + rpm);
        REVLibError lErr = leftPid.setReference(rpm, CANSparkMax.ControlType.kVelocity);
        REVLibError rErr = rightPid.setReference(rpm, CANSparkMax.ControlType.kVelocity);
        if(lErr != REVLibError.kOk) {
            System.out.println("L REVERROR: " + lErr.name());
        }
        if(rErr != REVLibError.kOk) {
            System.out.println("R REVERROR: " + rErr.name());
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
