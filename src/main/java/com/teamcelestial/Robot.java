package com.teamcelestial;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.TimedRobot;

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

    private final CANSparkMax leftCim = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightCim = new CANSparkMax(16, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
    private final DoubleTopic desiredMotorSpeedTopic = networkTable.getDoubleTopic("/datatable/motorSpeed");
    private final DoubleSubscriber desiredMotorSpeedSubscription = desiredMotorSpeedTopic.subscribe(0);
    private final DoublePublisher desiredMotorSpeedPublisher = desiredMotorSpeedTopic.publish();

    @Override
    public void robotInit()
    {
        desiredMotorSpeedPublisher.setDefault(0);
        desiredMotorSpeedPublisher.set(0);
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        System.out.println(desiredMotorSpeedSubscription.get());
        leftCim.set(desiredMotorSpeedSubscription.get());
        rightCim.set(-desiredMotorSpeedSubscription.get());
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
