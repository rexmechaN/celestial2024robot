package com.teamcelestial

import com.revrobotics.*
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.DoublePublisher
import edu.wpi.first.networktables.DoubleSubscriber
import edu.wpi.first.networktables.DoubleTopic
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.TimedRobot
import kotlin.math.max
import kotlin.math.min

class Robot : TimedRobot() {
    /*private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(4);
    private final WPI_VictorSPX leftMaster = new WPI_VictorSPX(2);
    private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(7);
    private final WPI_VictorSPX rightMaster = new WPI_VictorSPX(9);

    private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftSlave);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightSlave);
    private final DifferentialDrive yagsDrive = new DifferentialDrive(leftGroup, rightGroup);

    private final SlewRateLimiter limiter = new SlewRateLimiter(0.8);*/
    var joystick: Joystick = Joystick(0)

    private val leftCim: CANSparkMax = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless)
    private val rightCim: CANSparkMax = CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless)

    private val networkTable: NetworkTableInstance = NetworkTableInstance.getDefault()
    private val desiredMotorSpeedTopic: DoubleTopic = networkTable.getDoubleTopic("/datatable/motorSpeed")
    private val desiredMotorSpeedSubscription: DoubleSubscriber = desiredMotorSpeedTopic.subscribe(0.0)
    private val desiredMotorSpeedPublisher: DoublePublisher = desiredMotorSpeedTopic.publish()

    private var lastRpmPublish: Long = 0
    private val multiplier = 0.0

    private lateinit var leftPid: SparkPIDController
    private lateinit var rightPid: SparkPIDController
    private lateinit var driveTrainPid: PIDController

    var pValue: NetworkValue<Double> = NetworkValue("P", NetworkValueType.kDouble, 0.0)
    var iValue: NetworkValue<Double> = NetworkValue("I", NetworkValueType.kDouble, 0.0)
    var dValue: NetworkValue<Double> = NetworkValue("D", NetworkValueType.kDouble, 0.0)

    private var rpmTarget = 0.0

    private var shooter: Shooter? = null

    override fun robotInit() {
        desiredMotorSpeedPublisher.setDefault(0.0)
        desiredMotorSpeedPublisher.set(0.0)

        leftPid = leftCim.pidController
        rightPid = rightCim.pidController

        shooter = Shooter(listOf(leftPid, rightPid))

        updateDriveTrainPIDController()

        listOf(pValue, iValue, dValue).forEach {
            it.setListener {
                updateDriveTrainPIDController()
            }
        }

        setupPid(leftPid)
        setupPid(rightPid)
    }

    private fun updateDriveTrainPIDController() {
        driveTrainPid = PIDController(pValue.value, iValue.value, dValue.value)
    }

    private fun setupPid(controller: SparkPIDController?) {
        println("Motor PID:")
        println("P: " + controller!!.p)
        println("I: " + controller.i)
        println("D: " + controller.d)
        println("F: " + controller.ff)
        println("Iacc: " + controller.iAccum)
        println("======")
        val kP = 0.000000039
        val kI = 0.00000029
        val kD = 0.0
        val kIz = 0.0
        val kFF = 0.0
        val kMaxOutput = 1.0
        val kMinOutput = -1.0
        val maxRPM = 5700.0
        // set PID coefficients
        controller.setP(kP)
        controller.setI(kI)
        controller.setD(kD)
        controller.setIZone(kIz)
        controller.setFF(kFF)
        controller.setOutputRange(kMinOutput, kMaxOutput)
    }

    override fun robotPeriodic() {}

    override fun autonomousInit() {}

    override fun autonomousPeriodic() {}

    override fun teleopInit() {}

    private var increaseLatch = false
    private var reduceLatch = false
    private var shooterStarted = false
    override fun teleopPeriodic() {
        val increase = joystick.getRawButton(5)
        val reduce = joystick.getRawButton(3)
        val reverse = joystick.getRawButton(12)

        if (increase && reduce) {
            rpmTarget = 0.0
        } else if (increase && !increaseLatch) {
            rpmTarget = rpmTarget + 500
        } else if (reduce && !reduceLatch) {
            rpmTarget = rpmTarget - 500
        }

        increaseLatch = increase
        reduceLatch = reduce

        rpmTarget = min(max(0.0, rpmTarget), 5500.0)

        if (joystick.getRawButton(1) || reverse) {
            if (!shooterStarted) {
                shooter!!.start(2.65, 3.40)
                shooterStarted = true
            }
            //setMotor(reverse ? -rpmTarget : rpmTarget);
        } else {
            if (shooterStarted) {
                shooter!!.stop()
                shooterStarted = false
            }
        }

        shooter!!.tick()

        val leftRpm = leftCim.encoder.velocity
        val rightRpm = rightCim.encoder.velocity

        if (lastRpmPublish + 500 < System.currentTimeMillis()) {
            lastRpmPublish = System.currentTimeMillis()
            //System.out.println("Left RPM: " + leftRpm + " \tRight RPM: " + rightRpm + " \tTarget RPM: " + rpmTarget);
        }
    }

    var reference: Double = -1.0

    fun setMotor(rpm: Double) {
        if (true) return
        if (reference == rpm) return
        reference = rpm
        println("Set motor: $rpm")
        val lErr: REVLibError = leftPid.setReference(rpm, CANSparkBase.ControlType.kVelocity)
        val rErr: REVLibError = rightPid.setReference(rpm, CANSparkBase.ControlType.kVelocity)
        if (lErr != REVLibError.kOk) {
            println("L REVERROR: " + lErr.name)
        }
        if (rErr != REVLibError.kOk) {
            println("R REVERROR: " + rErr.name)
        }
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
