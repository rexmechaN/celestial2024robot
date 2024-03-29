package com.teamcelestial.subsystems

import com.revrobotics.*
import com.teamcelestial.math.solver.NumericalSolver
import com.teamcelestial.math.solver.NumericalSolverMode
import com.teamcelestial.math.solver.SolverResult
import com.teamcelestial.math.util.toRadians
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.system.coherence.SubsystemCoherenceDependency
import com.teamcelestial.system.shooter.ShooterCalcResult
import edu.wpi.first.wpilibj2.command.SubsystemBase

import kotlin.math.*

class Shooter : SubsystemBase() {
    private val leftNeo = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless)
    private val rightNeo = CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless)

    private val leftPid = leftNeo.pidController
    private val rightPid = rightNeo.pidController
    private val pidControllers = listOf(leftPid, rightPid)
    private var deploymentAvailabilityDependency: SubsystemCoherenceDependency? = null

    private val encoders = listOf(
        leftNeo.getEncoder(),
        rightNeo.getEncoder()
    )

    init {
        setupPid(leftPid)
        setupPid(rightPid)
    }

    private val flywheelRadius = 0.0504
    private var targetRpm = 0.0
    private var targetTheta = 0.0
    private var startTime = 0L
    private var reference = -1.0
    private var lastRpmPublish = 0L
    private val ballWeight = 0.230
    private val ballFinalSpeedTarget = 0.2
    private val inertiaMultiplier = NetworkValue<Double>("inertia_k", NetworkValueType.kDouble, 0.98)
    private val airResistanceMultiplierValue = NetworkValue<Double>("air_resistance_k", NetworkValueType.kDouble, 0.75)
    private val distanceUnitRpm = NetworkValue<Double>("dist_rpm_k", NetworkValueType.kDouble, 650.0)

    private var airResistanceMultiplier = 0.75
    private var distRpm = 650.0

    override fun periodic() {
        tick()
    }

    private fun tick() {
        setMotor(deploymentAvailabilityDependency?.takeIf {
            it.isReady()
        }?.let {
            targetRpm
        } ?: 0.0)
    }

    fun setTargetRPM(rpm: Double) {
        targetRpm = rpm
    }

    fun start(
        distance: Double,
        height: Double,
        runMotors: Boolean = true,
        thetaOverride: Double? = null,
        rpmOverride: Double? = null,
    ): ShooterCalcResult {
        startTime = System.currentTimeMillis()
        try {
            totalInertia = baseInertia * inertiaMultiplier.value
            distRpm = distanceUnitRpm.value
            airResistanceMultiplier = airResistanceMultiplierValue.value
        } catch (e: Exception) {
            e.printStackTrace()
        }

        NumericalSolver(
            5..90,
            0.5
        ) {
            calculateRpm(distance, height, it)
        }.solveFor(min(4800.0, thetaOverride?.let {
            calculateRpm(distance, height, it)
        } ?: (3400.0 + distance * distRpm)), solverMode = NumericalSolverMode.A_PLUS_PARABOLIC_MINIMUM, toleranceRate = 0.1).let {
            SolverResult(
                x = it.x,
                y = min(5000.0, it.y)
            )
        }.let {
            if (runMotors) setTargetRPM(rpmOverride ?: it.y)
            if (runMotors) targetTheta = it.x
            return ShooterCalcResult(rpm = it.y, theta = it.x).also { shooterCalcResult ->
                println(shooterCalcResult)
            }
        }
    }

    private fun calculateApproximateRpmTargetWithoutFlightData(height: Double): Double {
        return calculateRpmForEnergyTarget((0.5 * ballWeight * ballFinalSpeedTarget.pow(2) + ballWeight * 9.81 * height) * 1.1)
    }

    fun stop() {
        setTargetRPM(0.0)
    }

    private fun calculateAirResistanceMinusV(speed: Double, time: Double): Double {
        return (speed.pow(2) * airResistanceMultiplier * 0.01208955 / ballWeight) * time
    }

    private fun calculateRpmForVelocity(velocity: Double): Double {
        return calculateRpmForEnergyTarget(0.5 * ballWeight * velocity.pow(2))
    }

    private val churroInertia = 4 * 0.0001
    private val inertiaDisc1 = 2.125 * 0.0001
    private val inertiaDisc2 = 1.5 * 0.0001

    private var baseInertia = ((2 * churroInertia) + (6 * inertiaDisc1) + (6 * inertiaDisc2)) * 1.0

    private var totalInertia = ((2 * churroInertia) + (6 * inertiaDisc1) + (6 * inertiaDisc2)) * 1.0

    private val rateOfRpmRetention = 0.7
    private fun calculateRpmForEnergyTarget(energyTarget: Double): Double {
        // Energy is not fully transferred from the flywheel, rateOfRpmRetention is the rate of RPM conserved
        return sqrt((2 * energyTarget) / (totalInertia * (1 - rateOfRpmRetention))) * 60 / (2 * Math.PI)
    }

    fun setMotor(rpm: Double) {
        if (reference == rpm) return
        reference = rpm
        println("Set motor: $rpm")
        pidControllers.forEach {
            it.setReference(-rpm, CANSparkBase.ControlType.kVelocity)
        }
    }

    private fun calculateRpm(distance: Double, height: Double, thetaInDegrees: Double): Double {
        val g = 9.81
        val theta = thetaInDegrees.toRadians()
        val v = (distance / cos(theta)) * sqrt((g) / (2 * (distance * tan(theta) - height)))
        val t = distance / (v * cos(theta))
        // Flywheel speed is not equal to ring speed since the contact happens for a short time
        return calculateRpmForVelocity(v + calculateAirResistanceMinusV(v, t))
    }

    var counter = 0L

    fun atSetpoint(): Boolean {
        if (counter % 100 == 0L) {

        }

        counter++
        return encoders.all {
            abs(targetRpm - abs(it.velocity)) <= 50
        }
    }

    private fun setupPid(controller: SparkPIDController?) {
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

    fun registerDeploymentAvailabilityDependency(dependency: SubsystemCoherenceDependency) {
        deploymentAvailabilityDependency = dependency
    }
}