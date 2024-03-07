package com.teamcelestial.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.ReplanningConfig
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkPIDController
import com.teamcelestial.constant.DrivetrainConstant
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.*
import edu.wpi.first.units.MutableMeasure.mutable
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import kotlin.math.abs


object Drivetrain: SubsystemBase() {
    private val leftSlave = CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless)
    private val leftMaster = CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless)
    private val rightSlave = CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless)
    private val rightMaster = CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless)

    private val robotDrive =  DifferentialDrive(leftMaster, rightMaster)

    private val leftEncoder = Encoder(8,9, false, CounterBase.EncodingType.k1X)
    private val rightEncoder = CANcoder(5)
    private val navx2 = AHRS(SPI.Port.kMXP)

    /*    private val leftPid = SparkPIDController(0.36066, 0.0, 0.0)
    private val rightPid = SparkPIDController(0.37119, 0.0, 0.0)*/
    private val leftPid = leftMaster.pidController
    private val rightPid = rightMaster.pidController

    private val kinematics = DifferentialDriveKinematics(0.66)
    private val odometry = DifferentialDriveOdometry(
        navx2.rotation2d,
        leftEncoderMeters,
        rightEncoderMeters
    )

    private val m_appliedVoltage: MutableMeasure<Voltage> = mutable(Volts.of(0.0))

    private val m_distance: MutableMeasure<Distance> = mutable(Meters.of(0.0))

    private val m_velocity: MutableMeasure<Velocity<Distance>> = mutable(MetersPerSecond.of(0.0))

    private val field = Field2d()

    private val routine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            {
                leftMaster.setVoltage(it.`in`(Volts))
                rightMaster.setVoltage(it.`in`(Volts))
            },
            {
                it.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            leftMaster.appliedOutput * leftMaster.busVoltage, Volts
                        )
                    )
                    .linearPosition(m_distance.mut_replace(leftEncoderMeters, Meters))
                    .linearVelocity(m_velocity.mut_replace(leftEncoder.rate, MetersPerSecond))

                it.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightMaster.appliedOutput * rightMaster.busVoltage, Volts))
                    .linearPosition(m_distance.mut_replace(rightEncoderMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(rightEncoder.velocity.valueAsDouble * DrivetrainConstant.WHEEL_CIRCUMFERENCE, MetersPerSecond));
            },
            this
        )
    )

    private val leftEncoderMeters
        get() = leftEncoder.distance

    private val rightEncoderMeters
        get() = rightEncoder.position.valueAsDouble * DrivetrainConstant.WHEEL_CIRCUMFERENCE


    init {
        updateDriveTrainPIDController()
        leftEncoder.distancePerPulse = DrivetrainConstant.WHEEL_CIRCUMFERENCE * (1.0 / DrivetrainConstant.LEFT_ENCODER_CPR)
        configureMotors()
        resetSensors()

        AutoBuilder.configureRamsete(
            ::getPose,
            ::resetOdometry,
            ::getCurrentSpeeds,
            ::pidDrive,
            ReplanningConfig(),
            {
                DriverStation.getAlliance().isPresent &&
                DriverStation.getAlliance().get() == Alliance.Red
            },
            this
        )
    }

    override fun periodic() {
        updateOdometryWithSensorValues()
        SmartDashboard.putNumber("Left Encoder", leftEncoderMeters)
        SmartDashboard.putNumber("Right Encoder", rightEncoderMeters)
        SmartDashboard.putNumber("Left Velocity", leftEncoder.rate)
        SmartDashboard.putNumber("Right Velocity", rightEncoder.velocity.valueAsDouble * DrivetrainConstant.WHEEL_CIRCUMFERENCE)
    }

    fun drive(x: Double, y: Double) {
        val steer = (x * 1.0 - (abs(y) * x * 0.6)) * 0.8
        robotDrive.arcadeDrive(y, steer)
    }

    fun pidDrive(speeds: ChassisSpeeds) {
        println(speeds)
        val diffSpeeds = kinematics.toWheelSpeeds(speeds)
        //leftPid.setReference(diffSpeeds.leftMetersPerSecond, CANSparkBase.ControlType.kVelocity)
        //rightPid.setReference(diffSpeeds.rightMetersPerSecond, CANSparkBase.ControlType.kVelocity)
    }

    fun getPose(): Pose2d {
        return odometry.poseMeters
    }

    fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            navx2.rotation2d,
            leftEncoderMeters,
            rightEncoderMeters,
            pose
        )
    }

    fun getWheelSpeeds(): DifferentialDriveWheelSpeeds {
        return DifferentialDriveWheelSpeeds(leftEncoder.rate, rightEncoder.velocity.valueAsDouble * DrivetrainConstant.WHEEL_CIRCUMFERENCE)
    }

    fun getCurrentSpeeds(): ChassisSpeeds {
        return kinematics.toChassisSpeeds(getWheelSpeeds())
    }

    private fun updateOdometryWithSensorValues() {
        odometry.update(navx2.rotation2d, leftEncoderMeters, rightEncoderMeters);
    }

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
        return routine.quasistatic(direction)
    }

    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
        return routine.dynamic(direction)
    }

    private fun configureMotors() {
        leftMaster.setIdleMode(CANSparkBase.IdleMode.kBrake)
        leftSlave.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rightMaster.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rightSlave.setIdleMode(CANSparkBase.IdleMode.kBrake)

        rightMaster.inverted = true
        rightSlave.inverted = true

        leftSlave.follow(leftMaster)
        rightSlave.follow(rightMaster)
    }

    private fun resetSensors() {
        leftEncoder.reset()
        rightEncoder.setPosition(0.0)
        navx2.reset()
    }


    private fun updateDriveTrainPIDController() {
        leftPid.p = 0.36066
        leftPid.i = 0.0
        leftPid.d = 0.0
        leftPid.ff = 0.0

        rightPid.p = 0.36066
        rightPid.i = 0.0
        rightPid.d = 0.0
        rightPid.ff = 0.0
    }
}