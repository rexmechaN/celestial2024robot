package com.teamcelestial.math.control

import edu.wpi.first.math.MathSharedStore
import edu.wpi.first.math.MathUsageId
import edu.wpi.first.math.MathUtil
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import java.lang.IllegalArgumentException
import kotlin.math.pow


class PIDXController @JvmOverloads constructor(
    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kp The proportional coefficient. Must be &gt;= 0.
     */
    // Factor for "proportional" control
    var p: Double,
    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki The integral coefficient. Must be &gt;= 0.
     */
    // Factor for "integral" control
    var i: Double, kd: Double, period: Double = 0.02
) :
    Sendable, AutoCloseable {
    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd The differential coefficient. Must be &gt;= 0.
     */
    // Factor for "derivative" control
    var d: Double

    // The error range where "integral" control applies
    private var m_iZone = Double.POSITIVE_INFINITY

    /**
     * Returns the period of this controller.
     *
     * @return the period of the controller.
     */
    // The period (in seconds) of the loop that calls the controller
    val period: Double
    private var m_maximumIntegral = 1.0
    private var m_minimumIntegral = -1.0
    private var m_maximumInput = 0.0
    private var m_minimumInput = 0.0

    /**
     * Returns true if continuous input is enabled.
     *
     * @return True if continuous input is enabled.
     */
    // Do the endpoints wrap around? e.g. Absolute encoder
    var isContinuousInputEnabled = false
        private set

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    // The error at the time of the most recent call to calculate()
    var positionError = 0.0
        private set

    /**
     * Returns the velocity error.
     *
     * @return The velocity error.
     */
    var velocityError = 0.0
        private set

    // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
    private var m_prevError = 0.0

    // The sum of the errors for use in the integral calc
    private var m_totalError = 0.0

    /**
     * Returns the position tolerance of this controller.
     *
     * @return the position tolerance of the controller.
     */
    // The error that is considered at setpoint.
    var positionTolerance = 0.05
        private set

    /**
     * Returns the velocity tolerance of this controller.
     *
     * @return the velocity tolerance of the controller.
     */
    var velocityTolerance = Double.POSITIVE_INFINITY
        private set
    private var m_setpoint = 0.0
    private var m_measurement = 0.0
    private var m_haveMeasurement = false
    private var m_haveSetpoint = false
    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd.
     *
     * @param p The proportional coefficient.
     * @param i The integral coefficient.
     * @param kd The derivative coefficient.
     * @param period The period between controller updates in seconds.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
     * 0.02 seconds.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    init {
        i = i
        d = kd
        require(!(p < 0.0)) { "Kp must be a non-negative number!" }
        if (i < 0.0) {
            throw IllegalArgumentException("Ki must be a non-negative number!")
        }
        if (kd < 0.0) {
            throw IllegalArgumentException("Kd must be a non-negative number!")
        }
        if (period <= 0.0) {
            throw IllegalArgumentException("Controller period must be a positive number!")
        }
        this.period = period
        instances++
        SendableRegistry.addLW(this, "PIDController", instances)
        MathSharedStore.reportUsage(MathUsageId.kController_PIDController2, instances)
    }

    override fun close() {
        SendableRegistry.remove(this)
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     *
     * Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    fun setPID(kp: Double, ki: Double, kd: Double) {
        p = kp
        i = ki
        d = kd
    }

    var iZone: Double
        /**
         * Get the IZone range.
         *
         * @return Maximum magnitude of error to allow integral control.
         */
        get() = m_iZone
        /**
         * Sets the IZone range. When the absolute value of the position error is greater than IZone, the
         * total accumulated error will reset to zero, disabling integral gain until the absolute value of
         * the position error is less than IZone. This is used to prevent integral windup. Must be
         * non-negative. Passing a value of zero will effectively disable integral gain. Passing a value
         * of [Double.POSITIVE_INFINITY] disables IZone functionality.
         *
         * @param iZone Maximum magnitude of error to allow integral control.
         * @throws IllegalArgumentException if iZone &lt; 0
         */
        set(iZone) {
            if (iZone < 0) {
                throw IllegalArgumentException("IZone must be a non-negative number!")
            }
            m_iZone = iZone
        }
    var setpoint: Double
        /**
         * Returns the current setpoint of the PIDController.
         *
         * @return The current setpoint.
         */
        get() = m_setpoint
        /**
         * Sets the setpoint for the PIDController.
         *
         * @param setpoint The desired setpoint.
         */
        set(setpoint) {
            m_setpoint = setpoint
            m_haveSetpoint = true
            if (isContinuousInputEnabled) {
                val errorBound = (m_maximumInput - m_minimumInput) / 2.0
                positionError = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound)
            } else {
                positionError = m_setpoint - m_measurement
            }
            velocityError = (positionError - m_prevError) / period
        }

    /**
     * Returns true if the error is within the tolerance of the setpoint.
     *
     *
     * This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    fun atSetpoint(): Boolean {
        return ((m_haveMeasurement
                && m_haveSetpoint && Math.abs(positionError) < positionTolerance) && Math.abs(
            velocityError
        ) < velocityTolerance)
    }

    /**
     * Enables continuous input.
     *
     *
     * Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    fun enableContinuousInput(minimumInput: Double, maximumInput: Double) {
        isContinuousInputEnabled = true
        m_minimumInput = minimumInput
        m_maximumInput = maximumInput
    }

    /** Disables continuous input.  */
    fun disableContinuousInput() {
        isContinuousInputEnabled = false
    }

    /**
     * Sets the minimum and maximum values for the integrator.
     *
     *
     * When the cap is reached, the integrator value is added to the controller output rather than
     * the integrator value times the integral gain.
     *
     * @param minimumIntegral The minimum value of the integrator.
     * @param maximumIntegral The maximum value of the integrator.
     */
    fun setIntegratorRange(minimumIntegral: Double, maximumIntegral: Double) {
        m_minimumIntegral = minimumIntegral
        m_maximumIntegral = maximumIntegral
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY)
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
        this.positionTolerance = positionTolerance
        this.velocityTolerance = velocityTolerance
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint The new setpoint of the controller.
     * @return The next controller output.
     */
    fun calculate(measurement: Double, setpoint: Double): Double {
        m_setpoint = setpoint
        m_haveSetpoint = true
        return calculate(measurement)
    }

    private var lastLog: Long = 0L
    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    fun calculate(measurement: Double): Double {
        m_measurement = measurement
        m_prevError = positionError
        m_haveMeasurement = true
        if (isContinuousInputEnabled) {
            val errorBound = (m_maximumInput - m_minimumInput) / 2.0
            positionError = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound)
        } else {
            positionError = m_setpoint - m_measurement
        }
        velocityError = (positionError - m_prevError) / period

        // If the absolute value of the position error is greater than IZone, reset the total error
        if (Math.abs(positionError) > m_iZone) {
            m_totalError = 0.0
        } else if (i != 0.0) {
            m_totalError = MathUtil.clamp(
                m_totalError + positionError * period,
                m_minimumIntegral / i,
                m_maximumIntegral / i
            )
        }
        return p * (positionError.pow(1.25)) + i * m_totalError + d * velocityError
    }

    /** Resets the previous error and the integral term.  */
    fun reset() {
        positionError = 0.0
        m_prevError = 0.0
        m_totalError = 0.0
        velocityError = 0.0
        m_haveMeasurement = false
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("PIDController")
        builder.addDoubleProperty("p", { p }, { kp: Double -> p = kp })
        builder.addDoubleProperty("i", { i }, { ki: Double -> i = ki })
        builder.addDoubleProperty("d", { d }, { kd: Double -> d = kd })
        builder.addDoubleProperty(
            "izone", { iZone },
            { toSet: Double ->
                try {
                    iZone = toSet
                } catch (e: IllegalArgumentException) {
                    MathSharedStore.reportError("IZone must be a non-negative number!", e.getStackTrace())
                }
            })
        builder.addDoubleProperty("setpoint", { setpoint }, { setpoint: Double ->
            this.setpoint = setpoint
        })
    }

    companion object {
        private var instances = 0
    }
}

