package frc.robot.utils

import edu.wpi.first.math.controller.PIDController
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

/**
 * A class that extends the PIDController to include additional parameters.
 *
 * @property p The proportional gain.
 * @property i The integral gain.
 * @property d The derivative gain.
 * @property v The velocity gain.
 * @property s The static gain.
 * @property g The gravity gain.
 * @throws NullPointerException if v, s, or g are not set and are then accessed.
 */
data class Pingu
    @JvmOverloads
    constructor(
        var p: Double,
        var i: Double,
        var d: Double,
        var v: Double? = null,
        var s: Double? = null,
        var g: Double? = null,
    ) {
        /**
         * Gets the PIDController instance with the current p, i, and d values.
         *
         * @return A PIDController instance.
         */
        val pidController
            get() = PIDController(p, i, d)

        /**
         * Sets the PID values from the given PIDController instance.
         *
         * @param pidController The PIDController instance to get values from.
         */
        fun setPID(pidController: PIDController) {
            p = pidController.p
            i = pidController.i
            d = pidController.d
        }

        /**
         * Sets the proportional gain (p) from a LoggedNetworkNumber.
         *
         * @param p The LoggedNetworkNumber instance to get the value from.
         */
        fun setP(p: LoggedNetworkNumber) {
            this.p = p.get()
        }

        /**
         * Sets the integral gain (i) from a LoggedNetworkNumber.
         *
         * @param i The LoggedNetworkNumber instance to get the value from.
         */
        fun setI(i: LoggedNetworkNumber) {
            this.i = i.get()
        }

        /**
         * Sets the derivative gain (d) from a LoggedNetworkNumber.
         *
         * @param d The LoggedNetworkNumber instance to get the value from.
         */
        fun setD(d: LoggedNetworkNumber) {
            this.d = d.get()
        }

        /**
         * Sets the velocity gain (v) from a LoggedNetworkNumber.
         *
         * @param v The LoggedNetworkNumber instance to get the value from.
         */
        fun setV(v: LoggedNetworkNumber) {
            this.v = v.get()
        }

        /**
         * Sets the static gain (s) from a LoggedNetworkNumber.
         *
         * @param s The LoggedNetworkNumber instance to get the value from.
         */
        fun setS(s: LoggedNetworkNumber) {
            this.s = s.get()
        }

        /**
         * Sets the gravity gain (g) from a LoggedNetworkNumber.
         *
         * @param g The LoggedNetworkNumber instance to get the value from.
         */
        fun setG(g: LoggedNetworkNumber) {
            this.g = g.get()
        }
    }

/**
 * A class that represents a PID controller with network logging capabilities.
 *
 * @property p The proportional gain as a LoggedNetworkNumber.
 * @property i The integral gain as a LoggedNetworkNumber.
 * @property d The derivative gain as a LoggedNetworkNumber.
 * @property v The velocity gain as a LoggedNetworkNumber.
 * @property s The static gain as a LoggedNetworkNumber.
 * @property g The gravity gain as a LoggedNetworkNumber.
 * @throws NullPointerException if v, s, or g are not set and are then accessed.
 */
class NetworkPingu
    @JvmOverloads
    constructor(
        var p: LoggedNetworkNumber,
        var i: LoggedNetworkNumber,
        var d: LoggedNetworkNumber,
        var v: LoggedNetworkNumber? = null,
        var s: LoggedNetworkNumber? = null,
        var g: LoggedNetworkNumber? = null,
    ) : PIDController(p.get(), i.get(), d.get())

/**
 * A data class that represents a MagicPingu with velocity, acceleration, and jerk.
 *
 * @property velocity The velocity of the MagicPingu.
 * @property acceleration The acceleration of the MagicPingu.
 * @property jerk The jerk of the MagicPingu.
 */
data class MagicPingu(
    var velocity: Double,
    var acceleration: Double,
    var jerk: Double,
) {
    /**
     * Sets the velocity of the MagicPingu.
     *
     * @param velocity The new velocity as a LoggedNetworkNumber.
     */
    fun setVelocity(velocity: LoggedNetworkNumber) {
        this.velocity = velocity.get()
    }

    /**
     * Sets the acceleration of the MagicPingu.
     *
     * @param acceleration The new acceleration as a LoggedNetworkNumber.
     */
    fun setAcceleration(acceleration: LoggedNetworkNumber) {
        this.acceleration = acceleration.get()
    }

    /**
     * Sets the jerk of the MagicPingu.
     *
     * @param jerk The new jerk as a LoggedNetworkNumber.
     */
    fun setJerk(jerk: LoggedNetworkNumber) {
        this.jerk = jerk.get()
    }
}
