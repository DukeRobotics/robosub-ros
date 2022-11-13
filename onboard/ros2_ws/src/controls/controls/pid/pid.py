#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Empty
import math


class PID(Node):

    NODE_NAME = 'pid'
    DEFAULT_ENABLE_SERVICE = 'controls/pid/enable'
    DEFAULT_RESET_SERVICE = 'controls/pid/reset'
    DEFAULT_EFFORT_TOPIC = 'controls/pid/effort'
    DEFAULT_SETPOINT_TOPIC = 'controls/pid/setpoint'
    DEFAULT_STATE_TOPIC = 'controls/pid/state'
    DEFAULT_EFFORT_LIMIT = 1.0
    DEFAULT_WINDUP_LIMIT = 10.0
    DEFAULT_FREQ = 30
    DEFAULT_KP = 1.0
    DEFAULT_KI = 0.0
    DEFAULT_KD = 0.0
    C = 1.0  # LPF with cutoff = 1/4 of sampling rate

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self._reset()

        # Parameters that support reconfiguration
        self.p_param = self.declare_parameter('Kp', self.DEFAULT_KP)
        self.i_param = self.declare_parameter('Ki', self.DEFAULT_KI)
        self.d_param = self.declare_parameter('Kd', self.DEFAULT_KD)

        # Parameters that can't be reconfigured
        self.upper_limit = self.declare_parameter('upper_limit', self.DEFAULT_EFFORT_LIMIT).value
        self.lower_limit = self.declare_parameter('lower_limit', -self.DEFAULT_EFFORT_LIMIT).value
        self.windup_limit = self.declare_parameter('windup_limit', self.DEFAULT_WINDUP_LIMIT).value
        target_freq = self.declare_parameter('target_frequency', self.DEFAULT_FREQ).value
        effort_topic = self.declare_parameter('effort', self.DEFAULT_EFFORT_TOPIC).value
        setpoint_topic = self.declare_parameter('setpoint', self.DEFAULT_SETPOINT_TOPIC).value
        state_topic = self.declare_parameter('state', self.DEFAULT_STATE_TOPIC).value
        enable_srv = self.declare_parameter('enable_service', self.DEFAULT_ENABLE_SERVICE).value
        reset_srv = self.declare_parameter('reset_service', self.DEFAULT_RESET_SERVICE).value

        self.create_service(SetBool, enable_srv, self._handle_enable_pid)
        self.create_service(Empty, reset_srv, self._handle_reset)

        self.pub_effort = self.create_publisher(Float64, effort_topic, 10)
        self.create_subscription(Float64, setpoint_topic, self._on_setpoint_received, 10)
        self.create_subscription(Float64, state_topic, self._on_state_received, 10)

        self.timer = self.create_timer(1/target_freq, self.tick)
        self.timer.cancel()  # Disabled by default

    def _on_setpoint_received(self, setpoint):
        self.setpoint = setpoint.data
        self.recalc = True

    def _on_state_received(self, state):
        self.state = state.data
        self.recalc = True

    def _handle_enable_pid(self, req, res):
        """ Callback for exposed enable/disable service """
        if req.data:
            self.timer.reset()
            res.message = f'Successfully enabled pid'
        else:
            self.timer.cancel()
            res.message = f'Successfully disabled pid'
        res.success = True
        return res

    def _handle_reset(self, req, res):
        """ Callback for exposed reset service """
        self._reset()
        return res

    def _reset(self):
        """ Reset the state of the PID loop """
        self.setpoint = None
        self.state = None
        self.warned = False
        self.prev_time = 0.0
        self.i_error = 0.0
        self.p_error = [0.0, 0.0, 0.0]  # We keep track of last 3 errors for filtering
        self.d_error = [0.0, 0.0, 0.0]
        self.filtered_p_error = [0.0, 0.0, 0.0]
        self.filtered_d_error = [0.0, 0.0, 0.0]

    def _p(self):
        """ Get KP constant from ROS param """
        return self.p_param.value

    def _i(self):
        """ Get KI constant from ROS param """
        return self.i_param.value

    def _d(self):
        """ Get KD constant from ROS param """
        return self.d_param.value

    def tick(self):
        if self.recalc:
            if self._check_invalid_data():
                return
            # Propagate errors by one step
            self.p_error[2] = self.p_error[1]
            self.p_error[1] = self.p_error[0]
            self.p_error[0] = self.setpoint - self.state

            dt = self._calc_timestep()
            if dt == 0 or math.isnan(dt) or math.isinf(dt):
                return

            p_term = self._calc_p_term()
            i_term = self._calc_i_term(dt)
            d_term = self._calc_d_term(dt)
            effort = self._normalize_effort(p_term + i_term + d_term)

            self.pub_effort.publish(Float64(data=effort))
        self.recalc = False

    def _calc_p_term(self):
        self._lpf(self.p_error, self.filtered_p_error)
        return self._p() * self.p_error[0]

    def _calc_i_term(self, dt):
        self.i_error += (dt * self.p_error[0])
        i_term = self._i() * self.i_error
        i_term = min(i_term, self.windup_limit)
        i_term = max(i_term, -self.windup_limit)
        return i_term

    def _calc_d_term(self, dt):
        self.d_error[2] = self.d_error[1]
        self.d_error[1] = self.d_error[0]
        self.d_error[0] = (self.p_error[0] - self.p_error[1]) / dt

        self._lpf(self.d_error, self.filtered_d_error)
        d_term = self._d() * self.filtered_d_error[0]
        return d_term

    def _lpf(self, error, filtered_error):
        """ Calculate filtered error per 
        https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html and 
        https://bitbucket.org/AndyZe/pid/src/master/src/pid.cpp """
        filtered_error[2] = filtered_error[1]
        filtered_error[1] = filtered_error[0]
        h = (1/((self.C**2) + (1.414 * self.C) + 1))
        filtered_error[0] = h * (error[2] + (2 * error[1]) + error[0] -
                                 ((self.C**2) - 1.414 * self.C + 1) * filtered_error[2] -
                                 (-2 * (self.C**2) + 2) * filtered_error[1])

    def _normalize_effort(self, effort):
        """ Normalize control effort to upper and lower limit. 
        TODO: This is how the old pid package did it, but this feels wrong. """
        normalized_effort = min(effort, self.upper_limit)
        normalized_effort = max(normalized_effort, self.lower_limit)
        return normalized_effort

    def _check_invalid_data(self):
        """ Check if setpoint and state are received """
        if not self.warned:
            if self.setpoint == None:
                self.get_logger().warn(f"Waiting for first setpoint message")
            if self.state == None:
                self.get_logger().warn(f"Waiting for first state message")
        return (self.setpoint == None) or (self.state == None)

    def _calc_timestep(self):
        """ Calculate the timestep between tick updates """
        msg = self.get_clock().now().to_msg()
        now = msg.sec + msg.nanosec * 1e-9
        if self.prev_time == 0:
            self.prev_time = now
            return 0
        dt = now - self.prev_time
        self.prev_time = now
        return dt


def main(args=None):
    try:
        rclpy.init(args=args)
        pid = PID()
        rclpy.spin(pid)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        pid.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
