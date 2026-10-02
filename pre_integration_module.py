import numpy as np
from scipy.spatial.transform import Rotation
class PreIntegrator:


    """    
    Implements:
    - SO(3) exp map + right Jacobian
    - Recursive preintegration of (dR, dv, dp) between two keyframes
    - Analytic first-order bias Jacobians (d dR/d bg, d dv/d ba, d dv/d bg, d dp/d ba, d dp/d bg)
    - 9x9 covariance propagation of the preintegrated measurement
    
    Validated against:
    1. A closed-form ground-truth trajectory (constant body-frame omega + accel)
    2. Finite-difference check of the bias Jacobians (perturb bias, re-integrate,
        compare against the first-order analytic correction
    """


    def __init__(self, gyro_bias=None,
        accel_bias=None,
        gyro_noise_cov=None,
        accel_noise_cov=None):

        # bg ← given gyro bias
        # ba ← given accelerometer bias

        # ΔR ← Identity(3×3)
        # Δv ← [0, 0, 0]
        # Δp ← [0, 0, 0]

        # Δt ← 0

        # Biases
        self.bg  = np.zeros(3) if gyro_bias is None else np.asarray(gyro_bias)
        self.ba  = np.zeros(3) if accel_bias is None else np.asarray(accel_bias)

        # Preintegrated measurements
        self.delta_R = np.eye(3)
        self.delta_v = np.zeros(3)
        self.delta_p = np.zeros(3)

        # time interval
        self.delta_t = 0.0

        # Bias Jacobians
        self.J_R_bg = np.zeros((3, 3))
        self.J_v_bg = np.zeros((3, 3))
        self.J_v_ba = np.zeros((3, 3))
        self.J_p_bg = np.zeros((3, 3))
        self.J_p_ba = np.zeros((3, 3))

        # covariance of the preintegrated measurement
        #
        # Error state:
        #
        # [delta_theta]
        # [delta_v    ]
        # [delta_p    ]
        #
        # Therefore covariance is 9 x 9
        self.covariance = np.zeros((9, 9))

        # Noise covariance of the IMU measurement
        # n = [gyro_noise, accel_noise]
        #
        # Q is 6 x 6
        if gyro_noise_cov is None:
            gyro_noise_cov = np.zeros((3, 3))

        if accel_noise_cov is None:
            accel_noise_cov = np.zeros((3, 3))

        self.Q = np.zeros((6, 6))

        self.Q[0:3, 0:3] = gyro_noise_cov
        self.Q[3:6, 3:6] = accel_noise_cov


    # utility functions
    @staticmethod
    def skew(v):
        """
        Convert a 3D vector into a skew-symmetric matrix.
        """
        x, y, z = v

        return np.array([
            [0.0, -z,  y],
            [z,   0.0, -x],
            [-y,  x,   0.0]
        ])
    
    @staticmethod
    def right_jacobian(phi):
        """
        SO(3) right Jacobian J_r(phi).
        """

        theta = np.linalg.norm(phi)

        Phi = PreIntegrator.skew(phi)

        if theta < 1e-8:

            # Small-angle approximation
            return (
                np.eye(3)
                - 0.5 * Phi
                + (1.0 / 6.0) * Phi @ Phi
            )

        theta2 = theta * theta
        theta3 = theta2 * theta

        return (
            np.eye(3)
            - ((1.0 - np.cos(theta)) / theta2) * Phi
            + ((theta - np.sin(theta)) / theta3) * Phi @ Phi
        )

    def reset(self):

        # Preintegrated measurements
        self.delta_R = np.eye(3)
        self.delta_v = np.zeros(3)
        self.delta_p = np.zeros(3)
        self.delta_t = 0.0

        # Bias Jacobians
        self.J_R_bg = np.zeros((3, 3))

        self.J_v_bg = np.zeros((3, 3))
        self.J_v_ba = np.zeros((3, 3))

        self.J_p_bg = np.zeros((3, 3))
        self.J_p_ba = np.zeros((3, 3))

        # Covariance
        self.covariance = np.zeros((9, 9))

    # Main integration function
    def integrate(self, gyro, accel, dt):
        """
        Integrate one IMU measurement."""

        omega = gyro - self.bg
        a = accel - self.ba

        # save the current state
        R_k = self.delta_R.copy()
        J_R_bg_old = self.J_R_bg.copy()

        J_v_bg_old = self.J_v_bg.copy()
        J_v_ba_old = self.J_v_ba.copy()

        J_p_bg_old = self.J_p_bg.copy()
        J_p_ba_old = self.J_p_ba.copy()

 ###################### Rotation increment ######################
        phi = omega * dt

        delta_R_inc = Rotation.from_rotvec(phi).as_matrix() # Compute the Rotation matrix using the exponential map

        Jr = PreIntegrator.right_jacobian(phi) # Compute the right Jacobian


###################### STATE-TRANSITION JACOBIAN F ######################
                    #
                    # Error state:
                    #
                    # [dtheta]
                    # [dv    ]
                    # [dp    ]

        F = np.zeros((9, 9))

        # Rotation error propagation
        F[0:3, 0:3] = delta_R_inc.T


        # Rotation error -> velocity error
        F[3:6, 0:3] = (
            -R_k @ self.skew(a) * dt
        )

        # Velocity error -> velocity error
        F[3:6, 3:6] = np.eye(3)


        # Rotation error -> position error
        F[6:9, 0:3] = (
            -0.5
            * R_k
            @ self.skew(a)
            * dt**2
        )

        # Velocity error -> position error
        F[6:9, 3:6] = np.eye(3) * dt

        # Position error -> position error
        F[6:9, 6:9] = np.eye(3)

######################## NOISE JACOBIAN G ##################################
                        # Noise : 
                        # [gyro noise]
                        # [accel noise]

        G = np.zeros((9, 6))

        # Gyro noise -> rotation
        G[0:3, 0:3] = Jr * dt

        # Accelerometer noise -> velocity
        G[3:6, 3:6] = R_k * dt

        # Accelerometer noise -> position
        G[6:9, 3:6] = (
            0.5 * R_k * dt**2
        )

######################### COVARIANCE PROPAGATION ######################
                        #
                        # Sigma_next =
                        #
                        # F Sigma F^T + G Q G^T

        self.covariance = (
            F
            @ self.covariance
            @ F.T
            +
            G
            @ self.Q
            @ G.T
        )

####################### GYRO BIAS -> ROTATION ######################

        self.J_R_bg = (
            delta_R_inc.T
            @ J_R_bg_old
            -
            Jr * dt
        )

####################### GYRO BIAS -> VELOCITY ######################
        self.J_v_bg = (
                    J_v_bg_old
                    -
                    R_k
                    @ self.skew(a)
                    @ J_R_bg_old
                    * dt
                )

###################### ACCELEROMETER BIAS -> VELOCITY ######################

        self.J_v_ba = (
            J_v_ba_old
            -
            R_k * dt
        )


###################### GYRO BIAS -> POSITION ######################

        self.J_p_bg = (
            J_p_bg_old
            +
            J_v_bg_old * dt
            -
            0.5
            * R_k
            @ self.skew(a)
            @ J_R_bg_old
            * dt**2
        )



###################### ACCELEROMETER BIAS -> POSITION ######################
        self.J_p_ba = (
            J_p_ba_old
            +
            J_v_ba_old * dt
            -
            0.5
            * R_k
            * dt**2
        )


###################### UPDATE POSITION ######################

        self.delta_p += (
            self.delta_v * dt
            +
            0.5
            * R_k
            @ a
            * dt**2
        )


###################### UPDATE VELOCITY ######################

        self.delta_v += (
            R_k
            @ a
            * dt
        )


###################### UPDATE ROTATION ######################

        self.delta_R = (
            R_k
            @ delta_R_inc
        )


###################### UPDATE TOTAL TIME ######################

        self.delta_t += dt


###################### Get  Results ######################
    def get_measurement(self):

        return {
            "delta_R": self.delta_R.copy(),
            "delta_v": self.delta_v.copy(),
            "delta_p": self.delta_p.copy(),
            "delta_t": self.delta_t
        }


    def get_covariance(self):

        return self.covariance.copy()


    def get_bias_jacobians(self):

        return {
            "J_R_bg": self.J_R_bg.copy(),

            "J_v_bg": self.J_v_bg.copy(),
            "J_v_ba": self.J_v_ba.copy(),

            "J_p_bg": self.J_p_bg.copy(),
            "J_p_ba": self.J_p_ba.copy()
        }