CLASS IMUPreintegrator


FUNCTION initialize(
    gyro_bias,
    accel_bias,
    gyro_noise_covariance,
    accel_noise_covariance
):

    
    Bias estimates
    

    bg <= gyro_bias
    ba <= accel_bias


    
    Preintegrated measurements
    

    ΔR <= Identity 3×3
    Δv <= Zero 3-vector
    Δp <= Zero 3-vector

    Δt <= 0


    
    State uncertainty
    

    Error state:

        δx = [ δθ
               δv
               δp ]

    Σ <= Zero 9×9


    
    IMU noise covariance
    

    Noise:

        n = [ ηg
              ηa ]

    Q <= Zero 6×6

    Q[0:3, 0:3] <= gyro_noise_covariance

    Q[3:6, 3:6] <= accel_noise_covariance


    
    Bias Jacobians
    

    J_R_bg <= Zero 3×3

    J_v_bg <= Zero 3×3
    J_v_ba <= Zero 3×3

    J_p_bg <= Zero 3×3
    J_p_ba <= Zero 3×3



FUNCTION reset():

    
    Preintegrated measurements
    

    ΔR <= Identity 3×3
    Δv <= Zero 3-vector
    Δp <= Zero 3-vector

    Δt <= 0


    
    State uncertainty
    

    Σ <= Zero 9×9


    
    Bias Jacobians
    

    J_R_bg <= Zero 3×3

    J_v_bg <= Zero 3×3
    J_v_ba <= Zero 3×3

    J_p_bg <= Zero 3×3
    J_p_ba <= Zero 3×3



FUNCTION integrate(
    gyro_measurement,
    accel_measurement,
    dt
):


    
    STEP 1 — Save current state
    

    R_k <= copy(ΔR)

    J_R_bg_old <= copy(J_R_bg)

    J_v_bg_old <= copy(J_v_bg)
    J_v_ba_old <= copy(J_v_ba)

    J_p_bg_old <= copy(J_p_bg)
    J_p_ba_old <= copy(J_p_ba)


    
    STEP 2 — Bias correction
    

    ω <= gyro_measurement - bg

    a <= accel_measurement - ba


    
    STEP 3 — Rotation increment
    

    φ <= ω × dt

    ΔR_increment <= Exp([φ]×)

    Jr <= RightJacobian(φ)


    
    STEP 4 — STATE-TRANSITION JACOBIAN F
    

    Error state:

        δx = [ δθ
               δv
               δp ]

    F <= Zero 9×9


    
    Rotation error => rotation error
    

    F[0:3, 0:3]
        <= ΔR_incrementᵀ


    
    Rotation error => velocity error
    

    F[3:6, 0:3]
        <= -R_k [a]× dt


    
    Velocity error => velocity error
    

    F[3:6, 3:6]
        <= I


    
    Rotation error => position error
    

    F[6:9, 0:3]
        <= -1/2 R_k [a]× dt²


    
    Velocity error => position error
    

    F[6:9, 3:6]
        <= I dt


    
    Position error => position error
    

    F[6:9, 6:9]
        <= I


    Therefore:

    F =

    [ ΔR_incrementᵀ       0        0
     -R_k[a]×dt           I        0
     -1/2 R_k[a]×dt²      I dt     I ]


    
    STEP 5 — NOISE JACOBIAN G
    

    Noise state:

        n = [ ηg
              ηa ]

    G <= Zero 9×6


    
    Gyroscope noise => rotation
    

    G[0:3, 0:3]
        <= Jr dt


    
    Accelerometer noise => velocity
    

    G[3:6, 3:6]
        <= R_k dt


    
    Accelerometer noise => position
    

    G[6:9, 3:6]
        <= 1/2 R_k dt²


    Therefore:

    G =

    [ Jr dt        0
        0         R_k dt
        0      1/2 R_k dt² ]


    
    STEP 6 — COVARIANCE PROPAGATION
    

    State uncertainty:

        Σ_k = E[δx_k δx_kᵀ]

    IMU noise covariance:

        Q = E[n_k n_kᵀ]


    Error propagation:

        δx_(k+1)
            =
        F_k δx_k
            +
        G_k n_k


    Therefore:

        Σ_(k+1)
            =
        F_k Σ_k F_kᵀ
            +
        G_k Q G_kᵀ


    
    Existing uncertainty
    

        F_k Σ_k F_kᵀ


    
    New IMU noise
    

        G_k Q G_kᵀ


    
    Final covariance
    

    Σ <=
        F Σ Fᵀ
        +
        G Q Gᵀ


    
    STEP 7 — GYRO BIAS => ROTATION
    

    J_R_bg <=
        ΔR_incrementᵀ
        J_R_bg_old
        -
        Jr dt


    
    STEP 8 — GYRO BIAS => VELOCITY
    

    J_v_bg <=
        J_v_bg_old
        -
        R_k
        [a]×
        J_R_bg_old
        dt


    
    STEP 9 — ACCELEROMETER BIAS => VELOCITY
    

    J_v_ba <=
        J_v_ba_old
        -
        R_k dt


    
    STEP 10 — GYRO BIAS => POSITION
    

    J_p_bg <=
        J_p_bg_old
        +
        J_v_bg_old dt
        -
        1/2
        R_k
        [a]×
        J_R_bg_old
        dt²


    
    STEP 11 — ACCELEROMETER BIAS => POSITION
    

    J_p_ba <=
        J_p_ba_old
        +
        J_v_ba_old dt
        -
        1/2
        R_k dt²


    
    STEP 12 — UPDATE POSITION
    

    Δp <=
        Δp
        +
        Δv dt
        +
        1/2 R_k a dt²


    
    STEP 13 — UPDATE VELOCITY
    

    Δv <=
        Δv
        +
        R_k a dt


    
    STEP 14 — UPDATE ROTATION
    

    ΔR <=
        R_k
        ΔR_increment


    
    STEP 15 — UPDATE TOTAL TIME
    

    Δt <= Δt + dt



FUNCTION get_measurement():

    RETURN

        ΔR
        Δv
        Δp
        Δt



FUNCTION get_covariance():

    RETURN Σ



FUNCTION get_bias_jacobians():

    RETURN

        J_R_bg

        J_v_bg
        J_v_ba

        J_p_bg
        J_p_ba

