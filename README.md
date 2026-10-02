                    IMU measurement
                           │
             ┌─────────────┴─────────────┐
             │                           │
       Bias correction              Noise
             │                           │
             ↓                           ↓
       ω = gyro - bg                 ηg, ηa
       a = accel - ba                    │
             │                           │
             ├──────────────┐            │
             ↓              ↓            ↓
          ΔR  Δv  Δp        F            G
             │              │            │
             │              │            │
             │              └──────┬─────┘
             │                     │
             │                     ↓
             │              State uncertainty
             │                     │
             │                  δxk
             │                     │
             │              δxk+1 = Fk δxk
             │                       + Gk nk
             │                     │
             │                     ↓
             │                    Σk
             │                     │
             │                     │
             │          Qk = E[nk nkᵀ]
             │                     │
             │                     ↓
             │          ┌────────────────────┐
             │          │                    │
             │          ↓                    ↓
             │      Fk Σk Fkᵀ          Gk Qk Gkᵀ
             │          │                    │
             │          └────────┬───────────┘
             │                   ↓
             │          Σk+1 = FkΣkFkᵀ
             │                  + GkQkGkᵀ
             │
             └──────────────┐
                            ↓
                    Bias Jacobians
                            │
             ┌──────────────┼──────────────┐
             ↓              ↓              ↓
          J_R_bg        J_v_bg/J_v_ba   J_p_bg/J_p_ba

