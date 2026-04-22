# SuspensionModelBase Phase-1 MVP

## 1. Model Goal / 模型目标
- `SuspensionModelBase.slx` is a standalone full-vehicle vertical suspension base model for the yaw-dynamics main flow.
- Phase 1 is intentionally limited to signal flow + LUT force elements.
- The model provides four-corner wheel loads, body heave/pitch/roll, and basic suspension states without introducing suspension geometry kinematics, tire lateral forces, or Simscape Multibody.

## 2. Boundary With HalfCarModel / 与 HalfCarModel 的边界
- Reused from `Vertical Dynamics`:
  - `Functions/SetSpringCurve.m`
  - `Functions/SetDamperClick.m`
  - `SpringTable.xlsx`
  - `DamperTable.xlsx`
  - The existing tire vertical stiffness/damping expression style
- Not reused:
  - The half-car pitch-plane topology
  - The front/rear two-channel model architecture
  - The old output interface focused on accelerations and two travels
  - The shaker-rig style default excitation concept

## 3. States / 状态量
- Sprung mass:
  - `z_s`, `dz_s`
  - `theta`, `dtheta`
  - `phi`, `dphi`
- Unsprung mass:
  - `z_u_FL`, `dz_u_FL`
  - `z_u_FR`, `dz_u_FR`
  - `z_u_RL`, `dz_u_RL`
  - `z_u_RR`, `dz_u_RR`

This is a 7-DOF mechanical model, represented as 14 first-order states.

## 4. Sign Convention / 符号约定
- `z` positive upward
- `theta` positive nose-up
- `phi` positive left side up / right side down
- `wheel_travel = z_u - z_body_corner`
- `damper_travel = motionRatio * wheel_travel`
- Positive `wheel_travel` is treated as bump / compression
- `aero_Fz_front` and `aero_Fz_rear` are axle-level downforce inputs, positive downward

## 5. Inputs / 输入
- `z_road_FL`
- `z_road_FR`
- `z_road_RL`
- `z_road_RR`
- `ax`
- `ay`
- `aero_Fz_front`
- `aero_Fz_rear`

Phase-1 handling of longitudinal/lateral acceleration:
- `ax` enters the body pitch equation as an equivalent inertial moment:
  - `M_pitch_inertial = -m_s * CGh * ax`
- `ay` enters the body roll equation as an equivalent inertial moment:
  - `M_roll_inertial = +m_s * CGh * ay`

## 6. Outputs / 输出
- `Fz_FL`, `Fz_FR`, `Fz_RL`, `Fz_RR`
- `heave`, `pitch`, `roll`
- `wheel_travel_FL`, `wheel_travel_FR`, `wheel_travel_RL`, `wheel_travel_RR`
- `damper_travel_FL`, `damper_travel_FR`, `damper_travel_RL`, `damper_travel_RR`
- `damper_vel_FL`, `damper_vel_FR`, `damper_vel_RL`, `damper_vel_RR`

## 7. Force Model / 力学表达
For each corner:
- `F_susp = F_spring + F_damper + F_arb`
- `F_tire = k_t * (z_road - z_u) + c_t * (dz_road - dz_u)`
- `m_u * z_u_ddot = F_tire - F_susp - m_u * g`

Sprung body:
- `m_s * z_s_ddot = sum(F_susp_corner) - m_s * g - aero_Fz_front - aero_Fz_rear`
- Pitch and roll are solved with small-angle rigid-body moments.

## 8. ARB Convention / 防倾杆符号约定
Phase 1 uses an equivalent linear wheel-side ARB:
- Front axle:
  - `delta_front = wheel_travel_FL - wheel_travel_FR`
  - `F_ARB_FL = +k_arb_front * delta_front / 2`
  - `F_ARB_FR = -k_arb_front * delta_front / 2`
- Rear axle follows the same pattern.

Interpretation:
- The ARB force is added on the body side of the suspension force path.
- The wheel-side reaction is equal magnitude and opposite direction through the unsprung equation.

## 9. Current Assumptions / 当前假设
- Left/right suspension parameters are symmetric on each axle.
- `CGx` is defined as front-axle-to-CG distance in meters.
- The phase-1 state reference is the unloaded geometric zero, not static ride-height equilibrium.
- The spring/damper LUTs are interpreted as wheel-side equivalent force curves.
- This LUT assumption is inherited from the existing helper workflow:
  - the helper scales force by `motionRatio^2`
  - the helper does not rescale LUT displacement/velocity axes
- No camber, toe, bump steer, pushrod kinematics, bump stop, droop stop, or hard-stop modeling.
- No tire lateral/longitudinal force generation is included here.
- Aero is limited to front/rear axle vertical load inputs.

## 10. Parameters / 参数口径
`get_suspension_default_params.m` returns `car` with SI units:
- `car.sprungMass`
- `car.pitchInertia`
- `car.rollInertia`
- `car.unsprungMassFront`
- `car.unsprungMassRear`
- `car.wheelbase`
- `car.trackFront`
- `car.trackRear`
- `car.CGx`
- `car.CGh`
- `car.frontTireStiffness`
- `car.frontTireDamping`
- `car.rearTireStiffness`
- `car.rearTireDamping`
- `car.frontARBStiffness`
- `car.rearARBStiffness`
- `car.frontMotionRatio`
- `car.rearMotionRatio`
- reserved: `car.aeroBalance`, `car.frontRCHeight`, `car.rearRCHeight`

## 11. Curve Preparation / 曲线准备
`prepare_suspension_curves.m` wraps the legacy Excel pipeline:
- front spring default: `linear_350`
- rear spring default: `linear_300`
- damper sheet default: `Multimatic_DSSV_VC01`
- default compression/rebound click: `6 / 6`

Current unit conversions inherited from the existing helpers:
- spring displacement from mm to m
- spring force from lbf-equivalent table value to N via existing sheet values
- damper velocity from mm/s to m/s
- damper force kept from the legacy table and scaled by `motionRatio^2`

## 12. Logging / 日志输出
- All top-level outputs are logged into `logsout`.
- Logging names match the top-level output names exactly.
- `run_suspension_standalone.m` reads `logsout` directly for plotting and summary checks.

## 13. Not Implemented Yet / 当前未实现项
- Corner geometric suspension model
- Camber/toe coupling
- Tire combined-slip or lateral-force coupling
- Load-transfer decomposition through roll centers / anti effects
- CornerModelBase integration
- Yaw main-flow closed-loop integration
- Nonlinear ARB maps
- Ride-height or aero map feedback

## 14. Next-Phase Extension Direction / 下一阶段扩展方向
- Replace the wheel-side equivalent force assumption with explicit damper-side / wheel-side mapping once the hardware definition is fixed.
- Add corner geometry outputs needed by `CornerModelBase`.
- Replace axle-level aero load inputs with ride-height-sensitive aero maps.
- Introduce wheel lift / no-tension tire contact handling if required by the use case.
- Add compatibility hooks for yaw main flow packaging, likely through a dedicated suspension I/O bus or a wrapper subsystem.
