
#include "motor.hpp"
#include "axis.hpp"
#include "low_level.h"
#include "odrive_main.h"

#include <algorithm>

static constexpr auto CURRENT_ADC_LOWER_BOUND =        (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MIN_VOLT / 3.3f);
static constexpr auto CURRENT_ADC_UPPER_BOUND =        (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MAX_VOLT / 3.3f);

/**
 * @brief This control law adjusts the output voltage such that a predefined
 * current is tracked. A hardcoded integrator gain is used for this.
 * 
 * TODO: this might as well be implemented using the FieldOrientedController.
 */
/**
  * @brief 这个控制律调整输出电压，使得一个预定义的
  * 电流被跟踪。 为此使用了硬编码的积分器增益。
  *
  * TODO：这也可以使用 FieldOrientedController 来实现。
  */
struct ResistanceMeasurementControlLaw : AlphaBetaFrameController {
    void reset() final {
        test_voltage_ = 0.0f;
        test_mod_ = std::nullopt;
    }

    ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<float2D> Ialpha_beta,
            uint32_t input_timestamp) final {

        if (Ialpha_beta.has_value()) {
            actual_current_ = Ialpha_beta->first;
            test_voltage_ += (kI * current_meas_period) * (target_current_ - actual_current_);
            I_beta_ += (kIBetaFilt * current_meas_period) * (Ialpha_beta->second - I_beta_);
        } else {
            actual_current_ = 0.0f;
            test_voltage_ = 0.0f;
        }
    
        if (std::abs(test_voltage_) > max_voltage_) {
            test_voltage_ = NAN;
            return Motor::ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        } else if (!vbus_voltage.has_value()) {
            return Motor::ERROR_UNKNOWN_VBUS_VOLTAGE;
        } else {
            float vfactor = 1.0f / ((2.0f / 3.0f) * *vbus_voltage);
            test_mod_ = test_voltage_ * vfactor;
            return Motor::ERROR_NONE;
        }
    }

    ODriveIntf::MotorIntf::Error get_alpha_beta_output(
            uint32_t output_timestamp,
            std::optional<float2D>* mod_alpha_beta,
            std::optional<float>* ibus) final {
        if (!test_mod_.has_value()) {
            return Motor::ERROR_CONTROLLER_INITIALIZING;
        } else {
            *mod_alpha_beta = {*test_mod_, 0.0f};
            *ibus = *test_mod_ * actual_current_;
            return Motor::ERROR_NONE;
        }
    }

    float get_resistance() {
        return test_voltage_ / target_current_;
    }

    float get_Ibeta() {
        return I_beta_;
    }

    const float kI = 1.0f; // [(V/s)/A]
    const float kIBetaFilt = 80.0f;
    float max_voltage_ = 0.0f;
    float actual_current_ = 0.0f;
    float target_current_ = 0.0f;
    float test_voltage_ = 0.0f;
    float I_beta_ = 0.0f; // [A] low pass filtered Ibeta response 低通滤波 Ibeta 响应
    std::optional<float> test_mod_ = NAN;
};

/**
 * @brief This control law toggles rapidly between positive and negative output
 * voltage. By measuring how large the current ripples are, the phase inductance
 * can be determined.
 * 
 * TODO: this method assumes a certain synchronization between current measurement and output application
 */
/**
  * @brief 这个控制律在正负输出之间快速切换电压。 
  * 通过测量电流纹波有多大，可以确定相电感。
  * 
  * TODO：此方法假定电流测量和输出应用之间有一定的同步
  */
struct InductanceMeasurementControlLaw : AlphaBetaFrameController {
    void reset() final {
        attached_ = false;
    }

    ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<float2D> Ialpha_beta,
            uint32_t input_timestamp) final
    {
        if (!Ialpha_beta.has_value()) {
            return {Motor::ERROR_UNKNOWN_CURRENT_MEASUREMENT};
        }

        float Ialpha = Ialpha_beta->first;

        if (attached_) {
            float sign = test_voltage_ >= 0.0f ? 1.0f : -1.0f;
            deltaI_ += -sign * (Ialpha - last_Ialpha_);
        } else {
            start_timestamp_ = input_timestamp;
            attached_ = true;
        }

        last_Ialpha_ = Ialpha;
        last_input_timestamp_ = input_timestamp;

        return Motor::ERROR_NONE;
    }

    ODriveIntf::MotorIntf::Error get_alpha_beta_output(
            uint32_t output_timestamp, std::optional<float2D>* mod_alpha_beta,
            std::optional<float>* ibus) final
    {
        test_voltage_ *= -1.0f;
        float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
        *mod_alpha_beta = {test_voltage_ * vfactor, 0.0f};
        *ibus = 0.0f;
        return Motor::ERROR_NONE;
    }

    float get_inductance() {
        // Note: A more correct formula would also take into account that there is a finite timestep.
        // However, the discretisation in the current control loop inverts the same discrepancy
        // 注意：更正确的公式也会考虑到时间步长是有限的。
        // 然而，当前控制回路中的离散化反转了相同的差异
        float dt = (float)(last_input_timestamp_ - start_timestamp_) / (float)TIM_1_8_CLOCK_HZ; // at 216MHz this overflows after 19 seconds
        return std::abs(test_voltage_) / (deltaI_ / dt);
    }

    // Config
    float test_voltage_ = 0.0f;

    // State
    bool attached_ = false;
    float sign_ = 0;

    // Outputs
    uint32_t start_timestamp_ = 0;
    float last_Ialpha_ = NAN;
    uint32_t last_input_timestamp_ = 0;
    float deltaI_ = 0.0f;
};


Motor::Motor(TIM_HandleTypeDef* timer,
             uint8_t current_sensor_mask,
             float shunt_conductance,
             TGateDriver& gate_driver,
             TOpAmp& opamp,
             OnboardThermistorCurrentLimiter& fet_thermistor,
             OffboardThermistorCurrentLimiter& motor_thermistor) :
        timer_(timer),
        current_sensor_mask_(current_sensor_mask),
        shunt_conductance_(shunt_conductance),
        gate_driver_(gate_driver),
        opamp_(opamp),
        fet_thermistor_(fet_thermistor),
        motor_thermistor_(motor_thermistor) {
    apply_config();
    fet_thermistor_.motor_ = this;
    motor_thermistor_.motor_ = this;
}

/**
 * @brief Arms the PWM outputs that belong to this motor.
 *
 * Note that this does not activate the PWM outputs immediately, it just sets
 * a flag so they will be enabled later.
 * 
 * The sequence goes like this:
 *  - Motor::arm() sets the is_armed_ flag.
 *  - On the next timer update event Motor::timer_update_cb() gets called in an
 *    interrupt context
 *  - Motor::timer_update_cb() runs specified control law to determine PWM values
 *  - Motor::timer_update_cb() calls Motor::apply_pwm_timings()
 *  - Motor::apply_pwm_timings() sets the output compare registers and the AOE
 *    (automatic output enable) bit.
 *  - On the next update event the timer latches the configured values into the
 *    active shadow register and enables the outputs at the same time.
 * 
 * The sequence can be aborted at any time by calling Motor::disarm().
 *
 * @param control_law: An control law that is called at the frequency of current
 *        measurements. The function must return as quickly as possible
 *        such that the resulting PWM timings are available before the next
 *        timer update event.
 * @returns: True on success, false otherwise
 */
/**
 * @brief Arms 属于该电机的 PWM 输出。
 *
 * 请注意，这不会立即激活 PWM 输出，它只是设置一个标志，以便稍后启用它们。
 *
 * 顺序是这样的：
 * - Motor::arm() 设置 is_armed_ 标志。
 * - 在下一个定时器更新事件 Motor::timer_update_cb() 在中断上下文中被调用
 * - Motor::timer_update_cb() 运行指定的控制律以确定 PWM 值
 * - Motor::timer_update_cb() 调用 Motor::apply_pwm_timings()
 * - Motor::apply_pwm_timings() 设置输出比较寄存器和 AOE（自动输出启用）位。
 * - 在下一次更新事件时，定时器将配置的值锁存到活动影子寄存器中并同时启用输出。
 * 
 * 可以通过调用 Motor::disarm() 随时中止序列。
 *
 * @param control_law: 以电流频率调用的控制律测量。函数必须尽快返回,
 * 这样产生的 PWM 时序在下一个之前可用定时器更新事件。
 * @returns: 成功时为真，否则为假
 */
bool Motor::arm(PhaseControlLaw<3>* control_law) {
    axis_->mechanical_brake_.release();

    CRITICAL_SECTION() {
        control_law_ = control_law;

        // Reset controller states, integrators, setpoints, etc.
        axis_->controller_.reset();
        axis_->acim_estimator_.rotor_flux_ = 0.0f;
        if (control_law_) {
            control_law_->reset();
        }

        if (!odrv.config_.enable_brake_resistor || brake_resistor_armed) {
            armed_state_ = 1;
            is_armed_ = true;
        } else {
            error_ |= Motor::ERROR_BRAKE_RESISTOR_DISARMED;
        }
    }

    return true;
}

/**
 * @brief Updates the phase PWM timings unless the motor is disarmed.
 *
 * If the motor is armed, the PWM timings come into effect at the next update
 * event (and are enabled if they weren't already), unless the motor is disarmed
 * prior to that.
 * 
 * @param tentative: If true, the update is not counted as "refresh".
 */
/**
  * @brief 更新相位 PWM 时序，除非电机已解除。
  *
  * 如果电机已启用，PWM 时序会在下一次更新事件时生效（如果尚未启用，则启用），
  * 除非电机在此之前已停用。
  * @param 暂定：如果为真，则更新不计为“刷新”。
  */ 
void Motor::apply_pwm_timings(uint16_t timings[3], bool tentative) {
    CRITICAL_SECTION() {
        if (odrv.config_.enable_brake_resistor && !brake_resistor_armed) {
            disarm_with_error(ERROR_BRAKE_RESISTOR_DISARMED);
        }

        TIM_HandleTypeDef* htim = timer_;
        TIM_TypeDef* tim = htim->Instance;
        tim->CCR1 = timings[0];
        tim->CCR2 = timings[1];
        tim->CCR3 = timings[2];
        
        if (!tentative) {
            if (is_armed_) {
                // Set the Automatic Output Enable so that the Master Output Enable
                // bit will be automatically enabled on the next update event.
                // 设置自动输出启用，以便在下一次更新事件时自动启用主输出启用位。
                tim->BDTR |= TIM_BDTR_AOE;
            }
        }
        
        // If a timer update event occurred just now while we were updating the
        // timings, we can't be sure what values the shadow registers now contain,
        // so we must disarm the motor.
        // (this also protects against the case where the update interrupt has too
        // low priority, but that should not happen)
        // 如果在我们更新 btimings 时刚刚发生了计时器更新事件，
        // 我们无法确定影子寄存器现在包含哪些值，因此我们必须解除电机。
        // （这也可以防止更新中断发生的情况 优先级太低，但这不应该发生）
        //if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE)) {
        //    disarm_with_error(ERROR_CONTROL_DEADLINE_MISSED);
        //}
    }
}

/**
 * @brief Disarms the motor PWM.
 * 
 * After this function returns, it is guaranteed that all three
 * motor phases are floating and will not be enabled again until
 * arm() is called.
 */
/**
 * @brief 电机 PWM 去使能
 * 此函数返回后，保证所有三个电机相位都处于浮动状态，
 * 并且在调用 arm() 之前不会再次启用。
 */
bool Motor::disarm(bool* p_was_armed) {
    bool was_armed;
    
    CRITICAL_SECTION() {
        was_armed = is_armed_;
        if (is_armed_) {
            gate_driver_.set_enabled(false);
        }
        is_armed_ = false;
        armed_state_ = 0;
        TIM_HandleTypeDef* timer = timer_;
        timer->Instance->BDTR &= ~TIM_BDTR_AOE; // prevent the PWMs from automatically enabling at the next update
                                                // 防止 PWM 在下次更新时自动启用
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timer);
        control_law_ = nullptr;
    }

    // Check necessary to prevent infinite recursion
    // 必要的检查以防止无限递归
    if (was_armed) {
        update_brake_current();
    }

    if (p_was_armed) {
        *p_was_armed = was_armed;
    }

    return true;
}

// @brief Tune the current controller based on phase resistance and inductance
// This should be invoked whenever one of these values changes.
// TODO: allow update on user-request or update automatically via hooks
// @brief 根据相电阻和电感调整电流控制器。只要这些值之一发生变化，就应该调用它。
// TODO: 允许更新用户请求或通过钩子自动更新 
void Motor::update_current_controller_gains() {
    // Calculate current control gains
    float p_gain = config_.current_control_bandwidth * config_.phase_inductance;
    float plant_pole = config_.phase_resistance / config_.phase_inductance;
    current_control_.pi_gains_ = {p_gain, plant_pole * p_gain};
}

bool Motor::apply_config() {
    config_.parent = this;
    is_calibrated_ = config_.pre_calibrated;
    update_current_controller_gains();
    return true;
}

// @brief Set up the gate drivers
// @brief 设置栅极驱动器
bool Motor::setup() {
    fet_thermistor_.update();
    motor_thermistor_.update();

    // Solve for exact gain, then snap down to have equal or larger range as requested
    // or largest possible range otherwise
    // 求解精确的增益，然后按要求缩小到具有相等或更大的范围或否则可能的最大范围
    constexpr float kMargin = 0.90f;
    constexpr float max_output_swing = 1.35f; // [V] out of amplifier
    float max_unity_gain_current = kMargin * max_output_swing * shunt_conductance_; // [A]
    float requested_gain = max_unity_gain_current / config_.requested_current_range; // [V/V]
    
    float actual_gain;
    if (!gate_driver_.config(requested_gain, &actual_gain))
        return false;

    // Values for current controller
    // 当前控制器的值
    phase_current_rev_gain_ = 1.0f / actual_gain;
    // Clip all current control to actual usable range
    // 将所有当前控件剪辑到实际可用范围
    max_allowed_current_ = max_unity_gain_current * phase_current_rev_gain_;

    max_dc_calib_ = 0.1f * max_allowed_current_;

    if (!gate_driver_.init())
        return false;

    return true;
}

void Motor::disarm_with_error(Motor::Error error){
    error_ |= error;
    last_error_time_ = odrv.n_evt_control_loop_ * current_meas_period;
    disarm();
}

bool Motor::do_checks(uint32_t timestamp) {
    gate_driver_.do_checks();

    if (!gate_driver_.is_ready()) {
        disarm_with_error(ERROR_DRV_FAULT);
        return false;
    }
    if (!motor_thermistor_.do_checks()) {
        disarm_with_error(ERROR_MOTOR_THERMISTOR_OVER_TEMP);
        return false;
    }
    if (!fet_thermistor_.do_checks()) {
        disarm_with_error(ERROR_FET_THERMISTOR_OVER_TEMP);
        return false;
    }
    return true;
}

float Motor::effective_current_lim() {
    // Configured limit
    float current_lim = config_.current_lim;
    // Hardware limit
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL) {
        current_lim = std::min(current_lim, 0.98f*one_by_sqrt3*vbus_voltage); //gimbal motor is voltage control
    } else {
        current_lim = std::min(current_lim, axis_->motor_.max_allowed_current_);
    }

    // Apply thermistor current limiters
    // 应用热敏电阻限流器
    current_lim = std::min(current_lim, motor_thermistor_.get_current_limit(config_.current_lim));
    current_lim = std::min(current_lim, fet_thermistor_.get_current_limit(config_.current_lim));
    effective_current_lim_ = current_lim;

    return effective_current_lim_;
}

//return the maximum available torque for the motor.
//Note - for ACIM motors, available torque is allowed to be 0.
//返回电机的最大可用扭矩。
//注意 - 对于 ACIM 电机，允许可用扭矩为 0。
float Motor::max_available_torque() {
    if (config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float max_torque = effective_current_lim_ * config_.torque_constant * axis_->acim_estimator_.rotor_flux_;
        max_torque = std::clamp(max_torque, 0.0f, config_.torque_lim);
        return max_torque;
    } else {
        float max_torque = effective_current_lim_ * config_.torque_constant;
        max_torque = std::clamp(max_torque, 0.0f, config_.torque_lim);
        return max_torque;
    }
}

std::optional<float> Motor::phase_current_from_adcval(uint32_t ADCValue) {
    // Make sure the measurements don't come too close to the current sensor's hardware limitations
    // 确保测量值不会太接近当前传感器的硬件限制
    if (ADCValue < CURRENT_ADC_LOWER_BOUND || ADCValue > CURRENT_ADC_UPPER_BOUND) {
        error_ |= ERROR_CURRENT_SENSE_SATURATION;
        return std::nullopt;
    }

    int adcval_bal = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * phase_current_rev_gain_;
    float current = shunt_volt * shunt_conductance_;
    return current;
}

//--------------------------------
// Measurement and calibration
//--------------------------------

// TODO check Ibeta balance to verify good motor connection
// TODO 检查 Ibeta 平衡以验证电机连接良好
bool Motor::measure_phase_resistance(float test_current, float max_voltage) {
    ResistanceMeasurementControlLaw control_law;
    control_law.target_current_ = test_current;
    control_law.max_voltage_ = max_voltage;

    arm(&control_law);

    for (size_t i = 0; i < 3000; ++i) {
        if (!((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_)) {
            break;
        }
        osDelay(1);
    }

    bool success = is_armed_;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    disarm();

    config_.phase_resistance = control_law.get_resistance();
    if (is_nan(config_.phase_resistance)) {
        // TODO: the motor is already disarmed at this stage. This is an error
        // that only pretains to the measurement and its result so it should
        // just be a return value of this function.
        // TODO：电机在此阶段已被解除。 
        // 这是一个仅适用于测量及其结果的错误，因此它应该只是此函数的返回值。
        disarm_with_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE);
        success = false;
    }

    float I_beta = control_law.get_Ibeta();
    if (is_nan(I_beta) || (abs(I_beta) / test_current) > 0.1f) {
        disarm_with_error(ERROR_UNBALANCED_PHASES);
        success = false;
    }

    return success;
}


bool Motor::measure_phase_inductance(float test_voltage) {
    InductanceMeasurementControlLaw control_law;
    control_law.test_voltage_ = test_voltage;

    arm(&control_law);

    for (size_t i = 0; i < 1250; ++i) {
        if (!((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_)) {
            break;
        }
        osDelay(1);
    }

    bool success = is_armed_;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    disarm();

    config_.phase_inductance = control_law.get_inductance();
    
    // TODO arbitrary values set for now
    // TODO 为现在设置的任意值
    if (!(config_.phase_inductance >= 2e-6f && config_.phase_inductance <= 4000e-6f)) {
        error_ |= ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        success = false;
    }

    return success;
}


// TODO: motor calibration should only be a utility function that's called from
// the UI on explicit user request. It should take its parameters as input
// arguments and return the measured results without modifying any config values.
// TODO：电机校准应该只是一个在用户明确请求时从 UI 调用的实用函数。 
// 它应该将其参数作为输入参数，并在不修改任何配置值的情况下返回测量结果。
bool Motor::run_calibration() {
    float R_calib_max_voltage = config_.resistance_calib_max_voltage;
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT
        || config_.motor_type == MOTOR_TYPE_ACIM) {
        if (!measure_phase_resistance(config_.calibration_current, R_calib_max_voltage))
            return false;
        if (!measure_phase_inductance(R_calib_max_voltage))
            return false;
    } else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        // no calibration needed
    } else {
        return false;
    }

    update_current_controller_gains();
    
    is_calibrated_ = true;
    return true;
}

void Motor::update(uint32_t timestamp) {
    // Load torque setpoint, convert to motor direction
    // 负载转矩设定值，转换为电机方向
    std::optional<float> maybe_torque = torque_setpoint_src_.present();
    if (!maybe_torque.has_value()) {
        error_ |= ERROR_UNKNOWN_TORQUE;
        return;
    }
    float torque = direction_ * *maybe_torque;

    // Load setpoints from previous iteration.
    // 从前一次迭代加载设定点。
    auto [id, iq] = Idq_setpoint_.previous()
                     .value_or(float2D{0.0f, 0.0f});
    // Load effective current limit
    // 负载有效电流限制
    float ilim = axis_->motor_.effective_current_lim_;

    // Autoflux tracks old Iq (that may be 2-norm clamped last cycle) to make sure we are chasing a feasable current.
    // Autoflux 跟踪旧的 Iq（可能是最后一个周期的 2 范数钳位）以确保我们正在追逐可行的电流。
    if ((axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) && config_.acim_autoflux_enable) {
        float abs_iq = std::abs(iq);
        float gain = abs_iq > id ? config_.acim_autoflux_attack_gain : config_.acim_autoflux_decay_gain;
        id += gain * (abs_iq - id) * current_meas_period;
        id = std::clamp(id, config_.acim_autoflux_min_Id, 0.9f * ilim); // 10% space reserved for Iq 为 Iq 保留 10% 的空间
    } else {
        id = std::clamp(id, -ilim*0.99f, ilim*0.99f); // 1% space reserved for Iq to avoid numerical issues
                                                      // 为 Iq 保留 1% 的空间以避免数字问题
    }

    // Convert requested torque to current
    // 将请求的扭矩转换为电流
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        iq = torque / (axis_->motor_.config_.torque_constant * std::max(axis_->acim_estimator_.rotor_flux_, config_.acim_gain_min_flux));
    } else {
        iq = torque / axis_->motor_.config_.torque_constant;
    }

    // 2-norm clamping where Id takes priority
    // 2 范数限制，其中 Id 优先
    float iq_lim_sqr = SQ(ilim) - SQ(id);
    float Iq_lim = (iq_lim_sqr <= 0.0f) ? 0.0f : sqrt(iq_lim_sqr);
    iq = std::clamp(iq, -Iq_lim, Iq_lim);

    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL) {
        Idq_setpoint_ = {id, iq};
    }

    // This update call is in bit a weird position because it depends on the
    // Id,q setpoint but outputs the phase velocity that we depend on later
    // in this function.
    // A cleaner fix would be to take the feedforward calculation out of here
    // and turn it into a separate component.
    // 这个更新调用有点奇怪，因为它取决于 Id,q 设置点，但输出我们稍后在这个函数中依赖的相速度。
    // 一个更简洁的解决方法是将前馈计算从这里去掉，并将其变成一个单独的组件。
    MEASURE_TIME(axis_->task_times_.acim_estimator_update)
        axis_->acim_estimator_.update(timestamp);

    float vd = 0.0f;
    float vq = 0.0f;

    std::optional<float> phase_vel = phase_vel_src_.present();

    if (config_.R_wL_FF_enable) {
        if (!phase_vel.has_value()) {
            error_ |= ERROR_UNKNOWN_PHASE_VEL;
            return;
        }

        vd -= *phase_vel * config_.phase_inductance * iq;
        vq += *phase_vel * config_.phase_inductance * id;
        vd += config_.phase_resistance * id;
        vq += config_.phase_resistance * iq;
    }

    if (config_.bEMF_FF_enable) {
        if (!phase_vel.has_value()) {
            error_ |= ERROR_UNKNOWN_PHASE_VEL;
            return;
        }

        vq += *phase_vel * (2.0f/3.0f) * (config_.torque_constant / config_.pole_pairs);
    }
    
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL) {
        // reinterpret current as voltage
        // 将电流重新解释为电压
        Vdq_setpoint_ = {vd + id, vq + iq};
    } else {
        Vdq_setpoint_ = {vd, vq};
    }
}


/**
 * @brief Called when the underlying hardware timer triggers an update event.
 */
/**
  * @brief 当底层硬件定时器触发更新事件时调用。
  */
void Motor::current_meas_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current) {
    // TODO: this is platform specific
    // TODO：这是特定于平台的
    //const float current_meas_period = static_cast<float>(2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1)) / TIM_1_8_CLOCK_HZ;
    TaskTimerContext tmr{axis_->task_times_.current_sense};

    n_evt_current_measurement_++;

    bool dc_calib_valid = (dc_calib_running_since_ >= config_.dc_calib_tau * 7.5f)
                       && (abs(DC_calib_.phA) < max_dc_calib_)
                       && (abs(DC_calib_.phB) < max_dc_calib_)
                       && (abs(DC_calib_.phC) < max_dc_calib_);

    if (armed_state_ == 1 || armed_state_ == 2) {
        current_meas_ = {0.0f, 0.0f, 0.0f};
        armed_state_ += 1;
    } else if (current.has_value() && dc_calib_valid) {
        current_meas_ = {
            current->phA - DC_calib_.phA,
            current->phB - DC_calib_.phB,
            current->phC - DC_calib_.phC
        };
    } else {
        current_meas_ = std::nullopt;
    }

    // Run system-level checks (e.g. overvoltage/undervoltage condition)
    // The motor might be disarmed in this function. In this case the
    // handler will continue to run until the end but it won't have an
    // effect on the PWM.
    // 运行系统级检查（例如过压/欠压情况）。电机可能会在此功能中解除。 
    // 在这种情况下，处理程序将继续运行直到结束，但不会对 PWM 产生影响。
    odrv.do_fast_checks();

    if (current_meas_.has_value()) {
        // Check for violation of current limit
        // 检查是否违反了电流限制
        // If Ia + Ib + Ic == 0 holds then we have:
        // Inorm^2 = Id^2 + Iq^2 = Ialpha^2 + Ibeta^2 = 2/3 * (Ia^2 + Ib^2 + Ic^2)
        float Itrip = effective_current_lim_ + config_.current_lim_margin;
        float Inorm_sq = 2.0f / 3.0f * (SQ(current_meas_->phA)
                                      + SQ(current_meas_->phB)
                                      + SQ(current_meas_->phC));

        // Hack: we disable the current check during motor calibration because
        // it tends to briefly overshoot when the motor moves to align flux with I_alpha
        // Hack：我们在电机校准期间禁用电流检查，因为当电机移动以将通量与 I_alpha 对齐时，它往往会短暂过冲
        if (Inorm_sq > SQ(Itrip)) {
            disarm_with_error(ERROR_CURRENT_LIMIT_VIOLATION);
        }
    } else if (is_armed_) {
        // Since we can't check current limits, be safe for now and disarm.
        // Theoretically we could continue to operate if there is no active
        // current limit.
        // 由于我们无法检查电流限制，现在请注意安全并解除武装。
        // 理论上，如果没有有效电流限制，我们可以继续操作。
        disarm_with_error(ERROR_UNKNOWN_CURRENT_MEASUREMENT);
    }

    if (control_law_) {
        Error err = control_law_->on_measurement(vbus_voltage,
                            current_meas_.has_value() ?
                                std::make_optional(std::array<float, 3>{current_meas_->phA, current_meas_->phB, current_meas_->phC})
                                : std::nullopt,
                            timestamp);
        if (err != ERROR_NONE) {
            disarm_with_error(err);
        }
    }
}

/**
 * @brief Called when the underlying hardware timer triggers an update event.
 */
/**
  * @brief 当底层硬件定时器触发更新事件时调用。
  */
void Motor::dc_calib_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current) {
    const float dc_calib_period = static_cast<float>(2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1)) / TIM_1_8_CLOCK_HZ;
    TaskTimerContext tmr{axis_->task_times_.dc_calib};

    if (current.has_value()) {
        const float calib_filter_k = std::min(dc_calib_period / config_.dc_calib_tau, 1.0f);
        DC_calib_.phA += (current->phA - DC_calib_.phA) * calib_filter_k;
        DC_calib_.phB += (current->phB - DC_calib_.phB) * calib_filter_k;
        DC_calib_.phC += (current->phC - DC_calib_.phC) * calib_filter_k;
        dc_calib_running_since_ += dc_calib_period;
    } else {
        DC_calib_.phA = 0.0f;
        DC_calib_.phB = 0.0f;
        DC_calib_.phC = 0.0f;
        dc_calib_running_since_ = 0.0f;
    }
}


void Motor::pwm_update_cb(uint32_t output_timestamp) {
    TaskTimerContext tmr{axis_->task_times_.pwm_update};
    n_evt_pwm_update_++;

    Error control_law_status = ERROR_CONTROLLER_FAILED;
    float pwm_timings[3] = {NAN, NAN, NAN};
    std::optional<float> i_bus;

    if (control_law_) {
        control_law_status = control_law_->get_output(
            output_timestamp, pwm_timings, &i_bus);
    }

    // Apply control law to calculate PWM duty cycles
    // 应用控制律来计算 PWM 占空比
    if (is_armed_ && control_law_status == ERROR_NONE) {
        uint16_t next_timings[] = {
            (uint16_t)(pwm_timings[0] * (float)TIM_1_8_PERIOD_CLOCKS),
            (uint16_t)(pwm_timings[1] * (float)TIM_1_8_PERIOD_CLOCKS),
            (uint16_t)(pwm_timings[2] * (float)TIM_1_8_PERIOD_CLOCKS)
        };
        apply_pwm_timings(next_timings, false);
    } else if (is_armed_) {
        if (!(timer_->Instance->BDTR & TIM_BDTR_MOE) && (control_law_status == ERROR_CONTROLLER_INITIALIZING)) {
            // If the PWM output is armed in software but not yet in
            // hardware we tolerate the "initializing" error.
            // 如果 PWM 输出在软件中武装但尚未在硬件中，我们可以容忍“初始化”错误。
            i_bus = 0.0f;
        } else {
            disarm_with_error(control_law_status);
        }
    }

    if (!is_armed_) {
        // If something above failed, reset I_bus to 0A.
        // 如果上述操作失败，则将 I_bus 重置为 0A。
        i_bus = 0.0f;
    } else if (is_armed_ && !i_bus.has_value()) {
        // If the motor is armed then i_bus must be known
        // 如果电机已启动，则必须知道 i_bus
        disarm_with_error(ERROR_UNKNOWN_CURRENT_MEASUREMENT);
        i_bus = 0.0f;
    }

    I_bus_ = *i_bus;

    if (*i_bus < config_.I_bus_hard_min || *i_bus > config_.I_bus_hard_max) {
        disarm_with_error(ERROR_I_BUS_OUT_OF_RANGE);
    }

    update_brake_current();
}
