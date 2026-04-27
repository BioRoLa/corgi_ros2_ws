#pragma once

#include <cmath>

namespace corgi
{

    /**
     * @brief Contact detection with OR-activate / AND-deactivate hysteresis.
     *
     * Activates  when  |rm| > rm_high  OR  |beta| > beta_high.
     * Deactivates when |rm| < rm_low  AND  |beta| < beta_low.
     */
    class ContactSchmittTrigger
    {
    public:
        ContactSchmittTrigger() = default;
        ContactSchmittTrigger(double rm_high, double rm_low,
                              double beta_high, double beta_low,
                              double gamma_high = 0.0, double gamma_low = 0.0)
            : rm_high_(rm_high), rm_low_(rm_low), beta_high_(beta_high), beta_low_(beta_low), gamma_high_(gamma_high), gamma_low_(gamma_low) {}

        bool update(double rm_value, double beta_value, double gamma_value = 0.0)
        {
            double abs_rm = std::abs(rm_value);
            double abs_beta = std::abs(beta_value);
            double abs_gamma = std::abs(gamma_value);

            const bool gamma_enabled = (gamma_high_ > 0.0 || gamma_low_ > 0.0);

            if (!state_)
            {
                if (abs_rm > rm_high_ || abs_beta > beta_high_ ||
                    (gamma_enabled && abs_gamma > gamma_high_))
                    state_ = true;
            }
            else
            {
                if (abs_rm < rm_low_ && abs_beta < beta_low_ &&
                    (!gamma_enabled || abs_gamma < gamma_low_))
                    state_ = false;
            }
            return state_;
        }

        bool state() const { return state_; }
        void reset() { state_ = false; }

    private:
        double rm_high_ = 0.0;
        double rm_low_ = 0.0;
        double beta_high_ = 0.0;
        double beta_low_ = 0.0;
        double gamma_high_ = 0.0;
        double gamma_low_ = 0.0;
        bool state_ = false;
    };

} // namespace corgi
