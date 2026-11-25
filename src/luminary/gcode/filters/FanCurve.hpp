///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ preFlight is based on PrusaSlicer and released under AGPLv3 or higher
///|/
#pragma once

// The layer-time fan curve shared by the cooling filter and the filament tab's preview, so the
// graph the user edits is the function the G-code is generated with. Header only and toolkit
// free: plain numbers in, a fan percentage out.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace Luminary
{

// Mirrors FanControlMode in PrintConfig.hpp; kept separate so this header needs no config.
enum class FanCurveMode
{
    Legacy,      // min/max interpolated between slowdown_below and fan_below layer times
    Curve,       // min/max shaped on a log-time axis between a start time and a full-speed time
    Breakpoints, // up to six (layer time, fan %) rows, interpolated on a log-time axis
};

struct FanCurve
{
    FanCurveMode mode{FanCurveMode::Legacy};
    bool fan_always_on{false};
    double min_speed{0.};   // %
    double max_speed{100.}; // %

    // Legacy: the stock thresholds, seconds.
    double legacy_fan_below{60.};
    double legacy_slowdown_below{5.};

    // Curve: the fan starts at min_speed below start_time, reaches max_speed at full_time, and
    // follows u^shape in between, u being the position between the two on a log-time axis.
    double start_time{3.};
    double full_time{0.25};
    double shape{1.};

    // Breakpoints: (layer time in seconds, fan %); rows with a time <= 0 are unused.
    std::vector<std::pair<double, double>> points;

    // Valid breakpoints sorted by decreasing layer time (slow layers first).
    std::vector<std::pair<double, double>> sorted_points() const
    {
        std::vector<std::pair<double, double>> out;
        for (const auto &p : points)
            if (p.first > 0.)
                out.emplace_back(p.first, std::clamp(p.second, 0., 100.));
        std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
        return out;
    }

    // Fan % the curve asks for at a layer time in seconds. For Curve and Breakpoints the time is
    // the natural (pre-slowdown) layer time; for Legacy it is whatever the stock logic passes.
    double eval(double layer_time) const
    {
        const double floor_speed = fan_always_on ? min_speed : 0.;
        switch (mode)
        {
        case FanCurveMode::Legacy:
        {
            if (layer_time < legacy_slowdown_below)
                return max_speed;
            if (layer_time < legacy_fan_below)
            {
                const double t = (layer_time - legacy_slowdown_below) / (legacy_fan_below - legacy_slowdown_below);
                return t * min_speed + (1. - t) * max_speed;
            }
            return floor_speed;
        }
        case FanCurveMode::Curve:
        {
            const double t_start = std::max(start_time, 1e-3);
            const double t_full = std::clamp(full_time, 1e-4, t_start);
            if (layer_time >= t_start)
                return floor_speed;
            if (layer_time <= t_full || t_full >= t_start)
                return max_speed;
            const double u = (std::log(t_start) - std::log(std::max(layer_time, 1e-6))) /
                             (std::log(t_start) - std::log(t_full));
            const double k = std::clamp(shape, 0.05, 20.);
            return std::max(floor_speed, min_speed + (max_speed - min_speed) * std::pow(std::clamp(u, 0., 1.), k));
        }
        case FanCurveMode::Breakpoints:
        {
            const auto pts = sorted_points();
            if (pts.empty())
                return floor_speed;
            if (layer_time >= pts.front().first)
                return pts.front().second;
            if (layer_time <= pts.back().first)
                return pts.back().second;
            for (size_t i = 0; i + 1 < pts.size(); ++i)
            {
                const auto &hi = pts[i];     // slower layer time
                const auto &lo = pts[i + 1]; // faster layer time
                if (layer_time <= hi.first && layer_time >= lo.first)
                {
                    if (hi.first <= lo.first)
                        return lo.second;
                    const double u = (std::log(hi.first) - std::log(layer_time)) /
                                     (std::log(hi.first) - std::log(lo.first));
                    return hi.second + (lo.second - hi.second) * u;
                }
            }
            return pts.back().second;
        }
        }
        return floor_speed;
    }

    // The span of values eval() can return; sizes the look-ahead window.
    std::pair<double, double> output_range() const
    {
        const double floor_speed = fan_always_on ? min_speed : 0.;
        if (mode == FanCurveMode::Breakpoints)
        {
            const auto pts = sorted_points();
            if (pts.empty())
                return {floor_speed, floor_speed};
            double lo = pts.front().second, hi = lo;
            for (const auto &p : pts)
            {
                lo = std::min(lo, p.second);
                hi = std::max(hi, p.second);
            }
            return {lo, hi};
        }
        return {std::min(floor_speed, min_speed), std::max({floor_speed, min_speed, max_speed})};
    }
};

// Rate limit for a fan sequence along Z, in % per mm. A rise is started early (look-ahead), so
// no layer gets less fan than its target; a fall is spread over the layers after the drop.
// rate <= 0 means unlimited in that direction.
struct FanRampLimits
{
    double up{0.};
    double down{0.};
    bool active() const { return up > 0. || down > 0.; }
};

// Fan for layer `i` of a run of targets at heights `z`, given the previous emitted layer's fan and
// height (prev_fan < 0: none). Only layers within (range / up) mm above `i` can raise it, so a
// streaming caller holding that much look-ahead gets the same answer as a whole-print pass.
inline double fan_ramp_value(const std::vector<double> &z, const std::vector<double> &target, size_t i,
                             const FanRampLimits &limits, double prev_fan, double prev_z)
{
    double f = target[i];
    if (limits.up > 0.)
        for (size_t j = i + 1; j < target.size(); ++j)
            f = std::max(f, target[j] - limits.up * std::max(0., z[j] - z[i]));
    if (limits.down > 0. && prev_fan >= 0.)
        f = std::max(f, prev_fan - limits.down * std::max(0., z[i] - prev_z));
    return std::clamp(f, 0., 100.);
}

// Height of look-ahead a rise limit needs: the largest possible jump divided by the rate.
inline double fan_ramp_lookahead_mm(const FanCurve &curve, const FanRampLimits &limits)
{
    if (limits.up <= 0.)
        return 0.;
    const auto [lo, hi] = curve.output_range();
    return std::max(0., hi - lo) / limits.up;
}

} // namespace Luminary
