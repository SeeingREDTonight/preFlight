///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/ Copyright (c) Prusa Research 2017 - 2022 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2016 Alessandro Ranellucci @alranel
///|/
///|/ Copyright (c) Prusa Research 2016 - 2017 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2016 Chow Loong Jin @hyperair
///|/
///|/ preFlight is based on PrusaSlicer and released under AGPLv3 or higher
///|/
#pragma once

#include <stddef.h>
#include <map>
#include <string>
#include <array>
#include <vector>
#include <cstddef>

#include "luminary/core/Prelude.hpp"
#include "luminary/geometry/primitives/Point.hpp"
#include "luminary/config/catalog/PrintConfig.hpp"
#include "FanRampEstimator.hpp"
#include "FanCurve.hpp"
#include <deque>

namespace Luminary
{

class GCodeGenerator;
class Layer;
struct PerExtruderAdjustments;
class PrintConfig;

// A standalone G-code filter, to control cooling of the print.
// The G-code is processed per layer. Once a layer is collected, fan start / stop commands are edited
// and the print is modified to stretch over a minimum layer time.
//
// The simple it sounds, the actual implementation is significantly more complex.
// Namely, for a multi-extruder print, each material may require a different cooling logic.
// For example, some materials may not like to print too slowly, while with some materials
// we may slow down significantly.
//
class CoolingBuffer
{
public:
    CoolingBuffer(GCodeGenerator &gcodegen);
    // Out of line: the pending-layer queue holds PerExtruderAdjustments, defined in the .cpp.
    ~CoolingBuffer();
    void reset(const Vec3d &position);
    void set_current_extruder(unsigned int extruder_id)
    {
        m_current_extruder = extruder_id;
        m_parse_extruder = extruder_id;
    }
    // print_z: the layer's height, for the fan curve's per-mm change limit. last_layer: the final
    // layer of the run (the print, or one object in sequential mode); drains the look-ahead queue.
    std::string process_layer(std::string &&gcode, size_t layer_id, bool flush, double print_z = 0.,
                              bool last_layer = false);
    std::string process_layer(const std::string &gcode, size_t layer_id, bool flush, double print_z = 0.,
                              bool last_layer = false)
    {
        return this->process_layer(std::string(gcode), layer_id, flush, print_z, last_layer);
    }

private:
    CoolingBuffer &operator=(const CoolingBuffer &) = delete;
    // current_extruder: the extruder active at the start of the G-code; updated to the one active at its end.
    std::vector<PerExtruderAdjustments> parse_layer_gcode(const std::string &gcode, std::array<float, 5> &current_pos,
                                                          unsigned int &current_extruder) const;
    float calculate_layer_slowdown(std::vector<PerExtruderAdjustments> &per_extruder_adjustments);
    // Apply slow down over G-code lines stored in per_extruder_adjustments, enable fan if needed.
    // Returns the adjusted G-code.
    // fan_override: per extruder id, the base fan % the fan curve picked (negative: use the stock logic).
    std::string apply_layer_cooldown(const std::string &gcode, size_t layer_id, float layer_time,
                                     std::vector<PerExtruderAdjustments> &per_extruder_adjustments,
                                     const std::vector<double> *fan_override = nullptr);

    // G-code snippet cached for the support layers preceding an object layer.
    std::string m_gcode;
    // Internal data.
    std::vector<char> m_axis;
    enum AxisIdx : int
    {
        X = 0,
        Y,
        Z,
        E,
        F,
        I,
        J,
        K,
        R,
        Count
    };
    std::array<float, 5> m_current_pos;
    // Current known fan speed or -1 if not known yet.
    int m_fan_speed;
    // Cached from GCodeWriter.
    // Printing extruder IDs, zero based.
    std::vector<unsigned int> m_extruder_ids;
    // Highest of m_extruder_ids plus 1.
    unsigned int m_num_extruders{0};
    const std::string m_toolchange_prefix;
    // Referencs GCodeGenerator::m_config, which is FullPrintConfig. While the PrintObjectConfig slice of FullPrintConfig is being modified,
    // the PrintConfig slice of FullPrintConfig is constant, thus no thread synchronization is required.
    const PrintConfig &m_config;
    // The machine limits the time estimator resolved for the flavor (fan-ramp timing input).
    MachineEnvelopeConfig m_machine_limits;
    unsigned int m_current_extruder;

    // Per-extruder fan ramp estimators, initialized from printer config
    std::vector<FanRampEstimator> m_fan_ramp_estimators;

    // Initialize fan ramp estimators from config
    void init_fan_ramp_estimators();

    // ---- Fan curve (Curve / Breakpoints modes) ----
    // Per extruder id: the curve and its change limits, from the filament config.
    std::vector<FanCurve> m_fan_curves;
    std::vector<FanRampLimits> m_fan_ramp_limits;
    // Any extruder uses a non-stock mode / a rise limit (which needs look-ahead).
    bool m_fan_curve_used{false};
    double m_fan_lookahead_mm{0.};
    void init_fan_curves();

    // The extruder active at the end of the last parsed layer. Parsing runs ahead of
    // apply_layer_cooldown (which tracks m_current_extruder) while layers wait in the queue.
    unsigned int m_parse_extruder{0};

    // A layer parsed and slowed down, waiting for enough look-ahead to set its fan.
    struct PendingLayer
    {
        std::string gcode;
        size_t layer_id;
        double print_z;
        float layer_time_stretched;
        std::vector<PerExtruderAdjustments> adjustments;
        std::vector<double> target; // per extruder id, the curve's fan % (negative: stock logic)
    };
    std::deque<PendingLayer> m_pending;
    // The fan each extruder's curve emitted on the previous layer, and that layer's height.
    std::vector<double> m_ramp_prev_fan;
    double m_ramp_prev_z{0.};
    // Sets the front pending layer's fan and emits it.
    std::string emit_pending_front();
};

} // namespace Luminary
