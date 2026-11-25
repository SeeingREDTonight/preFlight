///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/ TPMS-D (Schwarz Diamond) infill, ported from OrcaSlicer (src/libslic3r/Fill/FillTpmsD.*).
///|/
///|/ preFlight is based on PrusaSlicer and released under AGPLv3 or higher
///|/
#pragma once

#include <utility>

#include "luminary/core/Prelude.hpp"
#include "luminary/fill/contract/FillBase.hpp"
#include "luminary/geometry/contours/ExPolygon.hpp"
#include "luminary/geometry/contours/Polyline.hpp"

namespace Luminary
{
class Point;

class FillTpmsD : public Fill
{
public:
    FillTpmsD() {}
    Fill *clone() const override { return new FillTpmsD(*this); }

    bool use_bridge_flow() const override { return false; }
    bool is_self_crossing() override { return false; }

    // Correction applied to regular infill angle to maximize printing
    // speed in default configuration (degrees)
    static constexpr float CorrectionAngle = -45.;

    // Density adjustment to have a good %of weight.
    static constexpr double DensityAdjust = 2.1;

    // Upper resolution tolerance (mm^-2)
    static constexpr double PatternTolerance = 0.1;

protected:
    void _fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                              const std::pair<float, Point> &direction, ExPolygon expolygon,
                              Polylines &polylines_out) override;
};

} // namespace Luminary
