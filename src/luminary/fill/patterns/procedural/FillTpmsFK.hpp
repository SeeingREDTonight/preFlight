///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/ TPMS-FK (Fischer-Koch S) infill, ported from OrcaSlicer (src/libslic3r/Fill/FillTpmsFK.*).
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

class FillTpmsFK : public Fill
{
public:
    FillTpmsFK() {}
    Fill *clone() const override { return new FillTpmsFK(*this); }

    bool use_bridge_flow() const override { return false; }
    bool is_self_crossing() override { return false; }

    // Correction applied to regular infill angle to maximize printing
    // speed in default configuration (degrees)
    static constexpr float CorrectionAngle = -45.;

protected:
    void _fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                              const std::pair<float, Point> &direction, ExPolygon expolygon,
                              Polylines &polylines_out) override;
};

} // namespace Luminary
