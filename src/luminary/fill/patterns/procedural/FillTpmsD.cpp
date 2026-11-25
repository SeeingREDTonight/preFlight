///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/ TPMS-D (Schwarz Diamond) infill, ported from OrcaSlicer (src/libslic3r/Fill/FillTpmsD.*).
///|/
///|/ preFlight is based on PrusaSlicer and released under AGPLv3 or higher
///|/
#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>
#include <cstddef>

#include "luminary/geometry/clipper/ClipperUtils.hpp"
#include "luminary/toolpath/ordering/ShortestPath.hpp"
#include "FillTpmsD.hpp"
#include "luminary/geometry/primitives/BoundingBox.hpp"
#include "luminary/fill/contract/FillBase.hpp"
#include "luminary/geometry/primitives/Point.hpp"
#include "luminary/geometry/contours/Polygon.hpp"
#include "luminary/core/Prelude.hpp"

namespace Luminary
{

static double scaled_floor(double x, double scale)
{
    return std::floor(x / scale) * scale;
}

// Level set sin(x)sin(y)sin(z) - cos(x)cos(y)cos(z) = 0, traced per layer.
static Polylines make_tpmsd_waves(double gridZ, double density_adjusted, double line_spacing, double width,
                                  double height)
{
    const double scaleFactor = scale_(line_spacing) / density_adjusted;

    // tolerance in scaled units. clamp the maximum tolerance as there's
    // no processing-speed benefit to do so beyond a certain point
    const double tolerance = std::min(line_spacing / 2, FillTpmsD::PatternTolerance) / unscale<double>(scaleFactor);

    const double z = gridZ / scaleFactor;
    Polylines result;

    // sin(x)sin(y)sin(z) - cos(x)cos(y)cos(z) = 0
    // (sin(z) - cos(z)) cos(x - y) - (sin(z) + cos(z)) cos(x + y) = 0
    double a = sin(z) - cos(z);
    double b = sin(z) + cos(z);
    // a cos(u) - b cos(v) = 0 with u = x - y, v = x + y
    double minU = 0 - height;
    double maxU = width - 0;
    double minV = 0 + 0;
    double maxV = width + height;
    // if |b| < |a| then u = acos(b/a cos(v)) is a continuous line, otherwise swap u and v
    const bool swapUV = (std::abs(a) > std::abs(b));
    if (swapUV)
    {
        std::swap(a, b);
        std::swap(minU, minV);
        std::swap(maxU, maxV);
    }
    std::vector<std::pair<double, double>> wave;
    {
        // one wave
        const auto v = [&](double u) { return acos(a / b * cos(u)); };
        const int initialSegments = 16;
        for (int c = 0; c <= initialSegments; ++c)
        {
            const double u = minU + 2 * M_PI * c / initialSegments;
            wave.emplace_back(u, v(u));
        }
        {
            // refine until within tolerance
            int current = 0;
            while (current + 1 < int(wave.size()))
            {
                const double u1 = wave[current].first;
                const double u2 = wave[current + 1].first;
                const double middleU = (u1 + u2) / 2;
                const double v1 = wave[current].second;
                const double v2 = wave[current + 1].second;
                const double middleV = v((u1 + u2) / 2);
                if (std::abs(middleV - (v1 + v2) / 2) > tolerance)
                    wave.emplace(wave.begin() + current + 1, middleU, middleV);
                else
                    ++current;
            }
        }
        // start from 1: the 0-th point is already duplicated as the last one of a period
        for (int c = 1; c < int(wave.size()) && wave.back().first < maxU; ++c)
            wave.emplace_back(wave[c].first + 2 * M_PI, wave[c].second);
    }
    for (double vShift = scaled_floor(minV, 2 * M_PI); vShift < maxV + 2 * M_PI; vShift += 2 * M_PI)
    {
        for (bool forwardRoot : {false, true})
        {
            result.emplace_back();
            for (const auto &pair : wave)
            {
                const double u = pair.first;
                double v = pair.second;
                v = (forwardRoot ? v : -v) + vShift;
                const double x = (u + v) / 2;
                const double y = (v - u) / 2 * (swapUV ? -1 : 1);
                result.back().points.emplace_back(coord_t(x * scaleFactor), coord_t(y * scaleFactor));
            }
        }
    }
    return result;
}

constexpr double FillTpmsD::PatternTolerance;

void FillTpmsD::_fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                                     const std::pair<float, Point> &direction, ExPolygon expolygon,
                                     Polylines &polylines_out)
{
    auto infill_angle = float(this->angle + (CorrectionAngle * 2 * M_PI) / 360.);
    if (std::abs(infill_angle) >= EPSILON)
        expolygon.rotate(-infill_angle);

    BoundingBox bb = expolygon.contour.bounding_box();
    // Density adjusted to have a good %of weight.
    double density_adjusted = std::max(0., params.density * DensityAdjust);
    // Distance between the waves in scaled coordinates.
    coord_t distance = coord_t(scale_(this->spacing) / density_adjusted);

    // align bounding box to a multiple of our grid module
    bb.merge(align_to_grid(bb.min, Point(2 * M_PI * distance, 2 * M_PI * distance)));

    // generate pattern
    Polylines polylines = make_tpmsd_waves(scale_(this->z), density_adjusted, this->spacing,
                                           ceil(bb.size()(0) / distance) + 1., ceil(bb.size()(1) / distance) + 1.);

    // shift the polyline to the grid origin
    for (Polyline &pl : polylines)
        pl.translate(bb.min);

    polylines = intersection_pl(polylines, expolygon);

    if (!polylines.empty())
    {
        // Remove very small bits, but be careful to not remove infill lines connecting thin walls!
        // The infill perimeter lines should be separated by around a single infill line width.
        const double minlength = scale_(0.8 * this->spacing);
        polylines.erase(std::remove_if(polylines.begin(), polylines.end(),
                                       [minlength](const Polyline &pl) { return pl.length() < minlength; }),
                        polylines.end());
    }

    if (!polylines.empty())
    {
        // connect lines
        size_t polylines_out_first_idx = polylines_out.size();
        if (params.dont_connect())
            append(polylines_out,
                   chain_polylines_by_region(std::move(polylines), coord_t(scale_(this->spacing) * 3 / params.density),
                                             params.start_near ? &(*params.start_near) : nullptr));
        else
            this->connect_infill(std::move(polylines), expolygon, polylines_out, this->spacing, params);

        // new paths must be rotated back
        if (std::abs(infill_angle) >= EPSILON)
        {
            for (auto it = polylines_out.begin() + polylines_out_first_idx; it != polylines_out.end(); ++it)
                it->rotate(infill_angle);
        }
    }
}

} // namespace Luminary
