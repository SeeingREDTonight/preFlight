///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/ TPMS-FK (Fischer-Koch S) infill, ported from OrcaSlicer (src/libslic3r/Fill/FillTpmsFK.*).
///|/
///|/ preFlight is based on PrusaSlicer and released under AGPLv3 or higher
///|/
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <vector>
#include <utility>

#include "luminary/geometry/clipper/ClipperUtils.hpp"
#include "MarchingSquares.hpp"
#include "luminary/geometry/primitives/Point.hpp"
#include "luminary/geometry/primitives/BoundingBox.hpp"
#include "luminary/geometry/contours/Polyline.hpp"
#include "luminary/geometry/contours/Polygon.hpp"
#include "luminary/core/parallel/ExecutionTBB.hpp"
#include "luminary/fill/contract/FillBase.hpp"
#include "luminary/geometry/contours/ExPolygon.hpp"
#include "luminary/core/Prelude.hpp"
#include "FillTpmsFK.hpp"

namespace Luminary
{

// Fischer - Koch S equation:
// cos(2x)sin(y)cos(z) + cos(2y)sin(z)cos(x) + cos(2z)sin(x)cos(y) = 0
static float fischer_koch(float x, float y, float z)
{
    return cosf(2 * x) * sinf(y) * cosf(z) + cosf(2 * y) * sinf(z) * cosf(x) + cosf(2 * z) * sinf(x) * cosf(y);
}

// Simplification tolerance for the traced lines (0.04 mm, as OrcaSlicer's sparse infill resolution).
static constexpr double TpmsFKResolution = 0.04 / SCALING_FACTOR;

} // namespace Luminary

namespace marchsq
{
using namespace Luminary;

using coordr_t = long; // length type for (r, c) raster coordinates.
using Pointf = Vec2d;  // (x, y) field point in coordf_t.

struct TpmsFKField
{
    static constexpr float gsizef = 0.40;                        // grid cell size in mm (roughly line segment length).
    static constexpr float rsizef = 0.004;                       // raster pixel size in mm (roughly point accuracy).
    const coord_t rsize = scaled(rsizef);                        // raster pixel size in coord_t.
    const coordr_t gsize = std::round(gsizef / rsizef);          // grid cell size in coordr_t.
    Point size;                                                  // field size in coord_t.
    Point offs;                                                  // field offset in coord_t.
    coordf_t z;                                                  // z offset as a float.
    float freq;                                                  // field frequency in cycles per mm.
    float isoval = 0.0;                                          // iso value threshold to use.

    explicit TpmsFKField(const BoundingBox bb, const coordf_t z = 0.0, const float period = 10.0)
        : size{bb.size()}, offs{bb.min}, z{z}, freq{float(2 * PI) / period}
    {
    }

    float get_scalar(coordf_t x, coordf_t y, coordf_t z) const { return fischer_koch(freq * x, freq * y, freq * z); }

    float get_scalar(Coord p) const
    {
        Pointf pf = to_Pointf(p);
        return get_scalar(pf.x(), pf.y(), z);
    }

    inline coord_t to_coord(const coordr_t &x) const { return x * rsize; }
    inline coordr_t to_coordr(const coord_t &x) const { return x / rsize; }

    inline Point to_Point(const Coord &p) const { return Point(to_coord(p.c) + offs.x(), to_coord(p.r) + offs.y()); }
    inline Coord to_Coord(const Point &p) const
    {
        return Coord(to_coordr(p.y() - offs.y()), to_coordr(p.x() - offs.x()));
    }
    inline Pointf to_Pointf(const Point &p) const { return Pointf(unscaled(p.x()), unscaled(p.y())); }
    inline Pointf to_Pointf(const Coord &p) const { return to_Pointf(to_Point(p)); }
};

// Register the field as a RasterType for MarchingSquares.
template<> struct _RasterTraits<TpmsFKField>
{
    using ValueType = float;
    static float get(const TpmsFKField &sf, size_t row, size_t col) { return sf.get_scalar(Coord(row, col)); }
    static size_t rows(const TpmsFKField &sf) { return sf.to_coordr(sf.size.y()); }
    static size_t cols(const TpmsFKField &sf) { return sf.to_coordr(sf.size.x()); }
};

// Polylines of the zero level set. Tolerance is the max line deviation from simplifying (scaled); -1 = off.
static Polylines tpmsfk_polylines(const TpmsFKField &sf, const double tolerance = SCALED_EPSILON)
{
    std::vector<Ring> rings = execute_with_policy(ex_tbb, sf, sf.isoval, {sf.gsize, sf.gsize});

    Polylines polys;
    polys.reserve(rings.size());
    for (const Ring &ring : rings)
    {
        Polyline poly;
        Points &pts = poly.points;
        pts.reserve(ring.size() + 1);
        for (const Coord &crd : ring)
            pts.emplace_back(sf.to_Point(crd));
        // MarchingSquares' rings are polygons: repeat the first point to make a closed polyline.
        pts.push_back(pts.front());
        if (tolerance >= 0.0)
            poly.simplify(tolerance);
        polys.emplace_back(poly);
    }
    return polys;
}

} // namespace marchsq

namespace Luminary
{

void FillTpmsFK::_fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                                      const std::pair<float, Point> &direction, ExPolygon expolygon,
                                      Polylines &polylines_out)
{
    auto infill_angle = float(this->angle + (CorrectionAngle * 2 * M_PI) / 360.);
    if (std::abs(infill_angle) >= EPSILON)
        expolygon.rotate(-infill_angle);

    // Field period adjusted to have a good %of weight.
    const float period = 4.18f * float(this->spacing) / std::min(0.9f, params.density);

    BoundingBox bbox = expolygon.contour.bounding_box();
    // Enlarge the bounding box to avoid artifacts at the edges.
    bbox.offset(scale_(2 * this->spacing));

    marchsq::TpmsFKField sf(bbox, this->z, period);
    Polylines polylines = marchsq::tpmsfk_polylines(sf, TpmsFKResolution);

    // Prune the lines within the expolygon.
    polylines = intersection_pl(polylines, expolygon);

    if (!polylines.empty())
    {
        // Remove very small bits, but be careful to not remove infill lines connecting thin walls!
        const double minlength = scale_(0.8 * this->spacing);
        polylines.erase(std::remove_if(polylines.begin(), polylines.end(),
                                       [minlength](const Polyline &pl) { return pl.length() < minlength; }),
                        polylines.end());
    }

    if (!polylines.empty())
    {
        size_t polylines_out_first_idx = polylines_out.size();
        // Chaining doesn't suit this pattern (internal islands, and slow): always connect.
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
