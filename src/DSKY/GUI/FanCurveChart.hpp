///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ preFlight is based on PrusaSlicer and released under AGPLv3 or higher
///|/
#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include "luminary/gcode/filters/FanCurve.hpp"

namespace Luminary
{
class DynamicPrintConfig;
}

namespace DSKY
{

// Read-only preview of the filament's layer-time fan curve on the Cooling page: fan % over the
// layer time on a logarithmic axis, drawn with the same function the cooling filter uses.
class FanCurveChart : public wxWindow
{
public:
    FanCurveChart(wxWindow *parent);

    // Rebuilds the curve from the edited filament config (first extruder) and repaints.
    void update_from_config(const Luminary::DynamicPrintConfig &config);
    void sys_color_changed() { Refresh(); }

    // The curve the filament config describes, as the cooling filter builds it.
    static Luminary::FanCurve curve_from_config(const Luminary::DynamicPrintConfig &config);

private:
    void on_paint(wxPaintEvent &);

    Luminary::FanCurve m_curve;
    double m_slowdown_time{0.};
    bool m_cooling{true};
    bool m_manual_fan{false};
};

} // namespace DSKY
