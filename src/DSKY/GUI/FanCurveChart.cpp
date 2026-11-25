///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ preFlight is based on PrusaSlicer and released under AGPLv3 or higher
///|/
#include "FanCurveChart.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <wx/dcbuffer.h>

#include "GUI_App.hpp"
#include "I18N.hpp"
#include "Widgets/UIColors.hpp"
#include "luminary/config/catalog/PrintConfig.hpp"

namespace DSKY
{

using namespace Luminary;

namespace
{

double cfg_float(const DynamicPrintConfig &config, const char *key, double def)
{
    const auto *opt = config.option<ConfigOptionFloats>(key);
    return opt != nullptr && !opt->values.empty() ? opt->values.front() : def;
}

int cfg_int(const DynamicPrintConfig &config, const char *key, int def)
{
    const auto *opt = config.option<ConfigOptionInts>(key);
    return opt != nullptr && !opt->values.empty() ? opt->values.front() : def;
}

bool cfg_bool(const DynamicPrintConfig &config, const char *key, bool def)
{
    const auto *opt = config.option<ConfigOptionBools>(key);
    return opt != nullptr && !opt->values.empty() ? bool(opt->values.front()) : def;
}

} // namespace

FanCurveChart::FanCurveChart(wxWindow *parent)
    : wxWindow(parent, wxID_ANY, wxDefaultPosition,
               wxSize(46 * wxGetApp().em_unit(), 20 * wxGetApp().em_unit()))
{
    SetBackgroundStyle(wxBG_STYLE_PAINT);
    SetMinSize(GetSize());
    Bind(wxEVT_PAINT, &FanCurveChart::on_paint, this);
}

FanCurve FanCurveChart::curve_from_config(const DynamicPrintConfig &config)
{
    FanCurve c;
    const ConfigOption *mode = config.option("fan_control_mode");
    // The edited preset holds the generic enum vector, read through getInts()
    switch (mode != nullptr && !mode->getInts().empty() ? static_cast<FanControlMode>(mode->getInts().front())
                                                        : FanControlMode::Legacy)
    {
    case FanControlMode::Curve:
        c.mode = FanCurveMode::Curve;
        break;
    case FanControlMode::Breakpoints:
        c.mode = FanCurveMode::Breakpoints;
        break;
    default:
        c.mode = FanCurveMode::Legacy;
        break;
    }
    c.fan_always_on = cfg_bool(config, "fan_always_on", false);
    c.min_speed = cfg_int(config, "min_fan_speed", 0);
    c.max_speed = cfg_int(config, "max_fan_speed", 100);
    c.legacy_fan_below = cfg_int(config, "fan_below_layer_time", 60);
    c.legacy_slowdown_below = cfg_int(config, "slowdown_below_layer_time", 5);
    c.start_time = cfg_float(config, "fan_curve_start_time", 3.);
    c.full_time = cfg_float(config, "fan_curve_full_time", 0.25);
    c.shape = cfg_float(config, "fan_curve_shape", 1.);
    c.points.clear();
    for (int i = 0; i < 6; ++i)
    {
        const std::string n = std::to_string(i);
        c.points.emplace_back(cfg_float(config, ("fan_curve_time_" + n).c_str(), 0.),
                              cfg_int(config, ("fan_curve_speed_" + n).c_str(), 0));
    }
    return c;
}

void FanCurveChart::update_from_config(const DynamicPrintConfig &config)
{
    m_curve = curve_from_config(config);
    m_slowdown_time = cfg_int(config, "slowdown_below_layer_time", 0);
    m_cooling = cfg_bool(config, "cooling", true);
    m_manual_fan = cfg_bool(config, "enable_manual_fan_speeds", false);
    Refresh();
}

void FanCurveChart::on_paint(wxPaintEvent &)
{
    wxAutoBufferedPaintDC dc(this);
    const int em = wxGetApp().em_unit();
    const wxColour fg = UIColors::ContentForeground();
    const wxColour muted = UIColors::SecondaryText();
    const wxColour plot_bg = UIColors::InputBackground();
    const bool active = m_cooling && !m_manual_fan;
    const wxColour line_clr = active ? UIColors::AccentPrimary() : muted;

    dc.SetBackground(wxBrush(GetParent()->GetBackgroundColour()));
    dc.Clear();
    dc.SetFont(GetFont());

    const wxSize size = GetClientSize();
    const int left = 5 * em, right = size.x - em, top = em, bottom = size.y - 4 * em;
    if (right - left < 4 * em || bottom - top < 4 * em)
        return;

    // Time range: wide enough for every threshold the mode uses, on a log axis.
    double t_lo = 0.05, t_hi = 10.;
    switch (m_curve.mode)
    {
    case FanCurveMode::Legacy:
        t_lo = std::min(t_lo, 0.5 * std::max(0.01, m_curve.legacy_slowdown_below));
        t_hi = std::max(t_hi, 1.5 * m_curve.legacy_fan_below);
        break;
    case FanCurveMode::Curve:
        t_lo = std::min(t_lo, 0.5 * std::max(0.001, m_curve.full_time));
        t_hi = std::max(t_hi, 2. * m_curve.start_time);
        break;
    case FanCurveMode::Breakpoints:
        for (const auto &p : m_curve.sorted_points())
        {
            t_lo = std::min(t_lo, 0.5 * p.first);
            t_hi = std::max(t_hi, 2. * p.first);
        }
        break;
    }
    if (m_slowdown_time > 0.)
        t_hi = std::max(t_hi, 1.5 * m_slowdown_time);
    t_lo = std::max(t_lo, 0.005);
    t_hi = std::min(t_hi, 1000.);
    const double lg_lo = std::log(t_lo), lg_hi = std::log(t_hi);

    // Sample the curve; the fan axis tops out a little above its highest value.
    const int samples = std::max(50, right - left);
    std::vector<double> ts(samples + 1), fans(samples + 1);
    double fan_peak = 0.;
    for (int i = 0; i <= samples; ++i)
    {
        ts[i] = std::exp(lg_lo + (lg_hi - lg_lo) * double(i) / samples);
        fans[i] = m_curve.eval(ts[i]);
        fan_peak = std::max(fan_peak, fans[i]);
    }
    const double fan_top = std::clamp(std::ceil((fan_peak + 5.) / 10.) * 10., 20., 100.);

    auto px = [&](double t) { return left + int(std::lround((std::log(t) - lg_lo) / (lg_hi - lg_lo) * (right - left))); };
    auto py = [&](double f) { return bottom - int(std::lround(f / fan_top * (bottom - top))); };

    // Plot area and grid.
    dc.SetPen(wxPen(muted));
    dc.SetBrush(wxBrush(plot_bg));
    dc.DrawRectangle(left, top, right - left + 1, bottom - top + 1);
    dc.SetTextForeground(fg);
    const double y_step = fan_top <= 40. ? 5. : fan_top <= 80. ? 10. : 20.;
    for (double f = 0.; f <= fan_top + 1e-6; f += y_step)
    {
        const int y = py(f);
        dc.SetPen(wxPen(muted, 1, wxPENSTYLE_DOT));
        if (f > 0. && f < fan_top)
            dc.DrawLine(left, y, right, y);
        const wxString label = wxString::Format("%g", f);
        const wxSize ext = dc.GetTextExtent(label);
        dc.DrawText(label, left - ext.x - em / 2, y - ext.y / 2);
    }
    static const double ticks[] = {0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1., 2., 5., 10., 20., 50., 100., 200., 500., 1000.};
    for (double t : ticks)
    {
        if (t < t_lo * 0.999 || t > t_hi * 1.001)
            continue;
        const int x = px(t);
        dc.SetPen(wxPen(muted, 1, wxPENSTYLE_DOT));
        dc.DrawLine(x, top, x, bottom);
        const wxString label = wxString::Format("%g", t);
        const wxSize ext = dc.GetTextExtent(label);
        dc.DrawText(label, x - ext.x / 2, bottom + em / 3);
    }

    // The slowdown threshold: below it the layer is stretched to this time.
    if (m_slowdown_time > 0. && m_slowdown_time >= t_lo && m_slowdown_time <= t_hi)
    {
        const int x = px(m_slowdown_time);
        dc.SetPen(wxPen(UIColors::Warning(), 1, wxPENSTYLE_SHORT_DASH));
        dc.DrawLine(x, top, x, bottom);
        dc.SetTextForeground(UIColors::Warning());
        dc.DrawText(_(L("slowdown")), x + em / 3, top + em / 4);
    }

    // The curve.
    std::vector<wxPoint> pts;
    pts.reserve(samples + 1);
    for (int i = 0; i <= samples; ++i)
        pts.emplace_back(px(ts[i]), py(fans[i]));
    dc.SetPen(wxPen(line_clr, std::max(2, em / 5)));
    dc.DrawLines(int(pts.size()), pts.data());
    if (m_curve.mode == FanCurveMode::Breakpoints)
    {
        dc.SetBrush(wxBrush(line_clr));
        dc.SetPen(wxPen(line_clr));
        for (const auto &p : m_curve.sorted_points())
            if (p.first >= t_lo && p.first <= t_hi)
                dc.DrawCircle(px(p.first), py(p.second), std::max(3, em * 2 / 5));
    }

    // Axis titles and the note on which layer time the mode reads.
    dc.SetTextForeground(fg);
    const wxString x_title = m_curve.mode == FanCurveMode::Legacy ? _(L("Layer time after slowdown (s)"))
                                                                   : _(L("Layer time before slowdown (s)"));
    const wxSize x_ext = dc.GetTextExtent(x_title);
    dc.DrawText(x_title, (left + right - x_ext.x) / 2, bottom + em / 3 + x_ext.y + em / 4);
    dc.DrawRotatedText(_(L("Fan %")), 0, (top + bottom) / 2 + dc.GetTextExtent(_(L("Fan %"))).x / 2, 90.);

    if (!active)
    {
        const wxString note = !m_cooling ? _(L("Automatic cooling is off")) : _(L("Manual fan speeds set the fan"));
        const wxSize ext = dc.GetTextExtent(note);
        dc.SetTextForeground(muted);
        dc.DrawText(note, (left + right - ext.x) / 2, (top + bottom - ext.y) / 2);
    }
}

} // namespace DSKY
