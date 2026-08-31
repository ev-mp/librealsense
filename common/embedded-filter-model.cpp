// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <rsutils/easylogging/easyloggingpp.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>
#include "subdevice-model.h"
#include "embedded-filter-model.h"
#include "textual-icons.h"
#include "viewer.h"


namespace rs2
{
    namespace
    {
        // Logs every field read back from FW, only on real GETs (not every-frame re-reads).
        void print_minz_control( const rs2_minz_control & v )
        {
            LOG_DEBUG( "[MinZ GET] version=" << (int)v.version
                       << " flags=" << (int)v.flags
                       << " ctl_id=0x" << std::hex << v.ctl_id << std::dec
                       << " param_count=" << (int)v.param_count
                       << " param_type=" << (int)v.param_type
                       << " enable=" << v.enable
                       << " filter_type=" << v.filter_type
                       << " downscale_ratio=" << v.downscale_ratio
                       << " shift_mode=" << v.shift_mode
                       << " shift_pixels=" << v.shift_pixels
                       << " threshold_mode=" << v.threshold_mode
                       << " threshold_mm=" << v.threshold_mm
                       << " reserved=[" << v.reserved[0] << "]" );
        }

        // Ties the manual-entry pencil icon and the number it opens together visually - the same
        // color on both is what says "these two are one mode," not just proximity.
        const ImVec4 minz_manual_edit_color( 1.0f, 0.65f, 0.0f, 1.0f );

        // ImGui has no native "named enum" slider - a plain SliderInt whose format string is the
        // value's display name (instead of "%d") is the standard idiom. `names[0]` == `min_v`.
        bool slider_enum( const char * str_id, int * value, int min_v, int max_v, const char * const names[] )
        {
            std::string fmt = names[ *value - min_v ];
            return ImGui::SliderInt( str_id, value, min_v, max_v, fmt.c_str() );
        }
    }

    embedded_filter_model::embedded_filter_model(
        subdevice_model* owner,
        const rs2_embedded_filter_type& type,
        std::shared_ptr<rs2::embedded_filter> filter,
        viewer_model& viewer,
        std::string& error_message)
        : _embedded_filter(filter), _viewer(viewer), _destructing(false)
    {
        _name = rs2_embedded_filter_type_to_string(type);

        std::stringstream ss;
        ss << "##" << ((owner) ? owner->dev.get_info(RS2_CAMERA_INFO_NAME) : _name)
            << "/" << ((owner) ? (*owner->s).get_info(RS2_CAMERA_INFO_NAME) : "_")
            << "/" << (long long)this;

        // following method also updates the data member "_enabled"
        populate_options(ss.str().c_str(), owner, owner ? &owner->_options_invalidated : nullptr, error_message);
    }


    embedded_filter_model::~embedded_filter_model()
    {
        _destructing.store(true);
        try
        {
            _embedded_filter->on_options_changed([](const options_list& list) {});
        }
        catch (...)
        {
        }
    }

    void embedded_filter_model::draw_options( viewer_model & viewer,
                                               bool update_read_only_options,
                                               bool is_streaming,
                                               std::string & error_message )
    {
        for (auto& id_and_model : _options_id_to_model)
        {
            if( id_and_model.first == RS2_OPTION_EMBEDDED_FILTER_ENABLED )
                continue;

            id_and_model.second.draw_option( update_read_only_options, is_streaming, error_message, *viewer.not_model );
        }

        // Composite options have no generic per-field editing UI (see embedded-filter-model.h) -
        // RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL gets a hardcoded editor below; everything else
        // just shows read-only metadata (description, byte size).
        for( auto id : _composite_option_ids )
        {
            try
            {
                if( id == RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL )
                {
                    // No always-visible description line here - draw_minz_control_editor shows
                    // it as a tooltip on hovering the framed control instead, to save vertical
                    // space in this narrow side panel.
                    draw_minz_control_editor( error_message );
                    continue;
                }

                // TextWrapped, not TextDisabled - this side panel is narrow enough that a
                // one-line description reliably clips instead of just looking dim.
                ImGui::PushStyleColor( ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled] );
                ImGui::TextWrapped( "%s", _embedded_filter->get_composite_option_description( id ) );
                ImGui::PopStyleColor();

                auto bytes = _embedded_filter->get_composite_option( id );
                ImGui::TextDisabled( "  (%zu bytes, composite option - no generic editor yet)", bytes.size() );
            }
            catch( const std::exception& e )
            {
                error_message = e.what();
            }
        }
    }

    void embedded_filter_model::commit_slider_enum_edit( bool changed )
    {
        if( changed )
            _minz_editor.touch();
        if( ImGui::IsItemActive() )
            _minz_editor.touch();
        if( ImGui::IsItemDeactivatedAfterEdit() )
            _minz_editor.finalize();
    }

    // Left/Right-arrow nudge for a focused-but-not-dragging slider-enum.
    void embedded_filter_model::slider_enum_arrow_nudge( int & field, int min_v, int max_v )
    {
        if( ! ImGui::IsItemFocused() || ImGui::IsItemActive() )
            return;
        if( ImGui::IsKeyPressed( ImGuiKey_RightArrow ) && field < max_v )
        {
            ++field;
            _minz_editor.touch();
            _minz_editor.finalize( true );
        }
        else if( ImGui::IsKeyPressed( ImGuiKey_LeftArrow ) && field > min_v )
        {
            --field;
            _minz_editor.touch();
            _minz_editor.finalize( true );
        }
    }

    bool embedded_filter_model::draw_minz_filter_type_field()
    {
        static const char * const names[] = { "Downscale", "Lookup Shift" };
        ImGui::Text( "Filter Type:" );
        int v = _minz_editor.value.filter_type;
        bool changed = slider_enum( "##minz_filter_type", &v, 0, 1, names );
        if( changed )
            _minz_editor.value.filter_type = v;
        commit_slider_enum_edit( changed );
        slider_enum_arrow_nudge( _minz_editor.value.filter_type, 0, 1 );
        return ImGui::IsItemActive();
    }

    bool embedded_filter_model::draw_minz_downscale_ratio_field()
    {
        // Downscale: only the ratio matters; value 0 is reserved (not a legal wire value), so
        // the slider's range starts at 1.
        static const char * const names[] = { "x2", "x4" };
        ImGui::Text( "Downscale Ratio:" );
        int v = _minz_editor.value.downscale_ratio;
        bool changed = slider_enum( "##minz_downscale_ratio", &v, 1, 2, names );
        if( changed )
            _minz_editor.value.downscale_ratio = v;
        commit_slider_enum_edit( changed );
        slider_enum_arrow_nudge( _minz_editor.value.downscale_ratio, 1, 2 );
        return ImGui::IsItemActive();
    }

    bool embedded_filter_model::draw_minz_shift_mode_field()
    {
        // Lookup Shift: pick a fixed preset, or Manual - which draw_minz_control_editor() reveals
        // via a separate draw_minz_manual_editable_field() call for Shift Pixels.
        static const char * const names[] = { "Shift 126px", "Shift 64px", "Manual" };
        ImGui::Text( "Shift Mode:" );
        int v = _minz_editor.value.shift_mode;
        bool changed = slider_enum( "##minz_shift_mode", &v, 0, 2, names );
        if( changed )
            _minz_editor.value.shift_mode = v;
        commit_slider_enum_edit( changed );
        slider_enum_arrow_nudge( _minz_editor.value.shift_mode, 0, 2 );
        return ImGui::IsItemActive();
    }

    bool embedded_filter_model::draw_minz_threshold_mode_field()
    {
        // Zero range / MinZ (firmware-computed) / Manual. Per the design review, the FW-computed
        // MinZ value itself is NOT surfaced to the user at this stage - "MinZ (computed)" reveals
        // no readback field at all, unlike "Manual" (see draw_minz_manual_editable_field()).
        static const char * const names[] = { "Zero range", "MinZ (computed)", "Manual" };
        ImGui::Text( "Threshold Mode:" );
        int v = _minz_editor.value.threshold_mode;
        bool changed = slider_enum( "##minz_threshold_mode", &v, 0, 2, names );
        if( changed )
            _minz_editor.value.threshold_mode = v;
        commit_slider_enum_edit( changed );
        slider_enum_arrow_nudge( _minz_editor.value.threshold_mode, 0, 2 );
        bool active = ImGui::IsItemActive();
        if( ImGui::IsItemHovered() )
            ImGui::SetTooltip( "Zero range: fill only originally-empty depth pixels.\n"
                                "MinZ (computed): firmware picks the threshold for the active resolution.\n"
                                "Manual: use the threshold value below." );
        return active;
    }

    // The InputText half of draw_minz_manual_editable_field(): a narrow, centered box seeded
    // with the current value; Enter parses, clamps, and commits it, then flips back to slider mode.
    bool embedded_filter_model::draw_minz_manual_input( const char * id, int & value, int min_v, int max_v,
                                                          bool & edit_mode, std::string & edit_buf )
    {
        char buf[32] = {};
        strncpy( buf, edit_buf.c_str(), sizeof( buf ) - 1 );

        // Narrow and centered, highlighted like the pencil icon beside it, so it reads as "a
        // small typed value" distinct from the wide slider it stands in for.
        float avail_width = ImGui::GetContentRegionAvail().x;
        float input_width = ImGui::CalcTextSize( "000000" ).x + ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SetCursorPosX( ImGui::GetCursorPosX() + std::max( 0.0f, ( avail_width - input_width ) * 0.5f ) );
        ImGui::PushItemWidth( input_width );
        ImGui::PushStyleColor( ImGuiCol_Text, minz_manual_edit_color );
        std::string input_id = rsutils::string::from() << "##" << id << "_input";
        bool submitted = ImGui::InputText( input_id.c_str(), buf, sizeof( buf ),
                                            ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_EnterReturnsTrue );
        ImGui::PopStyleColor();
        ImGui::PopItemWidth();

        if( submitted )
        {
            char * end = nullptr;
            long parsed = std::strtol( buf, &end, 10 );
            if( end != buf )
            {
                value = (int)std::min( std::max( parsed, (long)min_v ), (long)max_v );
                _minz_editor.touch();
                _minz_editor.finalize();
            }
            edit_mode = false;
        }
        else
            edit_buf = buf;

        if( ImGui::IsItemActive() )
            _minz_editor.touch();
        return ImGui::IsItemActive();
    }

    // The slider half of draw_minz_manual_editable_field(). ImGui sliders don't nudge via arrow
    // keys on their own, so that's implemented by hand below.
    bool embedded_filter_model::draw_minz_slider_with_arrows( const char * id, int & value, int min_v, int max_v )
    {
        std::string slider_id = rsutils::string::from() << "##" << id;
        if( ImGui::SliderInt( slider_id.c_str(), &value, min_v, max_v ) )
            _minz_editor.touch();
        if( ImGui::IsItemActive() )
            _minz_editor.touch();
        if( ImGui::IsItemDeactivatedAfterEdit() )
            _minz_editor.finalize();
        else if( ImGui::IsItemFocused() && ! ImGui::IsItemActive() )
        {
            if( ImGui::IsKeyPressed( ImGuiKey_RightArrow ) )
            {
                value = std::min( value + 1, max_v );
                _minz_editor.touch();
                _minz_editor.finalize( true );   // arrow-key nudge - fast turnaround
            }
            else if( ImGui::IsKeyPressed( ImGuiKey_LeftArrow ) )
            {
                value = std::max( value - 1, min_v );
                _minz_editor.touch();
                _minz_editor.finalize( true );   // arrow-key nudge - fast turnaround
            }
        }
        return ImGui::IsItemActive();
    }

    // "label + pencil-toggle + (slider OR manual InputText)" - shared by Shift Pixels and
    // Threshold (mm), which otherwise repeated this pattern twice.
    bool embedded_filter_model::draw_minz_manual_editable_field( const char * label, const char * id, int & value,
                                                                   int min_v, int max_v, bool & edit_mode,
                                                                   std::string & edit_buf )
    {
        ImGui::Text( "%s", label );
        ImGui::SameLine();
        {
            std::string edit_id = rsutils::string::from() << textual_icons::edit << "##" << id << "_edit";
            if( edit_mode )
                ImGui::PushStyleColor( ImGuiCol_Text, minz_manual_edit_color );
            if( ImGui::SmallButton( edit_id.c_str() ) )
            {
                if( ! edit_mode )
                    edit_buf = std::to_string( value );
                edit_mode = ! edit_mode;
            }
            if( edit_mode )
                ImGui::PopStyleColor();
            if( ImGui::IsItemHovered() )
                ImGui::SetTooltip( edit_mode ? "Back to slider" : "Type an exact value" );
        }

        return edit_mode ? draw_minz_manual_input( id, value, min_v, max_v, edit_mode, edit_buf )
                          : draw_minz_slider_with_arrows( id, value, min_v, max_v );
    }

    // Reset to Default hides behind a small "..." marker in the box's bottom-right corner until
    // the mouse is nearby, so this rarely-used action doesn't compete for attention. Drawn as an
    // absolute-position overlay so its presence doesn't shift the fields above it.
    bool embedded_filter_model::draw_minz_reset_to_default_overlay( rs2_composite_option_id id,
                                                                      std::string & error_message,
                                                                      float frame_max_x,
                                                                      float frame_max_y )
    {
        bool any_active = false;
        ImVec2 frame_max( frame_max_x, frame_max_y );
        ImVec2 saved_cursor = ImGui::GetCursorScreenPos();

        ImVec2 button_size = ImGui::CalcTextSize( "Reset to Default" );
        button_size.x += ImGui::GetStyle().FramePadding.x * 2.0f;
        button_size.y += ImGui::GetStyle().FramePadding.y * 2.0f;
        ImVec2 button_pos( frame_max.x - button_size.x - 4.0f, frame_max.y - button_size.y - 4.0f );

        constexpr float reveal_margin = 24.0f;
        ImVec2 mouse = ImGui::GetIO().MousePos;
        bool nearby = mouse.x >= button_pos.x - reveal_margin && mouse.x <= frame_max.x + reveal_margin
                   && mouse.y >= button_pos.y - reveal_margin && mouse.y <= frame_max.y + reveal_margin;

        if( nearby )
        {
            // Goes through the same touch()/finalize() pipeline as every other field edit, so it
            // gets the same fade animation and undo grace window rather than a separate commit path.
            ImGui::SetCursorScreenPos( button_pos );
            if( ImGui::Button( "Reset to Default##minz" ) )
            {
                try
                {
                    auto range = _embedded_filter->get_composite_option_range_as< rs2_minz_control_range >( id );
                    _minz_editor.value = range.def;
                    _minz_editor.touch();
                    _minz_editor.finalize();
                }
                catch( const std::exception & e )
                {
                    error_message = e.what();
                }
            }
            any_active = ImGui::IsItemActive();
            if( ImGui::IsItemHovered() )
                ImGui::SetTooltip( "Restore all fields to the firmware-reported default values" );
        }
        else
        {
            ImVec2 marker_size = ImGui::CalcTextSize( "..." );
            ImGui::SetCursorScreenPos( ImVec2( frame_max.x - marker_size.x - 8.0f, frame_max.y - marker_size.y - 6.0f ) );
            ImGui::TextDisabled( "..." );
        }

        ImGui::SetCursorScreenPos( saved_cursor );
        return any_active;
    }

    void embedded_filter_model::draw_minz_control_editor( std::string & error_message )
    {
        const auto id = RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL;

        if( ! _minz_editor.ensure_initialized( _embedded_filter, id, error_message, print_minz_control ) )
            return;

        // ImGui::Indent(w) ADDS w; Unindent(w) SUBTRACTS w - they only cancel out when passed the
        // SAME w, so Unindent(-5.f) below (not e.g. Unindent(4.f)) is what undoes this.
        ImGui::Indent( -5.f );
        ImGui::Dummy( ImVec2( 0, 2 ) );

        float frame_left = ImGui::GetCursorScreenPos().x - 4.0f;
        float frame_top = ImGui::GetCursorScreenPos().y - 4.0f;
        float frame_width = ImGui::GetContentRegionAvail().x -30.0f;

        // Row header's own toggle drives `enable` directly. Grey the box out while off (global
        // alpha, not BeginDisabled, so fields stay clickable and re-enable on edit).
        const bool dim_while_disabled = ! _enabled;
        if( dim_while_disabled )
            ImGui::PushStyleVar( ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.35f );

        // |= not ||: each call below draws a real widget as a side effect and must run every
        // frame regardless of the flag accumulated so far.
        bool any_field_active = false;
        any_field_active |= draw_minz_filter_type_field();

        if( _minz_editor.value.filter_type == 0 )
        {
            any_field_active |= draw_minz_downscale_ratio_field();
        }
        else
        {
            any_field_active |= draw_minz_shift_mode_field();
            if( _minz_editor.value.shift_mode == 2 )
                any_field_active |= draw_minz_manual_editable_field( "Shift Pixels:", "minz_shift",
                    _minz_editor.value.shift_pixels, 0, 256, _minz_shift_edit_mode, _minz_shift_edit_buf );
        }

        any_field_active |= draw_minz_threshold_mode_field();
        if( _minz_editor.value.threshold_mode == 2 )
            any_field_active |= draw_minz_manual_editable_field( "Threshold (mm):", "minz_threshold",
                _minz_editor.value.threshold_mm, 0, 65535, _minz_threshold_edit_mode, _minz_threshold_edit_buf );

        ImGui::Dummy( ImVec2( 0, 2 ) );
        float frame_bottom = ImGui::GetCursorScreenPos().y;
        ImVec2 frame_min( frame_left, frame_top );
        ImVec2 frame_max( frame_left + frame_width, frame_bottom );

        // No line of its own is reserved for the reset overlay, so it may partially overlap the
        // last field's row rather than pushing the box taller.
        any_field_active |= draw_minz_reset_to_default_overlay( id, error_message, frame_max.x, frame_max.y );

        // before_commit forces enable on: with no Enable checkbox in here, touching e.g.
        // Threshold alone would otherwise send with enable unchanged (often off).
        _minz_editor.end_frame_and_maybe_commit( _embedded_filter, id, error_message, frame_min, frame_max, any_field_active,
            [this]( rs2_minz_control & v )
            {
                v.enable = 1;
                _enabled = true;
            } );

        if( dim_while_disabled )
            ImGui::PopStyleVar();

        ImGui::Unindent( -5.f );   // undo Indent(-5.f) above, exactly - see the comment there
    }

    void embedded_filter_model::embedded_filter_enable_disable(bool actual)
    {
        // Composite-only filters register no scalar RS2_OPTION_EMBEDDED_FILTER_ENABLED - route
        // the toggle through the composite `enable` field instead, read-modify-write.
        if( _embedded_filter->supports_composite_option( RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL ) )
        {
            try
            {
                _minz_editor.value = _embedded_filter->get_composite_option_as< rs2_minz_control >(
                    RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL );
                print_minz_control( _minz_editor.value );
                _minz_editor.value.enable = actual ? 1 : 0;
                _embedded_filter->set_composite_option_from( RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL, _minz_editor.value );
                _minz_editor.initialized = true;
                _enabled = actual;
            }
            catch( const std::exception & )
            {
                // Leave _enabled as it was - the toggle stays in its last known-good state
                // rather than claiming a change happened when the device rejected it.
            }
            return;
        }

        if( ! _embedded_filter->supports( RS2_OPTION_EMBEDDED_FILTER_ENABLED ) )
            return;
        _embedded_filter->set_option(RS2_OPTION_EMBEDDED_FILTER_ENABLED, actual ? 1.0f : 0.0f);
        _enabled = _embedded_filter->get_option(RS2_OPTION_EMBEDDED_FILTER_ENABLED);
    }

    void embedded_filter_model::populate_options(const std::string& opt_base_label,
        subdevice_model* model,
        bool* options_invalidated,
        std::string& error_message)
    {
        // Pass 1: regular (scalar) options - own registry, own loop. DDS-based embedded filters
        // (see src/dds/rs-dds-embedded-*-filter.cpp) still register RS2_OPTION_EMBEDDED_FILTER_ENABLED
        // as a genuine scalar option here, so this stays the primary source of _enabled whenever
        // it's present.
        for (option_value option : _embedded_filter->get_supported_option_values())
        {
            // Build the model first and insert only on success: an option whose range cannot be read
            // throws, and map::operator[] would leave a default-constructed (null-endpoint) entry
            // behind. Isolate per option so one bad control does not drop the rest.
            try
            {
                auto om = create_option_model( option,
                                               opt_base_label,
                                               model,
                                               _embedded_filter,
                                               model ? &model->_options_invalidated : nullptr,
                                               error_message );
                _options_id_to_model[option->id] = std::move( om );
            }
            catch( const std::exception & e )
            {
                if( _viewer.not_model )
                    _viewer.not_model->add_log( e.what(), RS2_LOG_SEVERITY_WARN );
            }
        }
        if( _embedded_filter->supports( RS2_OPTION_EMBEDDED_FILTER_ENABLED ) )
            _enabled = _embedded_filter->get_option(RS2_OPTION_EMBEDDED_FILTER_ENABLED);

        // Pass 2: composite options - a completely separate registry from scalar rs2_option (see
        // rs_composite_option.h), enumerated and primed in its own loop rather than folded into
        // pass 1 above.
        _composite_option_ids = _embedded_filter->get_supported_composite_options();
        for( auto id : _composite_option_ids )
        {
            // No generic per-composite-option editor exists yet (see draw_options()) - MinZ is
            // the one hardcoded case, and the only one with an `enable` field to prime here.
            if( id != RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL )
                continue;
            if( ! _embedded_filter->supports_composite_option( id ) )
                continue;

            if( _minz_editor.ensure_initialized( _embedded_filter, id, error_message, print_minz_control ) )
            {
                // Composite state is the FALLBACK source of _enabled, not an override - a DDS
                // filter that already set it from the scalar option in pass 1 keeps that value.
                if( ! _embedded_filter->supports( RS2_OPTION_EMBEDDED_FILTER_ENABLED ) )
                    _enabled = _minz_editor.value.enable != 0;
            }
        }

        try
        {
            _embedded_filter->on_options_changed([this](const options_list& list)
                {
                    for (auto changed_option : list)
                    {
                        auto it = _options_id_to_model.find(changed_option->id);
                        // Callback runs in different context, checking _options_id_to_model still valid
                        if (it != _options_id_to_model.end() && !_destructing)
                        {
                            it->second.update_value(changed_option, *_viewer.not_model);
                            if (it->first == RS2_OPTION_EMBEDDED_FILTER_ENABLED)
                            {
                                // rs2_option_value is a union over as_float/as_integer. For a FLOAT
                                // option, only as_float is initialized - reading as_integer would
                                // pick up the uninitialized upper 4 bytes (int64_t vs float).
                                if (changed_option->is_valid)
                                    _enabled = (changed_option->type == RS2_OPTION_TYPE_FLOAT)
                                             ? (changed_option->as_float != 0.0f)
                                             : (changed_option->as_integer != 0);
                            }
                        }
                    }
                });
        }
        catch (const std::exception& e)
        {
            if (_viewer.not_model)
                _viewer.not_model->add_log(e.what(), RS2_LOG_SEVERITY_WARN);
        }
    }
}
