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
        // DEBUG: every field actually read back from FW for RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL -
        // called right after each real GET (initial seed and the toggle's read-modify-write), never
        // on every-frame no-op re-reads, so this reflects only genuine device round-trips.
        // LOG_DEBUG (not std::cout) so this only emits when the app's configured log verbosity is
        // DEBUG or more verbose, not at the default INFO level.
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

        // ImGui has no native "named enum" slider - the standard idiom (see e.g.
        // https://pthom.github.io/imgui_explorer's SliderInt-with-steps examples) is a plain
        // SliderInt whose format string is the current value's display name instead of a "%d"
        // conversion, so it prints literally instead of the raw integer while dragging still
        // snaps to each integer position. `names[0]` corresponds to `min_v`. Because the format
        // string is built from the value going INTO this call, the label lags the drag by one
        // frame (updates next frame once *value has changed) - imperceptible in practice and the
        // same tradeoff every SliderInt-with-format idiom makes.
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

    void embedded_filter_model::draw_minz_control_editor( std::string & error_message )
    {
        const auto id = RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL;

        if( ! _minz_editor.ensure_initialized( _embedded_filter, id, error_message, print_minz_control ) )
            return;

        // Minimal indent - just enough padding that widget text doesn't sit flush on the frame
        // border - rather than the tree's full default indent, so the group sits as far left as
        // the panel allows.
        ImGui::Indent( -5.f );
        ImGui::Dummy( ImVec2( 0, 2 ) );

        float frame_left = ImGui::GetCursorScreenPos().x - 4.0f;
        float frame_top = ImGui::GetCursorScreenPos().y - 4.0f;
        float frame_width = ImGui::GetContentRegionAvail().x -30.0f;

        // No separate Enable checkbox here - the row header's own blue toggle (see
        // device-model.cpp's draw_embedded_filters()/embedded_filter_enable_disable() below)
        // drives rs2_minz_control::enable directly; a second checkbox for the same field would
        // be redundant.

        // Grey the whole box out while the toggle reads off - purely visual (global alpha, not
        // BeginDisabled) so fields stay clickable: editing any of them while off still forces
        // enable back on at commit time (see before_commit below), which is the intended way to
        // turn this control on from inside the box itself.
        const bool dim_while_disabled = ! _enabled;
        if( dim_while_disabled )
            ImGui::PushStyleVar( ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.35f );

        // Tracked across every widget below so end_frame_and_maybe_commit() can tell "nothing of
        // ours is active right now" (the normal, harmless gap between finishing one field and
        // touching the next) apart from "focus genuinely left the group" (some OTHER widget
        // elsewhere is active) - only the latter should cut the countdown short.
        bool any_field_active = false;

        // Every label below sits on its own line, with the interactive widget using a "##"-only
        // (invisible) label - this side panel is too narrow for "label: [====slider====] value"
        // all on one line; ImGui doesn't wrap inline widget labels, it just clips them.
        //
        // Per the design-review struct (see rs_hkr_minz_control.h), every mode/enum field below
        // - filter_type, downscale_ratio, shift_mode, threshold_mode - is rendered as a "slider
        // enum": a plain ImGui::SliderInt whose displayed text is the value's name instead of its
        // raw number (see slider_enum() above), replacing the old radio-button/checkbox widgets.
        // This also generalizes the earlier per-field manual-entry escape hatch (pencil icon +
        // InputText, still used below for Shift Pixels / Threshold (mm)): selecting the "Manual"
        // rung of shift_mode or threshold_mode is itself now what reveals that field, rather than
        // a separate checkbox.
        auto slider_enum_commit = [ this ]( bool changed )
        {
            if( changed )
                _minz_editor.touch();
            if( ImGui::IsItemActive() )
                _minz_editor.touch();
            if( ImGui::IsItemDeactivatedAfterEdit() )
                _minz_editor.finalize();
        };
        // Left/Right-arrow nudge for a focused-but-not-dragging slider-enum, mirroring the manual
        // numeric fields' own arrow-key handling below - one focused key press is one complete,
        // fast-committed edit.
        auto slider_enum_arrow_nudge = [ this ]( int & field, int min_v, int max_v )
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
        };

        static const char * const filter_type_names[] = { "Downscale", "Lookup Shift" };
        ImGui::Text( "Filter Type:" );
        {
            int v = _minz_editor.value.filter_type;
            bool changed = slider_enum( "##minz_filter_type", &v, 0, 1, filter_type_names );
            if( changed )
                _minz_editor.value.filter_type = v;
            slider_enum_commit( changed );
            slider_enum_arrow_nudge( _minz_editor.value.filter_type, 0, 1 );
            any_field_active = any_field_active || ImGui::IsItemActive();
        }

        if( _minz_editor.value.filter_type == 0 )
        {
            // Downscale: only the ratio matters; value 0 is reserved (not a legal wire value),
            // so the slider's range starts at 1.
            static const char * const downscale_ratio_names[] = { "x2", "x4" };
            ImGui::Text( "Downscale Ratio:" );
            int v = _minz_editor.value.downscale_ratio;
            bool changed = slider_enum( "##minz_downscale_ratio", &v, 1, 2, downscale_ratio_names );
            if( changed )
                _minz_editor.value.downscale_ratio = v;
            slider_enum_commit( changed );
            slider_enum_arrow_nudge( _minz_editor.value.downscale_ratio, 1, 2 );
            any_field_active = any_field_active || ImGui::IsItemActive();
        }
        else
        {
            // Lookup Shift: pick a fixed preset or drop into manual pixel entry.
            static const char * const shift_mode_names[] = { "Shift 126px", "Shift 64px", "Manual" };
            ImGui::Text( "Shift Mode:" );
            int v = _minz_editor.value.shift_mode;
            bool changed = slider_enum( "##minz_shift_mode", &v, 0, 2, shift_mode_names );
            if( changed )
                _minz_editor.value.shift_mode = v;
            slider_enum_commit( changed );
            slider_enum_arrow_nudge( _minz_editor.value.shift_mode, 0, 2 );
            any_field_active = any_field_active || ImGui::IsItemActive();

            if( _minz_editor.value.shift_mode == 2 )
            {
                // Each manual field below has two editing modes, toggled by the small pencil
                // button beside its label - the same edit_mode/edit_value pattern option_model
                // uses for ordinary scalar options (common/option-model.cpp), not ImGui's native
                // SliderInt Ctrl+Click/double-click text-input (that turned out not to be
                // reliably usable here):
                //   - slider mode (default): drag with the mouse, or press Left/Right once the
                //     slider has focus (single click, no drag needed) to nudge by 1 - ImGui
                //     sliders don't do arrow-key nudging on their own, so it's implemented by
                //     hand below. Each arrow press is one complete, discrete edit
                //     (touch()+finalize() together), like a radio/checkbox click, rather than a
                //     drag that only finalizes on release.
                //   - type mode: an InputText box seeded with the current value; Enter parses,
                //     clamps, and commits it, then flips back to slider mode.
                // In both modes, touch() is called unconditionally every frame the active widget
                // (slider drag OR the InputText box) has focus, not just on frames where the
                // value changes, so the debounce deadline stays parked at +infinity for the whole
                // time the user is engaged - otherwise a countdown already running from an
                // earlier edit could reach zero and fire a commit mid-edit.
                ImGui::Text( "Shift Pixels:" );
                ImGui::SameLine();
                {
                    std::string edit_id = rsutils::string::from() << textual_icons::edit << "##minz_shift_edit";
                    if( _minz_shift_edit_mode )
                        ImGui::PushStyleColor( ImGuiCol_Text, minz_manual_edit_color );
                    if( ImGui::SmallButton( edit_id.c_str() ) )
                    {
                        if( ! _minz_shift_edit_mode )
                            _minz_shift_edit_buf = std::to_string( _minz_editor.value.shift_pixels );
                        _minz_shift_edit_mode = ! _minz_shift_edit_mode;
                    }
                    if( _minz_shift_edit_mode )
                        ImGui::PopStyleColor();
                    if( ImGui::IsItemHovered() )
                        ImGui::SetTooltip( _minz_shift_edit_mode ? "Back to slider" : "Type an exact value" );
                }

                if( _minz_shift_edit_mode )
                {
                    char buf[32] = {};
                    strncpy( buf, _minz_shift_edit_buf.c_str(), sizeof( buf ) - 1 );

                    // Narrow, centered input box (not full-width/left-aligned like the slider it
                    // replaces) plus the same highlight color as the pencil icon above, so the
                    // box visually reads as "a small typed value," distinct from the wide slider
                    // it stands in for while active.
                    float avail_width = ImGui::GetContentRegionAvail().x;
                    float input_width = ImGui::CalcTextSize( "000000" ).x + ImGui::GetStyle().FramePadding.x * 2.0f;
                    ImGui::SetCursorPosX( ImGui::GetCursorPosX() + std::max( 0.0f, ( avail_width - input_width ) * 0.5f ) );
                    ImGui::PushItemWidth( input_width );
                    ImGui::PushStyleColor( ImGuiCol_Text, minz_manual_edit_color );
                    bool submitted = ImGui::InputText( "##minz_shift_input", buf, sizeof( buf ),
                                                        ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_EnterReturnsTrue );
                    ImGui::PopStyleColor();
                    ImGui::PopItemWidth();

                    if( submitted )
                    {
                        char * end = nullptr;
                        long parsed = std::strtol( buf, &end, 10 );
                        if( end != buf )
                        {
                            _minz_editor.value.shift_pixels = (int)std::min( std::max( parsed, 0L ), 256L );
                            _minz_editor.touch();
                            _minz_editor.finalize();
                        }
                        _minz_shift_edit_mode = false;
                    }
                    else
                        _minz_shift_edit_buf = buf;
                    if( ImGui::IsItemActive() )
                        _minz_editor.touch();
                    any_field_active = any_field_active || ImGui::IsItemActive();
                }
                else
                {
                    int shift = _minz_editor.value.shift_pixels;
                    if( ImGui::SliderInt( "##minz_shift", &shift, 0, 256 ) )
                    {
                        _minz_editor.value.shift_pixels = shift;
                        _minz_editor.touch();
                    }
                    if( ImGui::IsItemActive() )
                        _minz_editor.touch();
                    if( ImGui::IsItemDeactivatedAfterEdit() )
                        _minz_editor.finalize();
                    else if( ImGui::IsItemFocused() && ! ImGui::IsItemActive() )
                    {
                        if( ImGui::IsKeyPressed( ImGuiKey_RightArrow ) )
                        {
                            _minz_editor.value.shift_pixels = std::min( _minz_editor.value.shift_pixels + 1, 256 );
                            _minz_editor.touch();
                            _minz_editor.finalize( true );   // arrow-key nudge - fast turnaround
                        }
                        else if( ImGui::IsKeyPressed( ImGuiKey_LeftArrow ) )
                        {
                            _minz_editor.value.shift_pixels = std::max( _minz_editor.value.shift_pixels - 1, 0 );
                            _minz_editor.touch();
                            _minz_editor.finalize( true );   // arrow-key nudge - fast turnaround
                        }
                    }
                    any_field_active = any_field_active || ImGui::IsItemActive();
                }
            }
        }

        // Threshold mode: Zero range / MinZ (firmware-computed) / Manual. Per the design review,
        // the FW-computed MinZ value itself is NOT surfaced to the user at this stage - selecting
        // "MinZ (computed)" shows no readback field at all, unlike "Manual" which reveals one.
        static const char * const threshold_mode_names[] = { "Zero range", "MinZ (computed)", "Manual" };
        ImGui::Text( "Threshold Mode:" );
        {
            int v = _minz_editor.value.threshold_mode;
            bool changed = slider_enum( "##minz_threshold_mode", &v, 0, 2, threshold_mode_names );
            if( changed )
                _minz_editor.value.threshold_mode = v;
            slider_enum_commit( changed );
            slider_enum_arrow_nudge( _minz_editor.value.threshold_mode, 0, 2 );
            any_field_active = any_field_active || ImGui::IsItemActive();
            if( ImGui::IsItemHovered() )
                ImGui::SetTooltip( "Zero range: fill only originally-empty depth pixels.\n"
                                    "MinZ (computed): firmware picks the threshold for the active resolution.\n"
                                    "Manual: use the threshold value below." );
        }

        if( _minz_editor.value.threshold_mode == 2 )
        {
            ImGui::Text( "Threshold (mm):" );
            ImGui::SameLine();
            {
                std::string edit_id = rsutils::string::from() << textual_icons::edit << "##minz_threshold_edit";
                if( _minz_threshold_edit_mode )
                    ImGui::PushStyleColor( ImGuiCol_Text, minz_manual_edit_color );
                if( ImGui::SmallButton( edit_id.c_str() ) )
                {
                    if( ! _minz_threshold_edit_mode )
                        _minz_threshold_edit_buf = std::to_string( _minz_editor.value.threshold_mm );
                    _minz_threshold_edit_mode = ! _minz_threshold_edit_mode;
                }
                if( _minz_threshold_edit_mode )
                    ImGui::PopStyleColor();
                if( ImGui::IsItemHovered() )
                    ImGui::SetTooltip( _minz_threshold_edit_mode ? "Back to slider" : "Type an exact value" );
            }

            if( _minz_threshold_edit_mode )
            {
                char buf[32] = {};
                strncpy( buf, _minz_threshold_edit_buf.c_str(), sizeof( buf ) - 1 );

                float avail_width = ImGui::GetContentRegionAvail().x;
                float input_width = ImGui::CalcTextSize( "000000" ).x + ImGui::GetStyle().FramePadding.x * 2.0f;
                ImGui::SetCursorPosX( ImGui::GetCursorPosX() + std::max( 0.0f, ( avail_width - input_width ) * 0.5f ) );
                ImGui::PushItemWidth( input_width );
                ImGui::PushStyleColor( ImGuiCol_Text, minz_manual_edit_color );
                bool submitted = ImGui::InputText( "##minz_threshold_input", buf, sizeof( buf ),
                                                    ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_EnterReturnsTrue );
                ImGui::PopStyleColor();
                ImGui::PopItemWidth();

                if( submitted )
                {
                    char * end = nullptr;
                    long parsed = std::strtol( buf, &end, 10 );
                    if( end != buf )
                    {
                        _minz_editor.value.threshold_mm = (int)std::min( std::max( parsed, 0L ), 65535L );
                        _minz_editor.touch();
                        _minz_editor.finalize();
                    }
                    _minz_threshold_edit_mode = false;
                }
                else
                    _minz_threshold_edit_buf = buf;
                if( ImGui::IsItemActive() )
                    _minz_editor.touch();
                any_field_active = any_field_active || ImGui::IsItemActive();
            }
            else
            {
                int threshold = _minz_editor.value.threshold_mm;
                if( ImGui::SliderInt( "##minz_threshold", &threshold, 0, 65535 ) )
                {
                    _minz_editor.value.threshold_mm = threshold;
                    _minz_editor.touch();
                }
                if( ImGui::IsItemActive() )
                    _minz_editor.touch();
                if( ImGui::IsItemDeactivatedAfterEdit() )
                    _minz_editor.finalize();
                else if( ImGui::IsItemFocused() && ! ImGui::IsItemActive() )
                {
                    if( ImGui::IsKeyPressed( ImGuiKey_RightArrow ) )
                    {
                        _minz_editor.value.threshold_mm = std::min( _minz_editor.value.threshold_mm + 1, 65535 );
                        _minz_editor.touch();
                        _minz_editor.finalize( true );   // arrow-key nudge - fast turnaround
                    }
                    else if( ImGui::IsKeyPressed( ImGuiKey_LeftArrow ) )
                    {
                        _minz_editor.value.threshold_mm = std::max( _minz_editor.value.threshold_mm - 1, 0 );
                        _minz_editor.touch();
                        _minz_editor.finalize( true );   // arrow-key nudge - fast turnaround
                    }
                }
                any_field_active = any_field_active || ImGui::IsItemActive();
            }
        }

        ImGui::Dummy( ImVec2( 0, 2 ) );
        float frame_bottom = ImGui::GetCursorScreenPos().y;
        ImVec2 frame_min( frame_left, frame_top );
        ImVec2 frame_max( frame_left + frame_width, frame_bottom );

        // Reset to Default starts hidden behind a small "..." marker tucked into the box's
        // bottom-right corner - this is a destructive-ish, rarely-used action that doesn't need
        // to compete for attention with the fields above it every time the box is open. Hovering
        // anywhere within reveal_margin of the corner (not just exactly on the tiny marker, which
        // would be fiddly to hit) swaps it for the real button in the same spot; moving away
        // collapses it back to "...". Drawn as an absolute-position overlay via
        // SetCursorScreenPos rather than inline in the normal top-to-bottom flow, since its
        // presence/absence shouldn't shift any of the fields above it - the cursor is restored
        // afterward so the NEXT thing this panel draws isn't displaced. No line of its own is
        // reserved for it (frame_bottom sits right after the last field's normal 2px pad), so
        // the revealed button sits on top of - and may partially overlap - that last field's
        // own row rather than pushing the box taller.
        {
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
                // Goes through the exact same touch()/finalize() pipeline as every other field
                // edit above (one click = one discrete, debounced, atomically-committed change),
                // rather than a separate commit path, so it gets the fade animation and the undo
                // grace window for free. range.def is the FULL FW-reported default struct (header
                // fields included); before_commit below forces enable back on regardless, same as
                // any other edit in this box - "reset" doesn't leave the control disabled even if
                // that's literally what the FW default says, for consistency with "touching this
                // box means you want it on."
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
                any_field_active = any_field_active || ImGui::IsItemActive();
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
        }

        // Draws the dirty-state fade/border, the hover tooltip, and sends the whole struct once
        // the debounce timer lapses (or focus leaves the group early) - see
        // composite_control_editor<T> for the full mechanism, which is shared by any other
        // multi-param composite option that wants the same behavior.
        //
        // before_commit: editing any field in this box implies the control is meant to be active
        // - without the Enable checkbox in here anymore (it's the row header's toggle now), a
        // user who only ever touches e.g. Threshold would otherwise send with enable still at
        // whatever it was (often off), silently updating parameters for a control that stays
        // disabled. Force it on, and reflect that back into the row toggle's own state.
        _minz_editor.end_frame_and_maybe_commit( _embedded_filter, id, error_message, frame_min, frame_max, any_field_active,
            [this]( rs2_minz_control & v )
            {
                v.enable = 1;
                _enabled = true;
            } );

        if( dim_while_disabled )
            ImGui::PopStyleVar();

        ImGui::Unindent( 4.0f );
    }

    void embedded_filter_model::embedded_filter_enable_disable(bool actual)
    {
        // Composite-only embedded filters (e.g. HKR MinZ Control) register no
        // RS2_OPTION_EMBEDDED_FILTER_ENABLED scalar option at all - route the row header's
        // toggle through the composite option's own `enable` field instead: read-modify-write
        // so the other 4 fields go back exactly as the device last reported them, not
        // zero-initialized.
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
