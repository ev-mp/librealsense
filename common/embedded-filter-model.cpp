// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <realsense_imgui.h>
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
        // DEBUG: every field actually read back from FW for RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL -
        // called right after each real GET (initial seed and the toggle's read-modify-write), never
        // on every-frame no-op re-reads, so this reflects only genuine device round-trips.
        // LOG_DEBUG (not std::cout) so this only emits when the app's configured log verbosity is
        // DEBUG or more verbose, not at the default INFO level.
        void print_improved_close_range_control( const rs2_improved_close_range_control & v )
        {
            LOG_DEBUG( "[Improved Close Range GET] version=" << (int)v.header.version
                       << " flags=" << (int)v.header.flags
                       << " ctl_id=0x" << std::hex << v.header.ctl_id << std::dec
                       << " param_count=" << (int)v.header.param_count
                       << " param_type=" << (int)v.header.param_type
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
        const ImVec4 improved_close_range_manual_edit_color( 1.0f, 0.65f, 0.0f, 1.0f );

        // A real device's firmware may still speak the pre-design-review wire layout, so a field
        // can land outside its enum's legal range. Called after every raw device read that
        // populates a fresh rs2_improved_close_range_control, before any branching or display code
        // uses it - clamping only at display time would leave the raw stored value inconsistent
        // with what's shown (e.g. a garbage shift_mode of 200 displays as "Manual" but `== 2`
        // never matches), and would leave the conditional branching in
        // draw_improved_close_range_control_editor() looking at an undefined case.
        // RsImGui::CustomComboBox (used below) is separately bounds-checked against its own
        // items_count, so this isn't needed to prevent a crash - only to keep the displayed value
        // and the branching consistent with each other.
        void sanitize_improved_close_range_control( rs2_improved_close_range_control & v )
        {
            v.filter_type = std::min( std::max( v.filter_type, 0 ), 1 );
            v.downscale_ratio = std::min( std::max( v.downscale_ratio, 1 ), 2 );
            v.shift_mode = std::min( std::max( v.shift_mode, 0 ), 2 );
            v.threshold_mode = std::min( std::max( v.threshold_mode, 0 ), 2 );
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
        // RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL gets a hardcoded editor below; everything else
        // just shows read-only metadata (description, byte size).
        for( auto id : _composite_option_ids )
        {
            try
            {
                if( id == RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL )
                {
                    // No always-visible description line here - draw_improved_close_range_control_editor shows
                    // it as a tooltip on hovering the framed control instead, to save vertical
                    // space in this narrow side panel.
                    draw_improved_close_range_control_editor( error_message );
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

    // Real combo-box widget (RsImGui::CustomComboBox, see third-party/imgui/realsense_imgui.h),
    // matching the convention every other enum-valued option in the viewer already uses (see
    // option_model::draw_combobox() in common/option-model.cpp) - not a SliderInt whose format
    // string happens to show a name, which was never more than a slider styled to look like a
    // choice list. touch()/finalize() drive the same debounced auto-commit every other field in
    // this editor uses (see composite-control-editor.h).
    bool embedded_filter_model::draw_improved_close_range_filter_type_field()
    {
        static const char * const labels[] = { "Downscale", "Lookup Shift" };
        ImGui::Text( "Filter Type:" );
        ImGui::SameLine();
        int selected = _improved_close_range_editor.value.filter_type;
        if( RsImGui::CustomComboBox( "##improved_close_range_filter_type", &selected, labels, 2 ) )
        {
            _improved_close_range_editor.value.filter_type = selected;
            _improved_close_range_editor.touch();
            _improved_close_range_editor.finalize();
        }
        return ImGui::IsItemActive();
    }

    bool embedded_filter_model::draw_improved_close_range_downscale_ratio_field()
    {
        // Wire values are 1 (x2) and 2 (x4), not 0-based like the other enum fields - convert to
        // and from a 0-based combo index rather than changing the documented wire values.
        static const char * const labels[] = { "x2", "x4" };
        ImGui::Text( "Downscale Ratio:" );
        ImGui::SameLine();
        int selected = _improved_close_range_editor.value.downscale_ratio - 1;
        if( RsImGui::CustomComboBox( "##improved_close_range_downscale_ratio", &selected, labels, 2 ) )
        {
            _improved_close_range_editor.value.downscale_ratio = selected + 1;
            _improved_close_range_editor.touch();
            _improved_close_range_editor.finalize();
        }
        return ImGui::IsItemActive();
    }

    bool embedded_filter_model::draw_improved_close_range_shift_mode_field()
    {
        // Lookup Shift: pick a fixed preset, or Manual - which draw_improved_close_range_control_editor()
        // reveals via a separate draw_improved_close_range_manual_editable_field() call for Shift Pixels.
        static const char * const labels[] = { "Shift 126px", "Shift 64px", "Manual" };
        ImGui::Text( "Shift Mode:" );
        ImGui::SameLine();
        int selected = _improved_close_range_editor.value.shift_mode;
        if( RsImGui::CustomComboBox( "##improved_close_range_shift_mode", &selected, labels, 3 ) )
        {
            _improved_close_range_editor.value.shift_mode = selected;
            _improved_close_range_editor.touch();
            _improved_close_range_editor.finalize();
        }
        return ImGui::IsItemActive();
    }

    // The InputText half of draw_improved_close_range_manual_editable_field()'s two editing modes: a narrow,
    // centered box seeded with the current value; Enter parses, clamps, and commits it, then
    // flips back to slider mode. touch() is called every frame this box has focus, not just on
    // change, so the debounce deadline stays parked at +infinity for the whole time the user is
    // engaged - otherwise a countdown already running from an earlier edit could reach zero and
    // fire a commit mid-edit.
    bool embedded_filter_model::draw_improved_close_range_manual_input( const char * id, int & value, int min_v, int max_v,
                                                          bool & edit_mode, std::string & edit_buf )
    {
        char buf[32] = {};
        strncpy( buf, edit_buf.c_str(), sizeof( buf ) - 1 );

        // Narrow, centered input box (not full-width/left-aligned like the slider it replaces)
        // plus the same highlight color as the pencil icon beside it, so the box visually reads
        // as "a small typed value," distinct from the wide slider it stands in for while active.
        float avail_width = ImGui::GetContentRegionAvail().x;
        float input_width = ImGui::CalcTextSize( "000000" ).x + ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SetCursorPosX( ImGui::GetCursorPosX() + std::max( 0.0f, ( avail_width - input_width ) * 0.5f ) );
        ImGui::PushItemWidth( input_width );
        ImGui::PushStyleColor( ImGuiCol_Text, improved_close_range_manual_edit_color );
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
                _improved_close_range_editor.touch();
                _improved_close_range_editor.finalize();
            }
            edit_mode = false;
        }
        else
            edit_buf = buf;

        if( ImGui::IsItemActive() )
            _improved_close_range_editor.touch();
        return ImGui::IsItemActive();
    }

    // The slider half of draw_improved_close_range_manual_editable_field()'s two editing modes. ImGui sliders
    // don't do arrow-key nudging on their own, so it's implemented by hand: one focused arrow
    // press is one complete, discrete edit (touch()+finalize() together), like a radio/checkbox
    // click, rather than a drag that only finalizes on release.
    bool embedded_filter_model::draw_improved_close_range_slider_with_arrows( const char * id, int & value, int min_v, int max_v )
    {
        std::string slider_id = rsutils::string::from() << "##" << id;
        if( ImGui::SliderInt( slider_id.c_str(), &value, min_v, max_v ) )
            _improved_close_range_editor.touch();
        if( ImGui::IsItemActive() )
            _improved_close_range_editor.touch();
        if( ImGui::IsItemDeactivatedAfterEdit() )
            _improved_close_range_editor.finalize();
        else if( ImGui::IsItemFocused() && ! ImGui::IsItemActive() )
        {
            if( ImGui::IsKeyPressed( ImGuiKey_RightArrow ) )
            {
                value = std::min( value + 1, max_v );
                _improved_close_range_editor.touch();
                _improved_close_range_editor.finalize( true );   // arrow-key nudge - fast turnaround
            }
            else if( ImGui::IsKeyPressed( ImGuiKey_LeftArrow ) )
            {
                value = std::max( value - 1, min_v );
                _improved_close_range_editor.touch();
                _improved_close_range_editor.finalize( true );   // arrow-key nudge - fast turnaround
            }
        }
        return ImGui::IsItemActive();
    }

    // One "label + pencil-toggle + (slider OR manual InputText)" field - shared by Disparity
    // Shift and Threshold (mm), which otherwise repeated this exact pattern twice. The pencil
    // button toggles between the two edit_mode/edit_buf-backed widgets drawn by
    // draw_improved_close_range_manual_input()/draw_improved_close_range_slider_with_arrows() above (the same edit_mode/edit_buf
    // pattern option_model uses for ordinary scalar options in common/option-model.cpp, not
    // ImGui's native SliderInt Ctrl+Click/double-click text-input, which turned out not to be
    // reliably usable here).
    bool embedded_filter_model::draw_improved_close_range_manual_editable_field( const char * label, const char * id, int & value,
                                                                   int min_v, int max_v, bool & edit_mode,
                                                                   std::string & edit_buf )
    {
        ImGui::Text( "%s", label );
        ImGui::SameLine();
        {
            std::string edit_id = rsutils::string::from() << textual_icons::edit << "##" << id << "_edit";
            if( edit_mode )
                ImGui::PushStyleColor( ImGuiCol_Text, improved_close_range_manual_edit_color );
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

        return edit_mode ? draw_improved_close_range_manual_input( id, value, min_v, max_v, edit_mode, edit_buf )
                          : draw_improved_close_range_slider_with_arrows( id, value, min_v, max_v );
    }

    bool embedded_filter_model::draw_improved_close_range_threshold_mode_field()
    {
        // Zero range / MinZ (firmware-computed) / Manual. Per the design review, the FW-computed
        // MinZ value itself is NOT surfaced to the user at this stage - "MinZ (computed)" reveals
        // no readback field at all, unlike "Manual" (see draw_improved_close_range_manual_editable_field()).
        static const char * const labels[] = { "Zero range", "MinZ (computed)", "Manual" };
        ImGui::Text( "Threshold Mode:" );
        ImGui::SameLine();
        int selected = _improved_close_range_editor.value.threshold_mode;
        if( RsImGui::CustomComboBox( "##improved_close_range_threshold_mode", &selected, labels, 3 ) )
        {
            _improved_close_range_editor.value.threshold_mode = selected;
            _improved_close_range_editor.touch();
            _improved_close_range_editor.finalize();
        }
        bool active = ImGui::IsItemActive();
        if( ImGui::IsItemHovered() )
            ImGui::SetTooltip( "Zero range: fill only originally-empty depth pixels.\n"
                                "MinZ (computed): firmware picks the threshold for the active resolution.\n"
                                "Manual: use the threshold value below." );
        return active;
    }

    // Reset to Default starts hidden behind a small "..." marker tucked into the box's
    // bottom-right corner - this is a destructive-ish, rarely-used action that doesn't need to
    // compete for attention with the fields above it every time the box is open. Hovering
    // anywhere within reveal_margin of the corner (not just exactly on the tiny marker, which
    // would be fiddly to hit) swaps it for the real button in the same spot; moving away
    // collapses it back to "...". Drawn as an absolute-position overlay via SetCursorScreenPos
    // rather than inline in the normal top-to-bottom flow, since its presence/absence shouldn't
    // shift any of the fields above it - the cursor is restored afterward so the NEXT thing this
    // panel draws isn't displaced.
    bool embedded_filter_model::draw_improved_close_range_reset_to_default_overlay( rs2_composite_option_id id,
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
            // Goes through the exact same touch()/finalize() pipeline as every other field edit
            // (one click = one discrete, debounced, atomically-committed change), rather than a
            // separate commit path, so it gets the fade animation and the undo grace window for
            // free. range.def is the FULL FW-reported default struct (header fields included);
            // draw_improved_close_range_control_editor()'s before_commit forces enable back on regardless, same
            // as any other edit in this box - "reset" doesn't leave the control disabled even if
            // that's literally what the FW default says, for consistency with "touching this box
            // means you want it on."
            ImGui::SetCursorScreenPos( button_pos );
            if( ImGui::Button( "Reset to Default##improved_close_range" ) )
            {
                try
                {
                    auto range = _embedded_filter->get_composite_option_range_as< rs2_improved_close_range_control_range >( id );
                    _improved_close_range_editor.value = range.def;
                    sanitize_improved_close_range_control( _improved_close_range_editor.value );
                    _improved_close_range_editor.touch();
                    _improved_close_range_editor.finalize();
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

    void embedded_filter_model::draw_improved_close_range_control_editor( std::string & error_message )
    {
        const auto id = RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL;

        if( ! _improved_close_range_editor.ensure_initialized( _embedded_filter, id, error_message, print_improved_close_range_control ) )
            return;
        sanitize_improved_close_range_control( _improved_close_range_editor.value );

        // Minimal indent - just enough padding that widget text doesn't sit flush on the frame
        // border - rather than the tree's full default indent, so the group sits as far left as
        // the panel allows. ImGui::Indent(w) ADDS w to the indent (so a negative w shifts left);
        // the matching ImGui::Unindent(w) below MUST pass the same -5.f, not a different value -
        // Unindent always SUBTRACTS its argument, so Unindent(-5.f) is what actually cancels
        // Indent(-5.f) out (subtracting a negative = adding back the 5 we removed).
        ImGui::Indent( -5.f );
        ImGui::Dummy( ImVec2( 0, 2 ) );

        float frame_left = ImGui::GetCursorScreenPos().x - 4.0f;
        float frame_top = ImGui::GetCursorScreenPos().y - 4.0f;
        float frame_width = ImGui::GetContentRegionAvail().x -30.0f;

        // No separate Enable checkbox - the row header's own toggle (device-model.cpp's
        // draw_embedded_filters()) drives rs2_improved_close_range_control::enable directly. Grey the whole box
        // out while it reads off (global alpha, not BeginDisabled, so fields stay clickable -
        // editing any of them while off forces enable back on at commit time below).
        const bool dim_while_disabled = ! _enabled;
        if( dim_while_disabled )
            ImGui::PushStyleVar( ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.35f );

        // any_field_active lets end_frame_and_maybe_commit() tell "nothing of ours is active
        // right now" apart from "focus genuinely left the group" - only the latter should cut the
        // debounce countdown short. |= (not ||=): each call below draws real widgets as a side
        // effect and must run every frame regardless of the flag accumulated so far.
        bool any_field_active = false;
        any_field_active |= draw_improved_close_range_filter_type_field();

        if( _improved_close_range_editor.value.filter_type == 0 )
        {
            any_field_active |= draw_improved_close_range_downscale_ratio_field();
        }
        else
        {
            any_field_active |= draw_improved_close_range_shift_mode_field();
            if( _improved_close_range_editor.value.shift_mode == 2 )
                any_field_active |= draw_improved_close_range_manual_editable_field( "Shift Pixels:", "improved_close_range_shift",
                    _improved_close_range_editor.value.shift_pixels, 0, 256, _improved_close_range_shift_edit_mode, _improved_close_range_shift_edit_buf );
        }

        any_field_active |= draw_improved_close_range_threshold_mode_field();
        if( _improved_close_range_editor.value.threshold_mode == 2 )
            any_field_active |= draw_improved_close_range_manual_editable_field( "Threshold (mm):", "improved_close_range_threshold",
                _improved_close_range_editor.value.threshold_mm, 0, 65535, _improved_close_range_threshold_edit_mode, _improved_close_range_threshold_edit_buf );

        ImGui::Dummy( ImVec2( 0, 2 ) );
        float frame_bottom = ImGui::GetCursorScreenPos().y;
        ImVec2 frame_min( frame_left, frame_top );
        ImVec2 frame_max( frame_left + frame_width, frame_bottom );

        // No line of its own is reserved for the reset overlay (frame_bottom sits right after the
        // last field's 2px pad), so it may partially overlap that field's row rather than pushing
        // the box taller.
        any_field_active |= draw_improved_close_range_reset_to_default_overlay( id, error_message, frame_max.x, frame_max.y );

        // Draws the dirty-state fade/border and sends the whole struct once the debounce timer
        // lapses (see composite_control_editor<T>). before_commit forces enable back on: without
        // an Enable checkbox in here anymore, a user who only touches e.g. Threshold would
        // otherwise send with enable unchanged (often off) - reflect that back into the row
        // toggle's own state too.
        _improved_close_range_editor.end_frame_and_maybe_commit( _embedded_filter, id, error_message, frame_min, frame_max, any_field_active,
            [this]( rs2_improved_close_range_control & v )
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
        // Composite-only embedded filters (e.g. HKR Improved Close Range Control) register no
        // RS2_OPTION_EMBEDDED_FILTER_ENABLED scalar option at all - route the row header's
        // toggle through the composite option's own `enable` field instead: read-modify-write
        // so the other 4 fields go back exactly as the device last reported them, not
        // zero-initialized.
        if( _embedded_filter->supports_composite_option( RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL ) )
        {
            try
            {
                _improved_close_range_editor.value = _embedded_filter->get_composite_option_as< rs2_improved_close_range_control >(
                    RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL );
                sanitize_improved_close_range_control( _improved_close_range_editor.value );
                print_improved_close_range_control( _improved_close_range_editor.value );
                _improved_close_range_editor.value.enable = actual ? 1 : 0;
                _embedded_filter->set_composite_option_from( RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL, _improved_close_range_editor.value );
                _improved_close_range_editor.initialized = true;
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
            // No generic per-composite-option editor exists yet (see draw_options()) - Improved Close Range is
            // the one hardcoded case, and the only one with an `enable` field to prime here.
            if( id != RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL )
                continue;
            if( ! _embedded_filter->supports_composite_option( id ) )
                continue;

            if( _improved_close_range_editor.ensure_initialized( _embedded_filter, id, error_message, print_improved_close_range_control ) )
            {
                sanitize_improved_close_range_control( _improved_close_range_editor.value );
                // Composite state is the FALLBACK source of _enabled, not an override - a DDS
                // filter that already set it from the scalar option in pass 1 keeps that value.
                if( ! _embedded_filter->supports( RS2_OPTION_EMBEDDED_FILTER_ENABLED ) )
                    _enabled = _improved_close_range_editor.value.enable != 0;
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
