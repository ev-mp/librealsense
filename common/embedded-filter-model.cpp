// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <algorithm>
#include <iostream>
#include <string>
#include "subdevice-model.h"
#include "embedded-filter-model.h"
#include "viewer.h"


namespace rs2
{
    namespace
    {
        // DEBUG: every field actually read back from FW for RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL -
        // called right after each real GET (initial seed and the toggle's read-modify-write), never
        // on every-frame no-op re-reads, so this reflects only genuine device round-trips.
        void print_minz_control( const rs2_minz_control & v )
        {
            std::cout << "[MinZ GET] version=" << (int)v.version
                       << " flags=" << (int)v.flags
                       << " ctl_id=0x" << std::hex << v.ctl_id << std::dec
                       << " param_count=" << (int)v.param_count
                       << " param_type=" << (int)v.param_type
                       << " enable=" << v.enable
                       << " downscale_ratio=" << v.downscale_ratio
                       << " disparity_shift=" << v.disparity_shift
                       << " threshold=" << v.threshold
                       << " threshold_mode=" << v.threshold_mode
                       << " reserved=[" << v.reserved[0] << "," << v.reserved[1] << "," << v.reserved[2] << "]"
                       << std::endl;
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
            ImGui::PushStyleVar( ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.6f );

        // Tracked across every widget below so end_frame_and_maybe_commit() can tell "nothing of
        // ours is active right now" (the normal, harmless gap between finishing one field and
        // touching the next) apart from "focus genuinely left the group" (some OTHER widget
        // elsewhere is active) - only the latter should cut the countdown short.
        bool any_field_active = false;

        // Every label below sits on its own line, with the interactive widget using a "##"-only
        // (invisible) label - this side panel is too narrow for "label: [====slider====] value"
        // all on one line; ImGui doesn't wrap inline widget labels, it just clips them.
        // Downscale ratio only ever takes {1,2,4} - radio buttons, not a slider, so the user
        // can't land on an invalid value like 3 (mirrors the field validation tool's own UI).
        ImGui::Text( "Downscale Ratio:" );
        for( int v : { 1, 2, 4 } )
        {
            ImGui::SameLine();
            if( ImGui::RadioButton( ( std::to_string( v ) + "##minz_ratio" ).c_str(), _minz_editor.value.downscale_ratio == v ) )
            {
                _minz_editor.value.downscale_ratio = v;
                _minz_editor.touch();
                _minz_editor.finalize();   // one click = started and finished at once
            }
            any_field_active = any_field_active || ImGui::IsItemActive();
        }

        ImGui::Text( "Disparity Shift:" );
        int shift = _minz_editor.value.disparity_shift;
        if( ImGui::SliderInt( "##minz_shift", &shift, 0, 512 ) )
        {
            _minz_editor.value.disparity_shift = shift;
            _minz_editor.touch();
        }
        if( ImGui::IsItemDeactivatedAfterEdit() )
            _minz_editor.finalize();
        any_field_active = any_field_active || ImGui::IsItemActive();

        ImGui::Text( "Threshold (mm):" );
        int threshold = _minz_editor.value.threshold;
        if( ImGui::SliderInt( "##minz_threshold", &threshold, 0, 65535 ) )
        {
            _minz_editor.value.threshold = threshold;
            _minz_editor.touch();
        }
        if( ImGui::IsItemDeactivatedAfterEdit() )
            _minz_editor.finalize();
        any_field_active = any_field_active || ImGui::IsItemActive();

        bool manual = _minz_editor.value.threshold_mode != 0;
        if( ImGui::Checkbox( "Manual Threshold##minz", &manual ) )
        {
            _minz_editor.value.threshold_mode = manual ? 1 : 0;
            _minz_editor.touch();
            _minz_editor.finalize();
        }
        any_field_active = any_field_active || ImGui::IsItemActive();
        if( ImGui::IsItemHovered() )
            ImGui::SetTooltip( "Unchecked = Auto (firmware-computed threshold)" );

        ImGui::Dummy( ImVec2( 0, 2 ) );
        float frame_bottom = ImGui::GetCursorScreenPos().y;
        ImVec2 frame_min( frame_left, frame_top );
        ImVec2 frame_max( frame_left + frame_width, frame_bottom );

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
