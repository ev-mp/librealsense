// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_hkr_hdrd_control.h>
#include <librealsense2/h/rs_hkr_temporal_filter_dpp.h>
#include "composite-control-editor.h"
#include <functional>
#include <string>
#include <vector>


namespace rs2
{
    class subdevice_model;
    class option_model;
    class viewer_model;

    class embedded_filter_model
    {
    public:
        embedded_filter_model( subdevice_model* owner,
            const rs2_embedded_filter_type& type,
            std::shared_ptr<rs2::embedded_filter> filter,
            viewer_model& viewer,
            std::string& error_message);

        virtual ~embedded_filter_model();

        const std::string& get_name() const { return _name; }

        void populate_options( const std::string& opt_base_label,
            subdevice_model* model,
            bool* options_invalidated,
            std::string& error_message );

        void draw_options( viewer_model & viewer,
                           bool update_read_only_options,
                           bool is_streaming,
                           std::string & error_message );

        // Hardcoded editor for RS2_COMPOSITE_OPTION_HKR_HDRD_CONTROL
        // specifically, called from draw_options() when that id is among _composite_option_ids.
        // There is no generic per-field composite-option editor (would need per-struct schema
        // knowledge generic view code doesn't have) - this special-cases the one struct by id,
        // same way application code is expected to (see rs_hkr_hdrd_control.h).
        void draw_hdrd_control_editor( std::string & error_message );

        // Same scheme as draw_hdrd_control_editor() above, for
        // RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP - a second, independent hardcoded editor,
        // not a generalization of the first: different struct, different fields, own editor
        // instance. Simpler than HDRD's - every field here is a plain range-bounded number, none
        // of them select which other fields are relevant, so there is no BeginDisabled()-greyed
        // conditional relevance to track.
        void draw_temporal_filter_dpp_control_editor( std::string & error_message );

        std::shared_ptr<rs2::embedded_filter> get_filter() { return _embedded_filter; }

        // error_message, if non-null, receives the reason when the device rejects the change
        // (e.g. some composite controls cannot be toggled while the sensor is streaming) - without
        // it the toggle would otherwise appear to silently do nothing.
        void enable( bool e = true, std::string * error_message = nullptr )
        {
            embedded_filter_enable_disable( e, error_message );
        }
        bool is_enabled() const { return _enabled; }

        bool _is_visible = true;

        // Optional predicate; null means always available.
        // When it returns false the enable toggle is grayed out in the UI.
        // Set by the owner after construction for filters with runtime constraints
        // (e.g. close range, which works on depth only and must be off while color streams).
        std::function<bool()> available_predicate;
        // Optional message shown when the toggle is unavailable; empty = none.
        std::string unavailable_tooltip;

        bool is_available() const { return !available_predicate || available_predicate(); }

        // Mirrors _hdrd_editor's/_temporal_filter_dpp_editor's own pending-auto-commit state (see
        // composite_control_editor<T>::try_get_progress()) so the row header's enable toggle,
        // drawn elsewhere (device-model.cpp's draw_embedded_filters()), can visually echo the
        // same "about to send" fade the framed editor box shows. A given embedded filter only
        // ever has one of the two editors actually initialized (each backs a different composite
        // option, and an embedded filter exposes at most one of these two today) - checking both
        // and returning whichever is dirty is simpler than tracking which one applies.
        bool has_pending_composite_commit( float & progress ) const
        {
            if( _hdrd_editor.try_get_progress( progress ) )
                return true;
            return _temporal_filter_dpp_editor.try_get_progress( progress );
        }

        void embedded_filter_enable_disable(bool actual, std::string * error_message = nullptr);

    private:
        // Helpers used only by draw_hdrd_control_editor() - each renders one self-contained
        // field/element of the Improved Close Range editor and returns whether the user is actively interacting
        // with it this frame (folded into draw_hdrd_control_editor()'s own any_field_active
        // tracking). Split out so no single function mixes more than one field's concerns; none
        // of these are meant to be called from anywhere else. frame_max is passed as separate
        // x/y floats rather than ImVec2 so this header does not need to pull in imgui.h.
        // Shared body for the four enum fields below - an ImGui::SliderInt whose displayed text is
        // the enum's own name rather than the raw integer, the same "slider enum" idiom shown in
        // Dear ImGui's own widget demo (Sliders section) - not a dropdown/combo, so it costs no
        // extra click to open before a value can be picked, and it supports the same drag/arrow-key
        // interaction as every numeric slider in this panel (see
        // draw_hdrd_slider_with_arrows()). value_offset converts between the
        // field's own wire values and the 0-based index the slider works in (only downscale_ratio's
        // wire values, 1 and 2, aren't already 0-based).
        bool draw_hdrd_enum_field( const char * label,
                                                    const char * id,
                                                    const char * const labels[],
                                                    int count,
                                                    int & value,
                                                    int value_offset );
        bool draw_hdrd_filter_type_field();
        bool draw_hdrd_downscale_ratio_field();
        bool draw_hdrd_shift_mode_field();
        bool draw_hdrd_threshold_mode_field();
        // Draws all 6 fields, in order, each wrapped in BeginDisabled()/EndDisabled() when the
        // current filter_type/shift_mode selection makes it irrelevant - the "what to draw"
        // concern, kept separate from draw_hdrd_control_editor()'s own framing/
        // dim/commit orchestration. Returns whether the user is actively interacting with any of
        // them (folded into that caller's own any_field_active tracking).
        bool draw_hdrd_fields();
        // Manual-entry escape hatch shared by Shift Pixels and Threshold (mm).
        bool draw_hdrd_manual_editable_field( const char * label,
                                               const char * id,
                                               int & value,
                                               int min_v,
                                               int max_v,
                                               bool & edit_mode,
                                               std::string & edit_buf );
        bool draw_hdrd_manual_input( const char * id, int & value, int min_v, int max_v,
                                      bool & edit_mode, std::string & edit_buf );
        bool draw_hdrd_slider_with_arrows( const char * id, int & value, int min_v, int max_v );
        bool draw_hdrd_reset_to_default_overlay( rs2_composite_option_id id,
                                                  std::string & error_message,
                                                  float frame_max_x,
                                                  float frame_max_y );

        // Helpers used only by draw_temporal_filter_dpp_control_editor() - same scheme as the
        // HDRD helpers above (manual-entry escape hatch, reset-to-default overlay), applied to
        // this struct's own fields. No enum-field/BeginDisabled machinery here - every field is
        // relevant all the time, so there is nothing equivalent to draw_hdrd_enum_field()/
        // draw_hdrd_fields()'s conditional-relevance tracking to duplicate.
        bool draw_temporal_filter_dpp_manual_editable_field( const char * label,
                                                              const char * id,
                                                              int & value,
                                                              int min_v,
                                                              int max_v,
                                                              bool & edit_mode,
                                                              std::string & edit_buf );
        bool draw_temporal_filter_dpp_manual_input( const char * id, int & value, int min_v, int max_v,
                                                     bool & edit_mode, std::string & edit_buf );
        bool draw_temporal_filter_dpp_slider_with_arrows( const char * id, int & value, int min_v, int max_v );
        // Draws all 3 fields in a row - the "what to draw" concern, kept separate from
        // draw_temporal_filter_dpp_control_editor()'s own framing/dim/commit orchestration, same
        // split as draw_hdrd_fields() above.
        bool draw_temporal_filter_dpp_fields();
        bool draw_temporal_filter_dpp_reset_to_default_overlay( rs2_composite_option_id id,
                                                                 std::string & error_message,
                                                                 float frame_max_x,
                                                                 float frame_max_y );

        // The three independent concerns populate_options() used to inline directly - split out
        // so each is readable on its own: scalar rs2_option models, composite-option editor
        // priming (Improved Close Range/Temporal Filter DPP's initial GET and _enabled fallback),
        // and the on_options_changed() callback that keeps _enabled and the scalar models synced
        // to later external changes.
        void populate_scalar_options( const std::string & opt_base_label,
                                       subdevice_model * model,
                                       std::string & error_message );
        void populate_composite_options( std::string & error_message );
        void register_options_changed_callback();

    protected:
        viewer_model& _viewer;
        std::atomic<bool> _destructing;
        bool _enabled = true;
        std::shared_ptr<rs2::embedded_filter> _embedded_filter;
        std::map< rs2_option, option_model > _options_id_to_model;
        // Composite options are a completely separate identity/registry space from scalar
        // rs2_option (see rs_composite_option.h) - enumerated and drawn through their own loop
        // in populate_options()/draw_options() rather than folded into _options_id_to_model.
        // There is no generic per-field editing UI for these yet (that would need per-struct
        // schema knowledge no generic view code has) - draw_options() shows read-only metadata
        // only (description, byte size).
        std::vector< rs2_composite_option_id > _composite_option_ids;
        std::string _name;

        // Debounced auto-commit editor for the Improved Close Range control, built on the
        // reusable composite_control_editor<T> mechanism (see composite-control-editor.h) - any
        // other multi-param composite option can get the same touch/fade/reset/commit behavior
        // by holding its own composite_control_editor<its-struct-type> the same way.
        composite_control_editor< rs2_hdrd_control > _hdrd_editor;

        // Per-field manual-entry toggle state for the Shift Pixels / Threshold (mm) sliders (see
        // draw_hdrd_control_editor()) - mirrors option_model's own edit_mode/edit_value pattern
        // (common/option-model.cpp) rather than relying on ImGui's native SliderInt Ctrl+Click/
        // double-click text-input, which turned out not to be reliably discoverable/usable here.
        bool _hdrd_shift_edit_mode = false;
        std::string _hdrd_shift_edit_buf;
        bool _hdrd_threshold_edit_mode = false;
        std::string _hdrd_threshold_edit_buf;

        // Same scheme as _hdrd_editor above, for RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP.
        composite_control_editor< rs2_temporal_filter_dpp_config > _temporal_filter_dpp_editor;

        // Per-field manual-entry toggle state, same scheme as the HDRD ones above - one pair per
        // field here since every field in this struct uses the manual-editable-field pattern
        // (HDRD's Shift Pixels/Threshold (mm) are the exception there, not the rule; here it's
        // every field, since none of them are enum-valued).
        bool _temporal_filter_dpp_smooth_alpha_edit_mode = false;
        std::string _temporal_filter_dpp_smooth_alpha_edit_buf;
        bool _temporal_filter_dpp_smooth_delta_edit_mode = false;
        std::string _temporal_filter_dpp_smooth_delta_edit_buf;
        bool _temporal_filter_dpp_persistency_index_edit_mode = false;
        std::string _temporal_filter_dpp_persistency_index_edit_buf;
    };
}
