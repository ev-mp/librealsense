// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_hkr_improved_close_range_control.h>
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

        // Hardcoded editor for RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL
        // specifically, called from draw_options() when that id is among _composite_option_ids.
        // There is no generic per-field composite-option editor (would need per-struct schema
        // knowledge generic view code doesn't have) - this special-cases the one struct by id,
        // same way application code is expected to (see rs_hkr_improved_close_range_control.h).
        void draw_improved_close_range_control_editor( std::string & error_message );

        std::shared_ptr<rs2::embedded_filter> get_filter() { return _embedded_filter; }

        void enable( bool e = true )
        {
            embedded_filter_enable_disable( e );
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

        // Mirrors _improved_close_range_editor's own pending-auto-commit state (see
        // composite_control_editor<T>::try_get_progress()) so the row header's enable toggle,
        // drawn elsewhere (device-model.cpp's draw_embedded_filters()), can visually echo the
        // same "about to send" fade the framed Improved Close Range editor box shows. Hardcoded to Improved Close Range same as
        // draw_improved_close_range_control_editor() - false for every other embedded filter.
        bool has_pending_composite_commit( float & progress ) const
        {
            return _improved_close_range_editor.try_get_progress( progress );
        }

        void embedded_filter_enable_disable(bool actual);

    private:
        // Helpers used only by draw_improved_close_range_control_editor() - each renders one self-contained
        // field/element of the Improved Close Range editor and returns whether the user is actively interacting
        // with it this frame (folded into draw_improved_close_range_control_editor()'s own any_field_active
        // tracking). Split out so no single function mixes more than one field's concerns; none
        // of these are meant to be called from anywhere else. frame_max is passed as separate
        // x/y floats rather than ImVec2 so this header does not need to pull in imgui.h.
        bool draw_improved_close_range_downscale_ratio_field();
        bool draw_improved_close_range_manual_editable_field( const char * label,
                                               const char * id,
                                               int & value,
                                               int min_v,
                                               int max_v,
                                               bool & edit_mode,
                                               std::string & edit_buf );
        bool draw_improved_close_range_manual_input( const char * id, int & value, int min_v, int max_v,
                                      bool & edit_mode, std::string & edit_buf );
        bool draw_improved_close_range_slider_with_arrows( const char * id, int & value, int min_v, int max_v );
        bool draw_improved_close_range_threshold_mode_field();
        bool draw_improved_close_range_reset_to_default_overlay( rs2_composite_option_id id,
                                                  std::string & error_message,
                                                  float frame_max_x,
                                                  float frame_max_y );

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
        composite_control_editor< rs2_improved_close_range_control > _improved_close_range_editor;

        // Per-field manual-entry toggle state for the Disparity Shift / Threshold sliders (see
        // draw_improved_close_range_control_editor()) - mirrors option_model's own edit_mode/edit_value pattern
        // (common/option-model.cpp) rather than relying on ImGui's native SliderInt Ctrl+Click/
        // double-click text-input, which turned out not to be reliably discoverable/usable here.
        bool _improved_close_range_shift_edit_mode = false;
        std::string _improved_close_range_shift_edit_buf;
        bool _improved_close_range_threshold_edit_mode = false;
        std::string _improved_close_range_threshold_edit_buf;
    };
}
