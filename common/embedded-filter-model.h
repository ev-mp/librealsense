// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_hkr_minz_control.h>
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

        // PROTOTYPE / DEMO: hardcoded editor for RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL
        // specifically, called from draw_options() when that id is among _composite_option_ids.
        // There is no generic per-field composite-option editor (would need per-struct schema
        // knowledge generic view code doesn't have) - this special-cases the one struct by id,
        // same way application code is expected to (see rs_hkr_minz_control.h).
        void draw_minz_control_editor( std::string & error_message );

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

        // PROTOTYPE / DEMO: mirrors _minz_editor's own pending-auto-commit state (see
        // composite_control_editor<T>::try_get_progress()) so the row header's enable toggle,
        // drawn elsewhere (device-model.cpp's draw_embedded_filters()), can visually echo the
        // same "about to send" fade the framed MinZ editor box shows. Hardcoded to MinZ same as
        // draw_minz_control_editor() - false for every other embedded filter.
        bool has_pending_composite_commit( float & progress ) const
        {
            return _minz_editor.try_get_progress( progress );
        }

        void embedded_filter_enable_disable(bool actual);

    private:
        // Per-field draw_minz_control_editor() helpers, each returning whether it's active this
        // frame. Not meant to be called elsewhere. frame_max is x/y floats to avoid pulling
        // imgui.h into this header.
        bool draw_minz_filter_type_field();
        bool draw_minz_downscale_ratio_field();
        bool draw_minz_shift_mode_field();
        bool draw_minz_threshold_mode_field();
        // Shared touch()/finalize() plumbing for the slider-enum fields above.
        void commit_slider_enum_edit( bool changed );
        void slider_enum_arrow_nudge( int & field, int min_v, int max_v );
        // Manual-entry escape hatch shared by Shift Pixels and Threshold (mm).
        bool draw_minz_manual_input( const char * id, int & value, int min_v, int max_v,
                                      bool & edit_mode, std::string & edit_buf );
        bool draw_minz_slider_with_arrows( const char * id, int & value, int min_v, int max_v );
        bool draw_minz_manual_editable_field( const char * label,
                                               const char * id,
                                               int & value,
                                               int min_v,
                                               int max_v,
                                               bool & edit_mode,
                                               std::string & edit_buf );
        bool draw_minz_reset_to_default_overlay( rs2_composite_option_id id,
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

        // PROTOTYPE / DEMO: debounced auto-commit editor for the MinZ control, built on the
        // reusable composite_control_editor<T> mechanism (see composite-control-editor.h) - any
        // other multi-param composite option can get the same touch/fade/reset/commit behavior
        // by holding its own composite_control_editor<its-struct-type> the same way.
        composite_control_editor< rs2_minz_control > _minz_editor;

        // Manual-entry toggle state for Shift Pixels / Threshold (mm), reachable once
        // shift_mode/threshold_mode is set to "Manual". Mirrors option_model's edit_mode pattern.
        bool _minz_shift_edit_mode = false;
        std::string _minz_shift_edit_buf;
        bool _minz_threshold_edit_mode = false;
        std::string _minz_threshold_edit_buf;
    };
}
