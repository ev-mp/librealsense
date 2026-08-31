// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>
#include <imgui.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <string>

namespace rs2
{
    // Shared "under change" color ramp - gold (just touched) fading to the panel's own
    // ImGuiCol_FrameBgHovered blue (about to commit) - used by the framed group's own fill in
    // end_frame_and_maybe_commit() below, and by anything else (e.g. the row header's enable
    // toggle) that wants to visually echo the same pending-auto-commit state. `progress` is
    // 0 = just touched/still editing, 1 = about to fire.
    inline ImVec4 composite_control_dirty_blend( float progress )
    {
        ImVec4 start_color( 255.f / 255.f, 210.f / 255.f, 40.f / 255.f, 90.f / 255.f );   // bright gold/yellow
        constexpr float initial_brightness = 1.2f;   // "under change" color, 20% brighter
        start_color.x = std::min( start_color.x * initial_brightness, 1.0f );
        start_color.y = std::min( start_color.y * initial_brightness, 1.0f );
        start_color.z = std::min( start_color.z * initial_brightness, 1.0f );
        const ImVec4 target_blue = ImGui::GetStyle().Colors[ImGuiCol_FrameBgHovered];
        return ImVec4(
            start_color.x + ( target_blue.x - start_color.x ) * progress,
            start_color.y + ( target_blue.y - start_color.y ) * progress,
            start_color.z + ( target_blue.z - start_color.z ) * progress,
            start_color.w + ( target_blue.w - start_color.w ) * progress );
    }

    // Reusable debounced-auto-commit editor for a composite option's local
    // struct T. Extracted from the original HKR Improved Close Range Control implementation so any other
    // multi-param composite option can get the same behavior - touch -> fade in -> reset on
    // retouch -> hard commit on lapse - by holding one of these instead of re-implementing the
    // mechanism. See embedded_filter_model::draw_improved_close_range_control_editor() for the reference usage.
    //
    // The whole framed group is treated as ONE editing unit, matching T being one atomic
    // multi-field struct sent in a single UVC transaction - there is no per-field Send.
    //
    // Per-frame usage:
    //   1. ensure_initialized(filter, id, error_message) - seeds `value` from a GET, once.
    //   2. Draw the control-specific widgets against `value`. For each one:
    //        - call touch() on every change while the edit is in progress (e.g. every tick of a
    //          slider drag) - this flags the group dirty and defers any pending commit;
    //        - call finalize() once that field's edit is done (slider release via
    //          ImGui::IsItemDeactivatedAfterEdit(), or immediately after a checkbox/radio click,
    //          which has no drag phase so touch+finalize happen together) - this (re)schedules
    //          the real auto-commit `commit_delay` seconds out.
    //   3. end_frame_and_maybe_commit(filter, id, error_message, frame_min, frame_max,
    //      any_field_active_this_frame, before_commit) - draws the fading fill + shrinking border
    //      around the widgets' bounding box (continuously, for the full commit_delay window - no
    //      early freeze), shows the hover tooltip, and sends `value` to the device in one atomic
    //      write once the debounce timer has lapsed with no further touch() calls.
    //      `any_field_active_this_frame` is true if any of THIS control's own widgets is the one
    //      currently active (mid-drag/mid-click) - pass false once focus has genuinely left the
    //      group (some OTHER widget elsewhere is now active) to finish the countdown immediately
    //      instead of making the user wait it out. `before_commit`, if set, is called with `value`
    //      right before it's actually sent - this control's own code (which knows T's fields,
    //      unlike this generic class) is the right place to enforce cross-field invariants, e.g.
    //      "editing any field implies the control should be enabled too".
    template< typename T >
    class composite_control_editor
    {
    public:
        T value{};
        bool initialized = false;

        // Seeds `value` from a real GET the first time this is called; a no-op afterward. Returns
        // whether `value` is safe to use (false only if the initial GET failed). `on_read`, if
        // set, is called with the freshly-read `value` right after this GET succeeds - the
        // caller (which knows T's fields, unlike this generic class) is the right place to log
        // what actually came back from the device.
        bool ensure_initialized( const std::shared_ptr< rs2::embedded_filter > & filter,
                                  rs2_composite_option_id id,
                                  std::string & error_message,
                                  const std::function< void( const T & ) > & on_read = nullptr )
        {
            if( initialized )
                return true;
            try
            {
                value = filter->get_composite_option_as< T >( id );
                initialized = true;
                if( on_read )
                    on_read( value );
            }
            catch( const std::exception & e )
            {
                error_message = e.what();
            }
            return initialized;
        }

        // Call while a field is actively being changed (every tick of a slider drag, or a
        // checkbox/radio click). Flags the group dirty and parks the deadline at +infinity so
        // nothing commits mid-edit.
        void touch()
        {
            _dirty = true;
            _commit_deadline = std::numeric_limits< double >::max();
        }

        // Call once a field's edit is finalized (slider released, or immediately after a
        // checkbox/radio click). Schedules the auto-commit out by commit_delay seconds, or by
        // the much shorter fast_commit_delay when use_fast_delay is set - the caller's own way
        // of saying "this particular edit came from a keyboard arrow-key nudge, not a mouse
        // click/drag," which should feel closer to immediate feedback than the deliberate,
        // give-me-a-moment-to-change-my-mind pause a mouse edit gets.
        void finalize( bool use_fast_delay = false )
        {
            _commit_deadline = ImGui::GetTime() + ( use_fast_delay ? fast_commit_delay : commit_delay );
            // A discrete edit (radio/checkbox click, keyboard arrow-key nudge) calls touch()+
            // finalize() together, synchronously, within the SAME frame as the
            // end_frame_and_maybe_commit() call below that just set this deadline - unlike a
            // mouse drag, which spans several frames with the widget genuinely active in
            // between, giving the focus-loss shortcut no chance to fire on that exact frame.
            // Suppress it for this one frame so it can't immediately collapse the deadline it
            // was never meant to see yet.
            _just_finalized = true;
        }

        // Side-effect-free readout of the same progress end_frame_and_maybe_commit() animates
        // with: false if nothing is pending, true with `progress` in [0,1] (0 = just
        // touched/still mid-edit, 1 = about to commit) otherwise. Lets other UI elements for the
        // same control (e.g. an enable toggle drawn elsewhere) mirror the pending-commit state
        // without duplicating the deadline math.
        bool try_get_progress( float & progress ) const
        {
            if( ! _dirty )
                return false;
            double remaining = _commit_deadline - ImGui::GetTime();
            double frac_remaining = std::min( std::max( remaining / commit_delay, 0.0 ), 1.0 );
            progress = static_cast< float >( 1.0 - frac_remaining );
            return true;
        }

        // Draws the dirty-state fill/border for [frame_min, frame_max] - call after drawing this
        // control's own fields, once their bounding box is known - and sends `value` to the
        // device in one atomic write if the debounce timer has lapsed since the last touch().
        void end_frame_and_maybe_commit( const std::shared_ptr< rs2::embedded_filter > & filter,
                                          rs2_composite_option_id id,
                                          std::string & error_message,
                                          const ImVec2 & frame_min,
                                          const ImVec2 & frame_max,
                                          bool any_field_active_this_frame,
                                          const std::function< void( T & ) > & before_commit = nullptr )
        {
            constexpr float base_border_thickness = 1.0f;
            float border_thickness = base_border_thickness;

            // While actively being edited (deadline parked at +infinity by touch()) the fill and
            // border sit at their starting intensity: 0% faded (bright gold), 400% border width.
            // Once a countdown is actually running (finalize() set a real deadline), both fade
            // smoothly over the ENTIRE commit_delay window, right up until the instant the commit
            // fires below - no early cap/freeze partway through, or the animation visibly stalls
            // for the remainder of the wait, which reads as a bug rather than a fade. The fill
            // cools from gold toward the border's own blue (ImGuiCol_FrameBgHovered - the same
            // color the border is drawn in, so the fill settles into harmony with it rather than
            // into the panel background); the border shrinks from 400% down to 250% of normal.
            // The commit itself then snaps both back to their plain idle look (no fill, 100%
            // border) in one abrupt jump rather than continuing the gentle fade, so the moment
            // something was actually sent reads as a distinct, more pronounced change than the
            // animation leading up to it.
            float progress = 0.0f;
            if( try_get_progress( progress ) )
            {
                ImVec4 blended = composite_control_dirty_blend( progress );

                ImGui::GetWindowDrawList()->AddRectFilled( frame_min, frame_max, ImGui::ColorConvertFloat4ToU32( blended ), 3.0f );
                border_thickness = base_border_thickness * ( border_start_scale + ( border_end_scale - border_start_scale ) * progress );
            }

            ImGui::GetWindowDrawList()->AddRect(
                frame_min, frame_max,
                ImGui::GetColorU32( ImGuiCol_FrameBgHovered ),
                3.0f,    // rounding
                0,       // flags
                border_thickness );

            if( ImGui::IsMouseHoveringRect( frame_min, frame_max ) )
            {
                try
                {
                    ImGui::SetTooltip( "%s", filter->get_composite_option_description( id ) );
                }
                catch( const std::exception & )
                {
                    // Best-effort tooltip only - a failure here shouldn't disrupt the editor.
                }
            }

            // Shortcut: focus genuinely left the group - some OTHER widget elsewhere is active
            // this frame, not just "nothing is active right now" (which is also true during the
            // normal quiet gap between finishing one field and touching the next one in THIS
            // group, and must NOT cut the wait short). When that happens, don't make the user
            // wait out the rest of the countdown - finish it now, same as if it had lapsed
            // naturally. Skipped on the one frame finalize() just ran on (see its comment) - a
            // discrete edit's own deadline must survive at least until the NEXT frame.
            if( _dirty && ! any_field_active_this_frame && ImGui::IsAnyItemActive() && ! _just_finalized )
                _commit_deadline = ImGui::GetTime();
            _just_finalized = false;

            // Fires once the countdown elapses quietly - checked every frame, so any fresh
            // touch() (which re-parks the deadline at +infinity) naturally defers this for as
            // long as the user keeps adjusting fields.
            if( _dirty && ImGui::GetTime() >= _commit_deadline )
            {
                try
                {
                    if( before_commit )
                        before_commit( value );
                    filter->set_composite_option_from( id, value );
                }
                catch( const std::exception & e )
                {
                    error_message = e.what();
                }
                _dirty = false;
                _commit_deadline = std::numeric_limits< double >::max();
            }
        }

    private:
        bool _dirty = false;
        double _commit_deadline = std::numeric_limits< double >::max();
        bool _just_finalized = false;

        static constexpr double commit_delay = 1.7;          // seconds of quiet before auto-sending
        static constexpr double fast_commit_delay = 0.1;     // ditto, for keyboard arrow-key nudges
        static constexpr float border_start_scale = 4.0f;    // 400% of normal width, right after an edit
        static constexpr float border_end_scale = 2.5f;      // 250% of normal width, right before commit
    };
}
