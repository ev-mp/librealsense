// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "structured-xu-control.h"

#include <stdexcept>
#include <cstring>

namespace librealsense {

std::vector< uint8_t > xu_structured_control::get_raw( platform::uvc_device & dev ) const
{
    std::vector< uint8_t > data( _wire_size );

    // Exactly one get_xu() call - the whole payload arrives atomically.
    if( ! dev.get_xu( _xu, _ctrl_id, data.data(), (int)_wire_size ) )
        throw std::runtime_error( "get_xu() failed" );

    return data;
}

void xu_structured_control::set_raw( platform::uvc_device & dev, const void * data, uint32_t data_size ) const
{
    if( data_size != _wire_size )
        throw std::runtime_error( "xu_structured_control::set_raw: data_size does not match the control's wire size" );

    // Exactly one set_xu() call - the whole payload sent together, atomically.
    if( ! dev.set_xu( _xu, _ctrl_id, reinterpret_cast< const uint8_t * >( data ), (int)_wire_size ) )
        throw std::runtime_error( "set_xu() failed" );
}

}  // namespace librealsense
