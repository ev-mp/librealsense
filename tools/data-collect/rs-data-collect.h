// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.
// The utility shall be used to collect and analyze Realsense Cameras performance.
// Therefore no UI is provided, and the data is gathered and serialized in csv-compatible format for offline analysis.
// The utility is configured via command-line parameters and requires user-provided configuration file to run
// The following lines provide examples of the configurations available
/*
//Defines the order at which the params (comma-separated) should appear in configuration file
//#rs2_stream, width, height, fps, rs2_format, stream_index (optional, put zero or leave empty if you're not sure)
//# Examples (note that all lines starting with non-alpha character (#@!...) will be skipped:
#D435/415
DEPTH,640,480,30,Z16,0
INFRARED,640,480,30,Y8,1
INFRARED,640,480,30,Y8,2
COLOR,640,480,30,RGB8,0
##D435i-specific
ACCEL,1,1,63,MOTION_XYZ32F
GYRO,1,1,200,MOTION_XYZ32F
#T265
#FISHEYE,848,800,30,Y8,1
#FISHEYE,848,800,30,Y8,2
#ACCEL,1,1,62,MOTION_XYZ32F,0
#GYRO,1,1,200,MOTION_XYZ32F,0
#POSE,0,0,262,6DOF,0
*/

#include <librealsense2/rs.hpp>
#include "tclap/CmdLine.h"
#include <fstream>
#include <sstream>
#include <map>
#include <mutex>


using namespace std;
using namespace TCLAP;

namespace rs_data_collect
{
    const uint64_t  DEF_FRAMES_NUMBER = 100;
    constexpr uint32_t HIST_INTERVALS = 32; // Use 30 bins + 2 empty bins to wrap min/max boundaries
    const std::string DEF_OUTPUT_FILE_NAME("frames_data.csv");
    const uint8_t hw_intervals = 0;
    const uint8_t be_intervals = 1;
    const uint8_t host_intervals = 2;

    static const std::map<uint8_t, std::string> interval_names = {
    { hw_intervals,     "FW Intervals"},
    { be_intervals,     "Backend intervals"},
    { host_intervals,   "App Intervals"} };


    // Split string into token,  trim unreadable characters
    inline std::vector<std::string> tokenize(std::string line, char separator)
    {
        std::vector<std::string> tokens;

        // Remove trailing characters
        line.erase(line.find_last_not_of("\t\n\v\r ") + 1);

        stringstream ss(line);
        while (ss.good())
        {
            string substr;
            getline(ss, substr, separator);
            tokens.push_back(substr);
        }

        return tokens;
    }

    struct stringify
    {
        std::ostringstream ss;
        template<class T> stringify & operator << (const T & val) { ss << val; return *this; }
        operator std::string() const { return ss.str(); }
    };

    template <typename T>
    inline bool val_in_range(const T& val, const std::initializer_list<T>& list)
    {
        for (const auto& i : list) {
            if (val == i) {
                return true;
            }
        }
        return false;
    }

    inline int parse_number(char const *s, int base = 0)
    {
        char c;
        stringstream ss(s);
        int i;
        ss >> i;

        if (ss.fail() || ss.get(c))
        {
            throw runtime_error(string(string("Invalid numeric input - ") + s + string("\n")));
        }
        return i;
    }

    inline std::string to_lower(const std::string& s)
    {
        auto copy = s;
        std::transform(copy.begin(), copy.end(), copy.begin(), ::tolower);
        return copy;
    }

    inline rs2_format parse_format(const string str)
    {
        for (int i = RS2_FORMAT_ANY; i < RS2_FORMAT_COUNT; i++)
        {
            if (to_lower(rs2_format_to_string((rs2_format)i)) == str)
            {
                return (rs2_format)i;
            }
        }
        throw runtime_error((string("Invalid format - ") + str + string("\n")).c_str());
    }

    inline rs2_stream parse_stream_type(const string str)
    {
        for (int i = RS2_STREAM_ANY; i < RS2_STREAM_COUNT; i++)
        {
            if (to_lower(rs2_stream_to_string((rs2_stream)i)) == str)
            {
                return static_cast<rs2_stream>(i);
            }
        }
        throw runtime_error((string("Invalid stream type - ") + str + string("\n")).c_str());
    }

    inline int parse_fps(const string str)
    {
        return parse_number(str.c_str());
    }

    inline std::string get_profile_description(const rs2::stream_profile& profile)
    {
        std::stringstream ss;

        ss  << profile.stream_name() << ","
            << profile.stream_type() << ","
            << profile.format() << ","
            << profile.fps();

        if (auto vp = profile.as<rs2::video_stream_profile>())
            ss << "," << vp.width() << "," << vp.height();

        ss << std::endl;
        return ss.str().c_str();
    }

    struct stream_request
    {
        rs2_stream  _stream_type;
        rs2_format  _stream_format;
        int         _width;
        int         _height;
        int         _fps;
        int         _stream_idx;
    };

    inline std::ostream&  operator<<(std::ostream& os, const stream_request& req)
    {
        return os << "Type: " << req._stream_type << ", Idx: " << req._stream_idx << ", "
            << req._stream_format << ", [" << req._width << "x" << req._height << "], " << req._fps << "fps" << std::endl;
    }


    enum config_params {
        e_stream_type,
        e_res_width,
        e_res_height,
        e_fps,
        e_format,
        e_stream_index
    };

    enum application_stop : uint8_t {
        stop_on_frame_num,
        stop_on_timeout,
        stop_on_user_frame_num,
        stop_on_any
    };


    class data_collector
    {
    public:
        data_collector(std::shared_ptr<rs2::device> dev,
            ValueArg<int>& timeout, ValueArg<int>& max_frames);
        ~data_collector(){};
        data_collector(const data_collector&);

        void parse_and_configure(ValueArg<string>& config_file);
        void save_data_to_file(const string& out_filename);
        void collect_frame_attributes(rs2::frame f, std::chrono::time_point<std::chrono::high_resolution_clock> start_time);
        bool collecting(std::chrono::time_point<std::chrono::high_resolution_clock> start_time);

        const std::vector<rs2::sensor>& selected_sensors() const { return active_sensors; };

        //template<typename S, class Field>
        struct frame_record
        {
            frame_record(unsigned long long frame_number, long long frame_number_delta,
                        double frame_ts, double frame_ts_delta, double backend_ts, double backend_ts_delta,
                        double host_ts, double host_ts_delta,
                        rs2_timestamp_domain domain, rs2_stream stream_type,int stream_index,
                        double _p1=0., double _p2=0., double _p3=0.,
                        double _p4=0., double _p5=0., double _p6=0., double _p7=0.):
            _frame_number(frame_number),
            _frame_number_delta(frame_number_delta),
            _ts(frame_ts),
            _ts_delta(frame_ts_delta),
            _be_ts(backend_ts),
            _be_ts_delta(backend_ts_delta),
            _arrival_time(host_ts),
            _arrival_time_delta(host_ts_delta),
            _domain(domain),
            _stream_type(stream_type),
            _stream_idx(stream_index),
            _params({_p1,_p2,_p3,_p4,_p5,_p6,_p7})
            {};

            std::string to_string() const
            {
                std::stringstream ss;
                ss  << std::endl
                    <<  rs2_stream_to_string(_stream_type) << ","
                    << _stream_idx << "," << _frame_number << "," << _frame_number_delta << ","
                    << std::fixed << std::setprecision(3) << _ts << "," << _ts_delta << ","
                    << std::fixed << std::setprecision(0) << _be_ts << "," << _be_ts_delta << ","
                    << std::fixed << std::setprecision(3) << _arrival_time << "," << _arrival_time_delta;

                // IMU and Pose frame hold the sample data in addition to the frame's header attributes
                size_t specific_attributes = 0;
                if (val_in_range(_stream_type,{RS2_STREAM_GYRO,RS2_STREAM_ACCEL}))
                    specific_attributes = 3;
                if (val_in_range(_stream_type,{RS2_STREAM_POSE}))
                    specific_attributes = 7;

                for (auto i=0UL; i<specific_attributes; i++)
                    ss << "," << _params[i];

                return ss.str().c_str();
            }

            unsigned long long      _frame_number;
            long long               _frame_number_delta;// Interval with previous sample
            double                  _ts;                // Device-based timestamp. (msec).
            double                  _ts_delta;          // Interval with previous sample (msec)
            double                  _be_ts;             // UVC-Driver time of arrival (msec)
            double                  _be_ts_delta;       // Interval with previous sample (msec)
            double                  _arrival_time;      // Host arrival timestamp, relative to start streaming (msec)
            double                  _arrival_time_delta;// Interval with previous sample (msec)
            rs2_timestamp_domain    _domain;            // The origin of device-based timestamp. Note that Device timestamps may require kernel patches
            rs2_stream              _stream_type;
            int                     _stream_idx;
            std::array<double,7>    _params;            // |The parameters are optional and sensor specific
        };


        struct intervals_stats
        {
            double _interval_min, _interval_max, _interval_average, _interval_mean,interval_stdev;
            std::array<size_t, HIST_INTERVALS> _interval_hist;
            std::array<double, HIST_INTERVALS> _interval_bins;
            std::vector<double> _intervals_filtered;
        };

        enum drop_reason  { frame_drop, counter_stuck, counter_inconsistent };

        struct frame_drop_data
        {
            uint64_t    prev_fn;
            uint64_t    current_fn;
            double      prev_ts;
            double      cur_ts;
            drop_reason reason;
        };

        struct stream_statistics
        {
            std::map<int64_t, uint32_t> _frame_drops_summary;                           // Frame diff: number of occurances
            std::vector<std::pair<uint64_t, frame_drop_data>> _frame_drops_occurances;  // Frame index at occurance: diff data
            std::array<intervals_stats, 3> _intervals;                                  // Stats on FW-generated, Back-end and User-space timestamps
            bool valid = false;
        };

    private:

        std::shared_ptr<rs2::device>        _dev;
        std::map<std::pair<rs2_stream, int>, std::vector<frame_record>> data_collection;
        std::map<std::pair<rs2_stream, int>, stream_statistics > stats_collection;
        std::vector<stream_request>         requests_to_go, user_requests;
        std::vector<rs2::sensor>            active_sensors;
        std::vector<rs2::stream_profile>    selected_stream_profiles;
        uint64_t                            _max_frames;
        int64_t                             _time_out_sec;
        application_stop                    _stop_cond;
        mutable std::recursive_mutex        _mtx;

        bool parse_configuration(const std::string& line, const std::vector<std::string>& tokens,
            rs2_stream& type, int& width, int& height, rs2_format& format, int& fps, int& index);

        stream_statistics calculate_stream_statistics(rs2_stream stream, const std::vector<frame_record>& input) const;

        // Assign the user configuration to the selected device
        bool configure_sensors();
    };
}
