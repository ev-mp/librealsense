// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved.
// Minimalistic command-line collect & analyze bandwidth/performance tool for Realsense Cameras.
// The data is gathered and serialized in csv-compatible format for offline analysis.
// Extract and store frame headers info for video streams; for IMU&Tracking streams also store the actual data
// The utility is configured with command-line keys and requires user-provided config file to run
// See rs-data-collect.h for config examples

#include <librealsense2/rs.hpp>
#include "rs-data-collect.h"
#include "tclap/CmdLine.h"
#include <thread>
#include <regex>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;
using namespace TCLAP;
using namespace rs_data_collect;


data_collector::data_collector(std::shared_ptr<rs2::device> dev,
    ValueArg<int>& timeout, ValueArg<int>& max_frames) : _dev(dev)
{
    _max_frames = max_frames.isSet() ? max_frames.getValue() :
        timeout.isSet() ? std::numeric_limits<uint64_t>::max() : DEF_FRAMES_NUMBER;
    _time_out_sec = timeout.isSet() ? timeout.getValue() : -1;

    _stop_cond = static_cast<application_stop>((int(max_frames.isSet()) << 1) + int(timeout.isSet()));
}

void data_collector::parse_and_configure(ValueArg<string>& config_file)
{
    if (!config_file.isSet())
        throw std::runtime_error(stringify()
            << "The tool requires a profile configuration file to proceed"
            << "\nRun rs-data-collect --help for details");

    const std::string config_filename(config_file.getValue());
    ifstream file(config_filename);

    if (!file.is_open())
        throw runtime_error(stringify() << "Given .csv configure file " << config_filename << " was not found!");

    // Line starting with non-alpha characters will be treated as comments
    const static std::regex starts_with("^[a-zA-Z]");
    string line;
    rs2_stream stream_type;
    rs2_format format;
    int width{}, height{}, fps{}, index{};

    // Parse the config file
    while (getline(file, line))
    {
        auto tokens = tokenize(line, ',');

        if (std::regex_search(line, starts_with))
        {
            if (parse_configuration(line, tokens, stream_type, width, height, format, fps, index))
                user_requests.push_back({ stream_type, format, width, height, fps,  index });
        }
    }

    // Sanity test agains multiple conflicting requests for the same sensor
    if (user_requests.size())
    {
        std::sort(user_requests.begin(), user_requests.end(),
            [](const stream_request& l, const stream_request& r) { return l._stream_type < r._stream_type; });

        for (auto i = 0UL; i < user_requests.size() - 1; i++)
        {
            if ((user_requests[i]._stream_type == user_requests[i + 1]._stream_type) && ((user_requests[i]._stream_idx == user_requests[i + 1]._stream_idx)))
                throw runtime_error(stringify() << "Invalid configuration file - multiple requests for the same sensor:\n"
                    << user_requests[i] << user_requests[+i]);
        }
    }
    else
        throw std::runtime_error(stringify() << "Invalid configuration file - " << config_filename << " zero requests accepted");

    // Assign user profile to actual sensors
    configure_sensors();

    // Report results
    std::cout << "\nDevice selected: \n\t" << _dev->get_info(RS2_CAMERA_INFO_NAME)
        << (_dev->supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) ?
            std::string((stringify() << ". USB Type: " << _dev->get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))) : "")
        << "\n\tS.N: " << (_dev->supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? _dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "")
        << "\n\tFW Ver: " << _dev->get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
        << "\n\nUser streams requested: " << user_requests.size()
        << ", actual matches: " << selected_stream_profiles.size() << std::endl;

    if (requests_to_go.size())
    {
        std::cout << "\nThe following request(s) are not supported by the device: " << std::endl;
        for (auto& elem : requests_to_go)
            std::cout << elem;
    }
}

void data_collector::save_data_to_file(const string& out_filename)
{
    if (!data_collection.size())
        throw runtime_error(stringify() << "No data collected, aborting");

    // Report amount of frames collected
    std::vector<uint64_t> frames_per_stream;
    for (const auto& kv : data_collection)
        frames_per_stream.emplace_back(kv.second.size());

    std::sort(frames_per_stream.begin(), frames_per_stream.end());
    std::cout << "\nData collection accomplished with ["
        << frames_per_stream.front() << "-" << frames_per_stream.back()
        << "] frames recorded per stream\nSerializing captured results to "
        << out_filename << std::endl;

    // Serialize and store data into csv-like format
    ofstream csv(out_filename);
    if (!csv.is_open())
        throw runtime_error(stringify() << "Cannot open the requested output file " << out_filename << ", please check permissions");


    csv << "Configuration:\nStream Type,Stream Name,Format,FPS,Width,Height\n";
    for (const auto& elem : selected_stream_profiles)
        csv << get_profile_description(elem);

    // Calculate frames statistics
    for (const auto& elem : data_collection)
    {
        stats_collection[elem.first] = calculate_stream_statistics(elem.first.first, elem.second);
    }

    // Print out all stats info
    for (const auto& elem : stats_collection)
    {
        auto& stat = elem.second;
        //Evgeni elem represent a stream with all its frames recorded
        if (stat.valid)
        {
            csv << "\nStatistics for stream," << elem.first.first;
            csv << "\nFrames arrived:," << data_collection.at(elem.first).size() 
                << ",[" << data_collection.at(elem.first).front()._frame_number << "-"
                << data_collection.at(elem.first).back()._frame_number << "]";
            csv << "\nFrame Drop Summary:,";
            if (stat._frame_drops_summary.size())
            {
                "\nFrame number gap,";
                for (auto const& reg : stat._frame_drops_summary)  csv << reg.first << ",";
                csv << "\nOccurances,";
                for (auto const& reg : stat._frame_drops_summary)  csv << reg.second << ",";
                csv << std::endl;
            }
            else
                csv << "Zero frame drops\n" ;

            // Find the expected interval
            double expected_interval = 1;
            for (const auto& sp : selected_stream_profiles)
            {
                if (sp.stream_type() == elem.first.first)
                {
                    expected_interval = 1000. / sp.fps();
                    break;
                }
            }
            for (auto val : { hw_intervals, be_intervals, host_intervals })
            {
                auto data = stat._intervals[val];
                csv << "\n" << interval_names.at(val) << ",golden,min,max,mean,avg,stdev\n,";
                csv << expected_interval << ","
                    << data._interval_min << ","
                    << data._interval_max << ","
                    << data._interval_mean << ","
                    << data._interval_average << ","
                    << data.interval_stdev << ","
                << "\nBins:,";
                for (auto& val : data._interval_bins) csv << val << ",";
                csv << "\nOccurances:,";
                for (auto& val : data._interval_hist) csv << val << ",";
                csv << std::endl;
            }

        }
    }


    for (const auto& elem : data_collection)
    {
        // elem represent a stream with all its frames recorded
        csv << "\n\nStream Type,Index,F#,F#_diff,HW Timestamp (ms),HW TS_diff (ms),Backend Timestamp(ms),BE TS diff (ms), Host Timestamp(ms), Host TS diff (ms)"
            << (val_in_range(elem.first.first, { RS2_STREAM_GYRO,RS2_STREAM_ACCEL }) ? ",3DOF_x,3DOF_y,3DOF_z" : "")
            << (val_in_range(elem.first.first, { RS2_STREAM_POSE }) ? ",t_x,t_y,t_z,r_x,r_y,r_z,r_w" : "")
            << std::endl;

        for (auto i = 0UL; i < elem.second.size(); i++)
            csv << elem.second[i].to_string();
    }
}

void data_collector::collect_frame_attributes(rs2::frame f, std::chrono::time_point<std::chrono::high_resolution_clock> start_time)
{
    auto arrival_time = std::chrono::duration<double, std::milli>(chrono::high_resolution_clock::now() - start_time);
    auto stream_uid = std::make_pair(f.get_profile().stream_type(), f.get_profile().stream_index());
    double be_time = f.supports_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP) ? static_cast<double>(f.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP)) : 0.;

    if (data_collection[stream_uid].size() < _max_frames)
    {
        frame_record rec{ f.get_frame_number(), 0,
            f.get_timestamp(), 0.0,
            be_time, 0.0,
            arrival_time.count(), 0.0,
            f.get_frame_timestamp_domain(),
            f.get_profile().stream_type(),
            f.get_profile().stream_index() };

        // Assume that the frame extensions are unique
        if (auto motion = f.as<rs2::motion_frame>())
        {
            auto axes = motion.get_motion_data();
            rec._params = { axes.x, axes.y, axes.z };
        }

        if (auto pf = f.as<rs2::pose_frame>())
        {
            auto pose = pf.get_pose_data();
            rec._params = { pose.translation.x, pose.translation.y, pose.translation.z,
                    pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w };
        }

        // Calculate frame parameter delta and log along with the newly-arrived sample
        auto prev_rec = data_collection[stream_uid].size() ? data_collection[stream_uid].back() : rec;
        rec._frame_number_delta = rec._frame_number - prev_rec._frame_number;
        rec._ts_delta           = rec._ts           - prev_rec._ts;
        rec._be_ts_delta        = rec._be_ts        - prev_rec._be_ts;
        rec._arrival_time_delta = rec._arrival_time - prev_rec._arrival_time;

        data_collection[stream_uid].emplace_back(rec);
    }
}

bool data_collector::collecting(std::chrono::time_point<std::chrono::high_resolution_clock> start_time)
{
    bool timed_out = false;

    if (_time_out_sec > 0)
    {
        timed_out = (chrono::high_resolution_clock::now() - start_time) > std::chrono::seconds(_time_out_sec);
        // When the timeout is the only option is specified, disregard frame number
        if (stop_on_timeout == _stop_cond)
            return !timed_out;
    }

    bool collected_enough_frames = true;
    for (auto&& profile : selected_stream_profiles)
    {
        auto key = std::make_pair(profile.stream_type(), profile.stream_index());
        if (!data_collection.size() || (data_collection.find(key) != data_collection.end() &&
            (data_collection[key].size() && data_collection[key].size() < _max_frames)))
        {
            collected_enough_frames = false;
            break;
        }
    }

    return !(timed_out || collected_enough_frames);

}

bool data_collector::parse_configuration(const std::string& line, const std::vector<std::string>& tokens,
    rs2_stream& type, int& width, int& height, rs2_format& format, int& fps, int& index)
{
    bool res = false;
    try
    {
        auto tokens = tokenize(line, ',');
        if (tokens.size() < e_stream_index)
            return res;

        // Convert string to uppercase
        type = parse_stream_type(to_lower(tokens[e_stream_type]));
        width = parse_number(tokens[e_res_width].c_str());
        height = parse_number(tokens[e_res_height].c_str());
        fps = parse_fps(tokens[e_fps]);
        format = parse_format(to_lower(tokens[e_format]));
        // Backward compatibility
        if (tokens.size() > e_stream_index)
            index = parse_number(tokens[e_stream_index].c_str());
        res = true;
        std::cout << "Request added for " << line << std::endl;
    }
    catch (...)
    {
        std::cout << "Invalid syntax in configuration line " << line << std::endl;
    }

    return res;
}

data_collector::stream_statistics data_collector::calculate_stream_statistics(rs2_stream stream, const std::vector<frame_record>& input) const
{
    stream_statistics stats{};
    if (input.size() < 2)
    {
        std::cout << "Not enough frames for statistics for stream " << stream << std::endl;
    }
    else
    {
        std::array<std::thread, 3> workers;
        stats.valid = true;
        int index = 0;

        std::cout << "Calculating statistics for stream " << stream << std::endl;
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        for (double frame_record::* field : { &frame_record::_ts , &frame_record::_be_ts, &frame_record::_arrival_time, })
        {
            workers[index] = std::thread([this, input, field, &stats, index](){

                // Filter out only the sequential frames
                auto& stat = stats._intervals[index];
                auto& filtered = stats._intervals[index]._intervals_filtered;

                for (size_t i = 1; i < input.size(); i++)
                {
                    auto frame_diff = input[i]._frame_number - input[i - 1]._frame_number;
                    if (1 == frame_diff)
                    {
                        filtered.push_back(input[i].*field - input[i - 1].*field); // Trace only the sequential records
                    }
                    else // Frame drop occurred
                    {
                        if (index == 0) // The first worker thread will caclulate the drops statistics
                        {
                            auto reason = (frame_diff > 1) ? frame_drop : (frame_diff == 0 ? counter_stuck : counter_inconsistent);
                            frame_drop_data drop= { input[i - 1]._frame_number , input[i]._frame_number, input[i - 1]._ts , input[i]._ts, reason };
                            stats._frame_drops_occurances.push_back(std::make_pair(input[i - 1]._frame_number, drop));
                            // All inconsistencies will be gathered under the same category
                            if (frame_diff < 0 ) frame_diff = -1;
                            stats._frame_drops_summary[frame_diff] += 1;
                        }
                    }
                }

                // Calculate basic stats
                std::sort(filtered.begin(), filtered.end());
                stat._interval_min = filtered.front();
                stat._interval_max = filtered.back();
                //Debug
                if (stat._interval_max / 2 > stat._interval_min)
                    std::cout << "Suspicious log occurred";
                stat._interval_mean = filtered.at(filtered.size() / 2);
                auto avg = std::accumulate(filtered.begin(), filtered.end(), 0.0) / filtered.size();
                stat._interval_average = avg;

                // Calc stdev
                std::vector<double> diff(filtered.size());
                std::transform(filtered.begin(), filtered.end(), diff.begin(), [avg](double sample) { return sample - avg; });
                double sum_sq = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
                stat.interval_stdev = std::sqrt(sum_sq / diff.size());

                // Calc histogram for floating point using preallocated bins
                // Set bin values with last and first being empty (bins -2 borders -1 (++split))
                auto bin_size = (stat._interval_max - stat._interval_min) / (stat._interval_bins.size() - 3);
                auto current = stat._interval_min - ( 2* bin_size); // start with border (empty value)
                std::generate(stat._interval_bins.begin(), stat._interval_bins.end(), [&current, bin_size]() { current += bin_size; return current; });

                auto filt_iter = filtered.begin();
                auto bin_iter = stat._interval_bins.begin();
                int64_t count = 0;
                for (size_t i = 1; i < stat._interval_bins.size()-1; )
                {
                    std::cout << "Looking into bin " << i << " with lower bound " << stat._interval_bins[i] << std::endl;;
                    // Find first element that is greater than the upper bound of that bin
                    filt_iter = std::lower_bound(filt_iter, filtered.end(), stat._interval_bins[i]);
                    if (filt_iter != filtered.end())
                    {
                        auto dist = std::distance(filtered.begin(), filt_iter) +1; // count is zero-indexed
                        // Find the next bin to look up
                        bin_iter = std::upper_bound(bin_iter, stat._interval_bins.end(), *filt_iter);
                        i = std::distance(stat._interval_bins.begin(), bin_iter);
                        std::cout << " Val " << *filt_iter << " at index " << dist << " will be assigned to bin " << i << " with upper bound " << *bin_iter << std::endl;
                        stat._interval_hist[i-1] = dist - count;
                        count = dist;

                        std::cout << "Frames counted " << count << " Bin index = " << i << " Val: " << stat._interval_hist[i] << std::endl;;
                    }
                    else
                        i = stat._interval_bins.size();
                }
            });

            index++;
        }

        for (auto& worker : workers)
            if (worker.joinable())
                worker.join();
    }

    return stats;
}

// Assign the user configuration to the selected device
bool data_collector::configure_sensors()
{
    bool succeed = false;
    requests_to_go = user_requests;
    std::vector<rs2::stream_profile> matches;

    // Configure and starts streaming
    for (auto&& sensor : _dev->query_sensors())
    {
        // Disable performance-affecting settings
        // Disable Global Timer for to expose actual HW timestamps
        // Disable AE to avoid FPS changes due to ambient conditions - TODO need to recalculate and overwrite manual exposure for preset FPS.
        for (auto opt : {/*RS2_OPTION_ENABLE_AUTO_EXPOSURE,*/ RS2_OPTION_GLOBAL_TIME_ENABLED})
        {
            if (sensor.supports(opt))
            {
                if (sensor.get_option(opt))
                    sensor.set_option(opt, 0.f);
            }
        }

        for (auto& profile : sensor.get_stream_profiles())
        {
            // All requests have been resolved
            if (!requests_to_go.size())
                break;

            // Find profile matches
            auto fulfilled_request = std::find_if(requests_to_go.begin(), requests_to_go.end(), [&matches, profile](const stream_request& req)
            {
                bool res = false;
                if ((profile.stream_type() == req._stream_type) &&
                    (profile.format() == req._stream_format) &&
                    (profile.stream_index() == req._stream_idx) &&
                    (profile.fps() == req._fps))
                {
                    if (auto vp = profile.as<rs2::video_stream_profile>())
                    {
                        if ((vp.width() != req._width) || (vp.height() != req._height))
                            return false;
                    }
                    res = true;
                    matches.push_back(profile);
                }

                return res;
            });

            // Remove the request once resolved
            if (fulfilled_request != requests_to_go.end())
                requests_to_go.erase(fulfilled_request);
        }

        // Aggregate resolved requests
        if (matches.size())
        {
            std::copy(matches.begin(), matches.end(), std::back_inserter(selected_stream_profiles));
            sensor.open(matches);
            active_sensors.emplace_back(sensor);
            matches.clear();
        }

        if (selected_stream_profiles.size() == user_requests.size())
            succeed = true;
    }
    return succeed;
}

int main(int argc, char** argv) try
{
    rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG);

    // Parse command line arguments
    CmdLine cmd("librealsense rs-data-collect example tool", ' ');
    ValueArg<int>    timeout("t", "Timeout", "Max amount of time to receive frames (in seconds)", false, 10, "");
    ValueArg<int>    max_frames("m", "MaxFrames_Number", "Maximum number of frames-per-stream to receive", false, 100, "");
    ValueArg<string> out_file("f", "FullFilePath", "the file where the data will be saved to", false, "", "");
    ValueArg<string> config_file("c", "ConfigurationFile", "Specify file path with the requested configuration", false, "", "");

    cmd.add(timeout);
    cmd.add(max_frames);
    cmd.add(out_file);
    cmd.add(config_file);
    cmd.parse(argc, argv);

    std::cout << "Running rs-data-collect: ";
    for (auto i=1; i < argc; ++i)
        std::cout << argv[i] << " ";
    std::cout << std::endl << std::endl;

    auto output_file       = out_file.isSet() ? out_file.getValue() : DEF_OUTPUT_FILE_NAME;

    {
        ofstream csv(output_file);
        if (!csv.is_open())
            throw runtime_error(stringify() << "Cannot open the requested output file " << output_file << ", please check permissions");
    }

    bool succeed = false;
    rs2::context ctx;
    rs2::device_list list;

    while (!succeed)
    {
        list = ctx.query_devices();

        if (0== list.size())
        {
            std::cout << "Connect Realsense Camera to proceed" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
            continue;
        }

        auto dev = std::make_shared<rs2::device>(list.front());

        data_collector  dc(dev,timeout,max_frames);         // Parser and the data aggregator

        dc.parse_and_configure(config_file);

        for (auto&& sensor : dc.selected_sensors())
        {
            if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
                sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED,0.f);
        }
        //data_collection buffer;
        auto start_time = chrono::high_resolution_clock::now();

        // Start streaming
        for (auto&& sensor : dc.selected_sensors())
            sensor.start([&dc,&start_time](rs2::frame f)
        {
            dc.collect_frame_attributes(f,start_time);
        });

        std::cout << "\nData collection started.... \n" << std::endl;

        while (dc.collecting(start_time))
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << "Collecting data for "
                    << chrono::duration_cast<chrono::seconds>(chrono::high_resolution_clock::now() - start_time).count()
                    << " sec" << std::endl;
        }

        // Stop & flush all active sensors
        for (auto&& sensor : dc.selected_sensors())
        {
            sensor.stop();
            sensor.close();
        }

        dc.save_data_to_file(output_file);

        succeed = true;
    }

    std::cout << "Task completed" << std::endl;

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << endl;
    return EXIT_FAILURE;
}
catch (const exception & e)
{
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}
