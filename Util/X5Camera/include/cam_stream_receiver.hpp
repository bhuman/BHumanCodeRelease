/*
 * cam_stream_receiver.hpp – Orin-side C++ library for receiving stereo NV12
 * frames streamed by cam_stream on the X5 board.
 *
 * Drop cam_stream_receiver.hpp + cam_stream_receiver.cpp into your Orin
 * project and add them to your build.  No extra dependencies beyond a C++17
 * compiler and POSIX sockets.
 *
 * Typical usage:
 *
 *   #include "cam_stream_receiver.hpp"
 *
 *   cam_stream::CamStreamReceiver rx("192.168.127.10");
 *
 *   rx.onCalibration([](const cam_stream::StereoCalibration &c) {
 *       // c.left.fx, c.left.d[], c.R[], c.T[], …
 *   });
 *   rx.onStereoFrame([](const cam_stream::StereoFrame &f) {
 *       // f.left.nv12  – raw NV12 bytes for the left channel
 *       // f.right.nv12 – raw NV12 bytes for the right channel
 *       // f.left.frame_id == f.right.frame_id (hardware-synchronised)
 *   });
 *   rx.start();           // non-blocking; receiver runs in its own thread
 *   // … do work …
 *   rx.stop();
 *
 * Calibration is sent by cam_stream once per TCP connection, immediately
 * before the first frame.  Use onCalibration() for an asynchronous
 * notification or getCalibration() to poll from any thread.
 *
 * The callbacks are invoked from the receiver thread.  Copy data before
 * returning if you need it to outlive the callback.
 *
 * Individual (unpaired) frames can also be consumed via onFrame().
 * Both frame callbacks may be registered simultaneously.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cam_stream {

// ── single camera frame ───────────────────────────────────────────────────────
struct CamFrame {
    uint8_t  channel;       // 0 = left, 1 = right
    uint32_t frame_id;
    uint64_t timestamp_ns;  // hardware timestamp, nanoseconds
    uint16_t width;
    uint16_t height;
    std::vector<uint8_t> nv12; // Y plane (w×h bytes) then interleaved UV (w×h/2)
};

// ── matched stereo pair (left.frame_id == right.frame_id) ────────────────────
struct StereoFrame {
    CamFrame left;
    CamFrame right;
};

// ── camera calibration ────────────────────────────────────────────────────────
struct CameraIntrinsics {
    double  fx, fy;       // focal lengths (pixels)
    double  cx, cy;       // principal point (pixels)
    double  d[8];         // distortion coefficients (first d_count are valid)
    uint8_t d_count;      // number of valid entries in d[]: 4, 5, or 8
};

// Stereo calibration as streamed by cam_stream on the X5.
// Matches the YAML produced by cam_eeprom and the camera driver.
// R and T follow the standard OpenCV stereoRectify convention (right→left).
struct StereoCalibration {
    uint16_t         width;                // calibration image width  (pixels)
    uint16_t         height;               // calibration image height (pixels)
    char             distortion_model[24]; // null-terminated:
                                           //   "plumb_bob" | "equidistant" |
                                           //   "rational_polynomial"
    CameraIntrinsics left;
    CameraIntrinsics right;
    double           R[9]; // 3×3 rotation matrix (row-major), right→left
    double           T[3]; // translation vector,              right→left
};

// Intrinsics for the scaled and rectified image as it arrives on the Orin.
// Computed by the sender via cv::stereoRectify at capture resolution, so
// these values can be used directly without any further scaling or
// undistortion on the receiver side.
struct RectifiedIntrinsics {
    uint16_t width, height; // capture resolution these intrinsics apply to
    double   fx, fy;        // focal length in pixels (shared by both cameras)
    double   cx, cy;        // principal point in pixels
    double   baseline;      // stereo baseline in metres (positive)
    // 4×4 disparity-to-depth mapping matrix (row-major).
    // Use with cv::reprojectImageTo3D(disparity, xyz, Q).
    double   Q[16];
};

using StereoCallback      = std::function<void(const StereoFrame &)>;
using FrameCallback       = std::function<void(const CamFrame &)>;
using CalibrationCallback = std::function<void(const StereoCalibration &)>;

// ── receiver ─────────────────────────────────────────────────────────────────
class CamStreamReceiver {
public:
    // host: IP address of the X5 board on the USB-ethernet link (e.g. "192.168.127.10")
    // port: must match --port passed to cam_stream (default 7654)
    explicit CamStreamReceiver(std::string host, uint16_t port = 7654);
    ~CamStreamReceiver();

    // Register callbacks before calling start().
    // onCalibration: fired once per connection when the sender transmits
    //                calibration data (before the first frame).
    // onStereoFrame: fired once per matched left+right pair.
    // onFrame:       fired for every individual frame regardless of pairing.
    // All callbacks may be set simultaneously.
    void onCalibration(CalibrationCallback cb);
    void onStereoFrame(StereoCallback cb);
    void onFrame(FrameCallback cb);

    // Returns true and fills `out` if calibration has been received at least
    // once since start().  Thread-safe; may be called at any time.
    bool getCalibration(StereoCalibration &out) const;

    // Returns true and fills `out` with intrinsics for the scaled, rectified
    // image.  These are computed by the sender at capture resolution via
    // cv::stereoRectify (alpha=0, CALIB_ZERO_DISPARITY), so fx/fy/cx/cy can
    // be used directly for 3-D reconstruction without further processing.
    // Thread-safe; may be called at any time after start().
    bool getRectifiedIntrinsics(RectifiedIntrinsics &out) const;

    // Start the background receiver thread.  Connects and auto-reconnects.
    void start();

    // Stop the background receiver thread and block until it exits.
    void stop();

    struct Stats {
        uint64_t frames_received;   // total individual frames (both channels)
        uint64_t stereo_pairs;      // matched left+right pairs delivered
        uint64_t dropped_unmatched; // frames discarded without finding a match
        uint64_t reconnects;        // number of TCP connections established
    };

    // Snapshot of cumulative counters since start() (thread-safe).
    Stats getStats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace cam_stream
