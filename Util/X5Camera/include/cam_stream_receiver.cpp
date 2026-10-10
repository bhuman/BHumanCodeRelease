/*
 * cam_stream_receiver.cpp – Orin-side implementation.
 * See include/cam_stream_receiver.hpp for the public API.
 */
#include "cam_stream_receiver.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace cam_stream {

// ── wire protocol (must mirror cam_stream.cpp) ────────────────────────────────
//
// Every message on the stream starts with a 4-byte magic that identifies its
// type.  The receiver reads the magic first and then dispatches accordingly.
//
// FRAME packet  (FRAME_MAGIC):
//   uint32_t magic        = 0xC0FFEE42
//   uint8_t  channel      = 0 (left) | 1 (right)
//   uint32_t frame_id
//   uint64_t timestamp_ns
//   uint16_t width
//   uint16_t height
//   uint32_t payload_len  = width * height * 3 / 2
//   uint8_t  payload[payload_len]
//
// CALIB packet  (CALIB_MAGIC):
//   uint32_t magic        = 0xCA11B42E
//   uint32_t payload_len  = sizeof(CalibPayload)  (currently 318)
//   CalibPayload payload

static constexpr uint32_t FRAME_MAGIC    = 0xC0FFEE42u;
static constexpr uint32_t CALIB_MAGIC    = 0xCA11B42Eu;
static constexpr uint32_t MAX_PAYLOAD    = 4u * 1024u * 1024u;

// Bytes following the 4-byte magic in a FRAME packet.
struct __attribute__((packed)) FrameBody {
    uint8_t  channel;
    uint32_t frame_id;
    uint64_t timestamp_ns;
    uint16_t width;
    uint16_t height;
    uint32_t payload_len;
};
static_assert(sizeof(FrameBody) == 21, "FrameBody size mismatch");

// Bytes following the 4-byte magic + 4-byte payload_len in a CALIB packet.
// Must stay in sync with the CalibPayload in cam_stream.cpp.
struct __attribute__((packed)) CalibPayload {
    // ── raw calibration (calibration resolution) ──────────────────────────
    uint16_t width;
    uint16_t height;
    char     distortion_model[24]; // null-terminated model name
    uint8_t  d_num_l;
    uint8_t  d_num_r;
    double   l_fx, l_fy, l_cx, l_cy;
    double   l_d[8];
    double   r_fx, r_fy, r_cx, r_cy;
    double   r_d[8];
    double   R[9]; // 3×3 rotation, row-major, right→left
    double   T[3]; // translation, right→left
    // ── rectified intrinsics (capture resolution, post-stereoRectify) ─────
    uint16_t capture_width;
    uint16_t capture_height;
    double   rect_fx, rect_fy;  // shared focal length after rectification
    double   rect_cx, rect_cy;  // principal point
    double   rect_baseline;     // stereo baseline (metres, positive)
    double   Q[16];             // 4×4 disparity-to-depth matrix, row-major
};
static_assert(sizeof(CalibPayload) == 490, "CalibPayload size mismatch");

// ── recv helper ───────────────────────────────────────────────────────────────
// Reads exactly `len` bytes.  Returns false on disconnect, error, or if
// `running` goes false mid-read (checked after each short recv timeout).
static bool recv_all(int fd, void *buf, size_t len,
                     const std::atomic<bool> &running) {
    char *p = static_cast<char *>(buf);
    while (len > 0) {
        if (!running) return false;
        ssize_t n = recv(fd, p, len, 0);
        if (n > 0) { p += n; len -= (size_t)n; continue; }
        if (n == 0) return false; // graceful close
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
        return false;
    }
    return true;
}

// ── Impl ──────────────────────────────────────────────────────────────────────
struct CamStreamReceiver::Impl {
    const std::string host;
    const uint16_t    port;

    StereoCallback      stereo_cb;
    FrameCallback       frame_cb;
    CalibrationCallback calib_cb;

    std::atomic<bool> running{false};
    std::thread       rx_thread;

    // Stats
    std::atomic<uint64_t> stat_frames{0};
    std::atomic<uint64_t> stat_pairs{0};
    std::atomic<uint64_t> stat_dropped{0};
    std::atomic<uint64_t> stat_reconnects{0};

    // Calibration (written from rx_thread, readable from any thread)
    std::mutex           calib_mu;
    StereoCalibration    calib_data{};
    RectifiedIntrinsics  rect_data{};
    std::atomic<bool>    calib_valid{false};

    // Pairing state (only accessed from rx_thread)
    std::map<uint32_t, CamFrame> pending[2]; // pending[0]=left, pending[1]=right
    uint32_t latest_id{0};

    Impl(std::string h, uint16_t p) : host(std::move(h)), port(p) {}

    int connect_to_x5() {
        int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) return -1;

        // 200 ms recv timeout so recv_all can periodically check `running`.
        timeval tv{0, 200'000};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY,  &one, sizeof(one));
        setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE, &one, sizeof(one));

        // 8 MB receive buffer to absorb bursts.
        int rcvbuf = 8 * 1024 * 1024;
        setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
            close(fd); return -1;
        }
        if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
            close(fd); return -1;
        }
        return fd;
    }

    void handle_calib(const CalibPayload &cp) {
        StereoCalibration sc{};
        sc.width  = cp.width;
        sc.height = cp.height;
        std::memcpy(sc.distortion_model, cp.distortion_model,
                    sizeof(sc.distortion_model));
        sc.distortion_model[sizeof(sc.distortion_model) - 1] = '\0';

        sc.left.fx      = cp.l_fx;  sc.left.fy  = cp.l_fy;
        sc.left.cx      = cp.l_cx;  sc.left.cy  = cp.l_cy;
        sc.left.d_count = cp.d_num_l;
        std::memcpy(sc.left.d, cp.l_d, sizeof(sc.left.d));

        sc.right.fx      = cp.r_fx;  sc.right.fy  = cp.r_fy;
        sc.right.cx      = cp.r_cx;  sc.right.cy  = cp.r_cy;
        sc.right.d_count = cp.d_num_r;
        std::memcpy(sc.right.d, cp.r_d, sizeof(sc.right.d));

        std::memcpy(sc.R, cp.R, sizeof(sc.R));
        std::memcpy(sc.T, cp.T, sizeof(sc.T));

        RectifiedIntrinsics ri{};
        ri.width    = cp.capture_width;
        ri.height   = cp.capture_height;
        ri.fx       = cp.rect_fx;
        ri.fy       = cp.rect_fy;
        ri.cx       = cp.rect_cx;
        ri.cy       = cp.rect_cy;
        ri.baseline = cp.rect_baseline;
        std::memcpy(ri.Q, cp.Q, sizeof(ri.Q));

        {
            std::lock_guard<std::mutex> lk(calib_mu);
            calib_data = sc;
            rect_data  = ri;
        }
        calib_valid.store(true, std::memory_order_release);

        fprintf(stderr,
                "[cam_stream_receiver] calibration received:"
                " %ux%u  model=%s  rect: %ux%u  fx=%.2f  fy=%.2f"
                "  cx=%.2f  cy=%.2f  baseline=%.4f m\n",
                sc.width, sc.height, sc.distortion_model,
                ri.width, ri.height,
                ri.fx, ri.fy, ri.cx, ri.cy, ri.baseline);

        if (calib_cb) calib_cb(sc);
    }

    // Try to match the newly received frame with its partner.
    void try_pair(CamFrame frame) {
        const uint8_t  mine  = frame.channel;
        const uint8_t  other = mine ^ 1u;
        const uint32_t fid   = frame.frame_id;

        if (fid > latest_id) latest_id = fid;

        auto it = pending[other].find(fid);
        if (it != pending[other].end()) {
            StereoFrame sf;
            if (mine == 0) {
                sf.left  = std::move(frame);
                sf.right = std::move(it->second);
            } else {
                sf.left  = std::move(it->second);
                sf.right = std::move(frame);
            }
            pending[other].erase(it);
            stat_pairs.fetch_add(1, std::memory_order_relaxed);
            if (stereo_cb) stereo_cb(sf);
        } else {
            pending[mine][fid] = std::move(frame);
        }

        // Expire unpaired frames that are too far behind the latest to ever
        // find their partner (avoids unbounded map growth).
        if (latest_id >= 8) {
            const uint32_t cutoff = latest_id - 8;
            for (auto &pm : pending) {
                for (auto jt = pm.begin(); jt != pm.end(); ) {
                    if (jt->first < cutoff) {
                        stat_dropped.fetch_add(1, std::memory_order_relaxed);
                        jt = pm.erase(jt);
                    } else {
                        ++jt;
                    }
                }
            }
        }
    }

    void receive_loop() {
        while (running) {
            int fd = connect_to_x5();
            if (fd < 0) {
                // X5 not ready yet; retry after a short pause.
                for (int i = 0; i < 10 && running; i++)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            stat_reconnects.fetch_add(1, std::memory_order_relaxed);
            fprintf(stderr, "[cam_stream_receiver] connected to %s:%u\n",
                    host.c_str(), port);

            // Clear pairing state from any previous connection.
            pending[0].clear();
            pending[1].clear();

            while (running) {
                // Dispatch on 4-byte magic so the stream can carry both
                // frame packets and calibration packets.
                uint32_t magic;
                if (!recv_all(fd, &magic, sizeof(magic), running)) break;

                if (magic == FRAME_MAGIC) {
                    FrameBody body;
                    if (!recv_all(fd, &body, sizeof(body), running)) break;

                    if (body.channel > 1 || body.payload_len == 0 ||
                        body.payload_len > MAX_PAYLOAD) {
                        fprintf(stderr,
                                "[cam_stream_receiver] invalid frame header"
                                " – resyncing\n");
                        break;
                    }

                    CamFrame f;
                    f.channel      = body.channel;
                    f.frame_id     = body.frame_id;
                    f.timestamp_ns = body.timestamp_ns;
                    f.width        = body.width;
                    f.height       = body.height;
                    f.nv12.resize(body.payload_len);

                    if (!recv_all(fd, f.nv12.data(), f.nv12.size(), running))
                        break;

                    stat_frames.fetch_add(1, std::memory_order_relaxed);
                    if (frame_cb) frame_cb(f);
                    try_pair(std::move(f));

                } else if (magic == CALIB_MAGIC) {
                    uint32_t payload_len;
                    if (!recv_all(fd, &payload_len, sizeof(payload_len),
                                  running)) break;

                    if (payload_len != sizeof(CalibPayload)) {
                        fprintf(stderr,
                                "[cam_stream_receiver] unexpected calib size"
                                " %u (expected %zu) – resyncing\n",
                                payload_len, sizeof(CalibPayload));
                        break;
                    }

                    CalibPayload cp;
                    if (!recv_all(fd, &cp, sizeof(cp), running)) break;
                    handle_calib(cp);

                } else {
                    fprintf(stderr,
                            "[cam_stream_receiver] bad magic 0x%08X"
                            " – resyncing\n", magic);
                    break;
                }
            }

            close(fd);
            if (running)
                fprintf(stderr,
                        "[cam_stream_receiver] disconnected, retrying...\n");
        }
    }
};

// ── public API ────────────────────────────────────────────────────────────────
CamStreamReceiver::CamStreamReceiver(std::string host, uint16_t port)
    : impl_(std::make_unique<Impl>(std::move(host), port)) {}

CamStreamReceiver::~CamStreamReceiver() { stop(); }

void CamStreamReceiver::onCalibration(CalibrationCallback cb) {
    impl_->calib_cb = std::move(cb);
}

void CamStreamReceiver::onStereoFrame(StereoCallback cb) {
    impl_->stereo_cb = std::move(cb);
}

void CamStreamReceiver::onFrame(FrameCallback cb) {
    impl_->frame_cb = std::move(cb);
}

bool CamStreamReceiver::getCalibration(StereoCalibration &out) const {
    if (!impl_->calib_valid.load(std::memory_order_acquire))
        return false;
    std::lock_guard<std::mutex> lk(impl_->calib_mu);
    out = impl_->calib_data;
    return true;
}

bool CamStreamReceiver::getRectifiedIntrinsics(RectifiedIntrinsics &out) const {
    if (!impl_->calib_valid.load(std::memory_order_acquire))
        return false;
    std::lock_guard<std::mutex> lk(impl_->calib_mu);
    out = impl_->rect_data;
    return true;
}

void CamStreamReceiver::start() {
    impl_->running = true;
    impl_->rx_thread = std::thread([this] { impl_->receive_loop(); });

    // Elevate the receiver thread to SCHED_FIFO priority 10.  On a loaded
    // Jetson this prevents 50–100 ms scheduler preemptions that show up as
    // pair-interval spikes.  Requires CAP_SYS_NICE (run as root, or set the
    // capability on the binary with: setcap cap_sys_nice+ep <binary>).
    // Failure is non-fatal: we fall back silently to the default scheduler.
    sched_param sp{};
    sp.sched_priority = 10;
    if (pthread_setschedparam(impl_->rx_thread.native_handle(),
                              SCHED_FIFO, &sp) != 0) {
        fprintf(stderr,
                "[cam_stream_receiver] note: could not set SCHED_FIFO"
                " (run as root or: setcap cap_sys_nice+ep <binary>)."
                " Scheduler jitter may affect latency.\n");
    }
}

void CamStreamReceiver::stop() {
    if (!impl_->running) return;
    impl_->running = false;
    if (impl_->rx_thread.joinable()) impl_->rx_thread.join();
}

CamStreamReceiver::Stats CamStreamReceiver::getStats() const {
    return {
        impl_->stat_frames  .load(std::memory_order_relaxed),
        impl_->stat_pairs   .load(std::memory_order_relaxed),
        impl_->stat_dropped .load(std::memory_order_relaxed),
        impl_->stat_reconnects.load(std::memory_order_relaxed),
    };
}

} // namespace cam_stream
