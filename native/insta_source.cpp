// insta_source: minimal in-process Insta360 live-stream source.
//
// SDK video callback -> native openh264 decode (no-flush: the 1-frame hold is
// the verified floor) -> RGB latest-frame slot guarded by a mutex + condvar.
// Exposes a tiny C ABI consumed via ctypes by robocam.drivers.insta360.
//
// swscale converts straight to RGB24 rather than BGR24 so the Python side can
// hand robocam its `images={"rgb": ...}` with no cvtColor on the critical path.
//
// The latency methodology, the measured numbers, and the dead ends behind the
// design below are written up in the portable_data_collection repo, in
// docs/camera_latency_check.md and docs/insta360_ROS2_driver.md section 7.
//
// Latency-relevant design, all verified on the bench (2026-07-13):
// - frames are stamped from the SDK's per-AU device timestamp (capture-side,
//   ms since camera boot), mapped to host CLOCK_REALTIME by the MINIMUM
//   observed (arrival - device_ts) offset over the first ~90 AUs (a single-AU
//   anchor makes the eps constant vary ~15 ms per relaunch);
// - the capture stamp rides through the decoder as uiInBsTimeStamp so every
//   decoded frame recovers ITS OWN stamp regardless of codec buffering;
// - decode is single-threaded openh264 (~4.5 ms/frame at 1920x960) done
//   directly on the SDK callback thread; no queues anywhere - latest wins.
//
// Build: native/build.sh (links CameraSDK + openh264 + swscale). The Insta360
// CameraSDK is proprietary and is NOT vendored here; see the README section
// "Insta360 SDK setup" for how build.sh locates it.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>
#include <stream/stream_delegate.h>

#include <wels/codec_api.h>

extern "C" {
#include <libswscale/swscale.h>
#include <libavutil/pixfmt.h>
}

namespace {

int64_t NowRealtimeNs() {
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// Only what an X5 actually streams, verified against hardware 2026-08-21.
// Three entries were removed after testing all six the SDK enum offers:
//
//   RES_1440_720P30    Rejected as a main stream. The camera does not report
//   RES_1152_1152P30   this - it silently keeps its previous live-stream
//                      resolution, which persists across processes and power
//                      cycles, so the delivered size depends on session order.
//                      RES_1440_720P30 remains valid as an LRV resolution (see
//                      the hardcoded lrv_video_resulution below).
//
//                      Note this differs from the unknown-string path below,
//                      which requests RES_1920_960P30 and genuinely gets it.
//   RES_2880_2880P30   StartLiveStreaming rejects it, but only after
//                      SetVideoCaptureParams has already committed it. That
//                      write drops the camera out of Android USB mode and
//                      rewrites its normal-video resolution; recovery needs
//                      the on-camera settings menu, not a replug.
//
// The Python driver rejects unsupported strings before they reach here, so a
// stale .so built before this change still cannot request them.
const std::map<std::string, ins_camera::VideoResolution>& ResolutionMap() {
    static const std::map<std::string, ins_camera::VideoResolution> kMap = {
        {"3840x1920", ins_camera::VideoResolution::RES_3840_1920P30},
        {"2560x1280", ins_camera::VideoResolution::RES_2560_1280P30},
        {"1920x960",  ins_camera::VideoResolution::RES_1920_960P30},
    };
    return kMap;
}

struct Handle;

// Receives SDK callbacks; owns decode + the latest-frame slot.
class Delegate : public ins_camera::StreamDelegate {
public:
    explicit Delegate(Handle* h) : h_(h) {}
    // StreamDelegate has no virtual dtor; lifetime is managed through
    // shared_ptr<Delegate>'s type-erased deleter, so this is still safe.
    ~Delegate();

    void OnAudioData(const uint8_t*, size_t, int64_t) override {}
    void OnGyroData(const std::vector<ins_camera::GyroData>&) override {}
    void OnExposureData(const ins_camera::ExposureData&) override {}
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp,
                     uint8_t streamType, int stream_index) override;

    bool InitDecoder();

private:
    int64_t Stamp(int64_t sdk_ts_ms);

    Handle* h_;
    ISVCDecoder* dec_ = nullptr;
    SwsContext* sws_ = nullptr;
    int sws_w_ = 0, sws_h_ = 0;

    // Min-delay device->host clock anchor (see file header).
    static constexpr uint32_t kAnchorWindow = 90;
    int64_t anchor_offset_ns_ = 0;
    int64_t prev_sdk_ms_ = -1;
    int64_t last_stamp_ns_ = 0;
    uint32_t au_count_ = 0;
};

struct Handle {
    std::shared_ptr<ins_camera::Camera> cam;
    std::shared_ptr<ins_camera::StreamDelegate> delegate;  // actually Delegate

    // Latest-frame slot.
    std::mutex mu;
    std::condition_variable cv;
    std::vector<uint8_t> rgb;      // full dual-fisheye RGB frame
    int width = 0, height = 0;
    int64_t stamp_ns = 0;
    uint64_t seq = 0;              // bumps on every stored frame
    uint64_t last_read_seq = 0;    // single-reader convention (robocam read())

    std::atomic<uint64_t> frames = 0;
    std::atomic<uint64_t> decode_errors = 0;
    std::atomic<bool> open = false;
};

Delegate::~Delegate() {  // NOLINT: see class comment
    if (sws_) sws_freeContext(sws_);
    if (dec_) {
        dec_->Uninitialize();
        WelsDestroyDecoder(dec_);
    }
}

bool Delegate::InitDecoder() {
    if (WelsCreateDecoder(&dec_) != 0 || !dec_) return false;
    SDecodingParam dp;
    memset(&dp, 0, sizeof(dp));
    dp.sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_AVC;
    dp.eEcActiveIdc = ERROR_CON_DISABLE;  // USB-local stream: fail loud, resync at IDR
    return dec_->Initialize(&dp) == cmResultSuccess;
}

int64_t Delegate::Stamp(int64_t sdk_ts_ms) {
    if (sdk_ts_ms <= 0) return NowRealtimeNs();
    const int64_t device_ns = sdk_ts_ms * 1'000'000LL;
    if (prev_sdk_ms_ > 0 && sdk_ts_ms < prev_sdk_ms_) {  // stream restart
        anchor_offset_ns_ = 0;
        au_count_ = 0;
    }
    prev_sdk_ms_ = sdk_ts_ms;
    ++au_count_;
    if (au_count_ <= kAnchorWindow) {
        const int64_t offset = NowRealtimeNs() - device_ns;
        if (anchor_offset_ns_ == 0 || offset < anchor_offset_ns_) {
            anchor_offset_ns_ = offset;
        }
    }
    int64_t stamp = anchor_offset_ns_ + device_ns;
    if (stamp <= last_stamp_ns_) stamp = last_stamp_ns_ + 1;  // strictly monotonic
    last_stamp_ns_ = stamp;
    return stamp;
}

void Delegate::OnVideoData(const uint8_t* data, size_t size, int64_t timestamp,
                           uint8_t /*streamType*/, int stream_index) {
    if (stream_index != 0 || size == 0 || !dec_) return;
    const int64_t stamp_ns = Stamp(timestamp);

    uint8_t* yuv[3] = {nullptr, nullptr, nullptr};
    SBufferInfo info;
    memset(&info, 0, sizeof(info));
    info.uiInBsTimeStamp = static_cast<unsigned long long>(stamp_ns);

    const DECODING_STATE st =
        dec_->DecodeFrameNoDelay(data, static_cast<int>(size), yuv, &info);
    if (st != dsErrorFree) {
        // Stale mid-GOP tap-in at start + rare corrupted-AU bursts; the
        // decoder resyncs at the next IDR on its own.
        ++h_->decode_errors;
    }
    if (info.iBufferStatus != 1) return;

    const int w = info.UsrData.sSystemBuffer.iWidth;
    const int h = info.UsrData.sSystemBuffer.iHeight;
    if (w <= 0 || h <= 0) return;

    if (!sws_ || sws_w_ != w || sws_h_ != h) {
        if (sws_) sws_freeContext(sws_);
        sws_ = sws_getContext(w, h, AV_PIX_FMT_YUV420P, w, h, AV_PIX_FMT_RGB24,
                              SWS_POINT, nullptr, nullptr, nullptr);
        sws_w_ = w;
        sws_h_ = h;
        if (!sws_) return;
    }

    {
        std::lock_guard<std::mutex> lock(h_->mu);
        h_->rgb.resize(static_cast<size_t>(w) * h * 3);
        const uint8_t* src[3] = {yuv[0], yuv[1], yuv[2]};
        const int src_stride[3] = {info.UsrData.sSystemBuffer.iStride[0],
                                   info.UsrData.sSystemBuffer.iStride[1],
                                   info.UsrData.sSystemBuffer.iStride[1]};
        uint8_t* dst[1] = {h_->rgb.data()};
        const int dst_stride[1] = {w * 3};
        sws_scale(sws_, src, src_stride, 0, h, dst, dst_stride);
        h_->width = w;
        h_->height = h;
        // The decoded frame's OWN capture stamp, recovered from the pts it
        // carried through the codec (the 1-frame hold means it is NOT this
        // call's stamp).
        h_->stamp_ns = static_cast<int64_t>(info.uiOutYuvTimeStamp);
        ++h_->seq;
    }
    h_->cv.notify_all();
    ++h_->frames;
}

void SetErr(char* err, int errlen, const std::string& msg) {
    if (err && errlen > 0) {
        snprintf(err, static_cast<size_t>(errlen), "%s", msg.c_str());
    }
}

} // namespace

extern "C" {

// Open the camera and start streaming. Returns an opaque handle or nullptr
// (with `err` filled). serial="" selects the first discovered camera.
// A C ABI must never let a C++ exception escape (std::terminate); the body
// lives in ins_open_impl and is wrapped below.
// Process-wide fleet discovery, run ONCE. Re-running GetAvailableDevices()
// while another camera in this process is already streaming probes (and
// claims) every camera's USB interface, hitting LIBUSB_ERROR_BUSY and
// killing the live stream with LIBUSB_ERROR_IO - the same race multicam.cpp
// avoids with its single discovery call. Descriptors are kept alive for the
// process lifetime because DeviceConnectionInfo carries a raw
// native_connection_info pointer into descriptor-owned memory (freeing it
// before Open() made the SDK read freed memory -> garbage-size bad_alloc).
// Consequence: plug in every camera BEFORE the process's first ins_open.
struct Fleet {
    std::mutex mu;
    ins_camera::DeviceDiscovery discovery;
    std::vector<ins_camera::DeviceDescriptor> list;
    bool discovered = false;
};

Fleet& TheFleet() {
    static Fleet f;
    return f;
}

static void* ins_open_impl(const char* serial, const char* resolution, int bitrate,
                           int live_view_mode, int service_port, char* err, int errlen) {
    Fleet& fleet = TheFleet();
    // Held across Open()+StartLiveStreaming: serializes multi-camera bring-up.
    std::lock_guard<std::mutex> fleet_lock(fleet.mu);
    if (!fleet.discovered) {
        fleet.list = fleet.discovery.GetAvailableDevices();
        fleet.discovered = true;
    }
    if (fleet.list.empty()) {
        SetErr(err, errlen, "no camera discovered (check USB mode = Android, /dev/insta)");
        return nullptr;
    }
    int chosen = -1;
    std::string available;
    for (size_t i = 0; i < fleet.list.size(); ++i) {
        if (!available.empty()) available += ", ";
        available += fleet.list[i].serial_number;
        if (!serial || !*serial || fleet.list[i].serial_number == serial) {
            if (chosen < 0) chosen = static_cast<int>(i);
        }
    }
    if (chosen < 0) {
        SetErr(err, errlen,
               "serial not in the process's one-shot discovery; found: [" + available +
               "] (plug all cameras in before the first open of the process)");
        return nullptr;
    }

    auto handle = std::make_unique<Handle>();
    handle->cam = std::make_shared<ins_camera::Camera>(fleet.list[chosen].info);
    // Two cameras in ONE process need distinct SDK service ports set before
    // Open() (the multicam.cpp pattern); two PROCESSES cannot share a camera
    // fleet at all (the SDK binds the port per process: "Address already in
    // use"). 0 = leave the SDK default (single-camera case).
    if (service_port > 0) {
        handle->cam->SetServicePort(service_port);
    }
    if (!handle->cam->Open()) {
        SetErr(err, errlen, "camera Open() failed");
        return nullptr;
    }

    auto delegate = std::make_shared<Delegate>(handle.get());
    if (!delegate->InitDecoder()) {
        SetErr(err, errlen, "openh264 decoder init failed");
        handle->cam->Close();
        return nullptr;
    }
    handle->delegate = delegate;
    handle->cam->SetStreamDelegate(handle->delegate);

    const auto& map = ResolutionMap();
    auto it = map.find(resolution ? resolution : "");
    const ins_camera::VideoResolution res =
        (it != map.end()) ? it->second : ins_camera::VideoResolution::RES_1920_960P30;

    if (live_view_mode) {
        // SDK 2.1.1 live-view flow; makes the X5 honor the requested preview
        // resolution (verified; officially "fixed"). Non-fatal on failure.
        if (handle->cam->SetVideoSubMode(ins_camera::SubVideoMode::VIDEO_LIVEVIEW)) {
            ins_camera::RecordParams rp;
            rp.resolution = res;
            rp.bitrate = 0;
            handle->cam->SetVideoCaptureParams(
                rp, ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM);
        }
    }

    ins_camera::LiveStreamParam param;
    param.video_resolution = res;
    param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
    param.video_bitrate = static_cast<uint32_t>(bitrate);
    param.enable_audio = false;
    param.using_lrv = false;
    if (!handle->cam->StartLiveStreaming(param)) {
        SetErr(err, errlen, "StartLiveStreaming failed");
        handle->cam->Close();
        return nullptr;
    }
    handle->open = true;
    return handle.release();
}

void* ins_open(const char* serial, const char* resolution, int bitrate,
               int live_view_mode, int service_port, char* err, int errlen) {
    try {
        return ins_open_impl(serial, resolution, bitrate, live_view_mode,
                             service_port, err, errlen);
    } catch (const std::exception& e) {
        SetErr(err, errlen, std::string("exception: ") + e.what());
        return nullptr;
    } catch (...) {
        SetErr(err, errlen, "unknown exception in ins_open");
        return nullptr;
    }
}

// Blocking read of the newest frame not yet returned (latest-wins; never
// returns the same frame twice). Copies RGB into out_buf. lens: 0=full,
// 1=front (right half), 2=back (left half). Returns bytes written, 0 on
// timeout, -1 if out_buf too small (required size in *out_w/*out_h), -2 if
// closed. Single-reader per handle.
int ins_read(void* hv, uint8_t* out_buf, int out_capacity,
             int* out_w, int* out_h, int64_t* out_stamp_ns,
             int lens, double timeout_s) {
    auto* h = static_cast<Handle*>(hv);
    std::unique_lock<std::mutex> lock(h->mu);
    const bool got = h->cv.wait_for(
        lock, std::chrono::duration<double>(timeout_s),
        [&] { return h->seq > h->last_read_seq || !h->open; });
    if (!h->open) return -2;
    if (!got) return 0;

    const int w = (lens == 0) ? h->width : h->width / 2;
    const int x0 = (lens == 1) ? h->width / 2 : 0;  // front = right half
    const int need = w * h->height * 3;
    if (out_w) *out_w = w;
    if (out_h) *out_h = h->height;
    if (need > out_capacity) return -1;

    const int full_stride = h->width * 3;
    const uint8_t* src = h->rgb.data() + x0 * 3;
    if (lens == 0) {
        memcpy(out_buf, h->rgb.data(), static_cast<size_t>(need));
    } else {
        for (int r = 0; r < h->height; ++r) {
            memcpy(out_buf + static_cast<size_t>(r) * w * 3,
                   src + static_cast<size_t>(r) * full_stride,
                   static_cast<size_t>(w) * 3);
        }
    }
    if (out_stamp_ns) *out_stamp_ns = h->stamp_ns;
    h->last_read_seq = h->seq;
    return need;
}

uint64_t ins_frames(void* hv) { return static_cast<Handle*>(hv)->frames.load(); }
uint64_t ins_decode_errors(void* hv) { return static_cast<Handle*>(hv)->decode_errors.load(); }

// Stop streaming and release the camera (always Ctrl-C/clean-exit through
// this - a killed process leaves the X5 in LIBUSB_ERROR_BUSY until replug).
void ins_close(void* hv) {
    auto* h = static_cast<Handle*>(hv);
    if (!h) return;
    {
        std::lock_guard<std::mutex> lock(h->mu);
        h->open = false;
    }
    h->cv.notify_all();
    if (h->cam) {
        h->cam->StopLiveStreaming();
        h->cam->Close();
    }
    delete h;
}

} // extern "C"
