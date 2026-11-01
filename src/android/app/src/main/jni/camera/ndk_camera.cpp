// Copyright 2020-2024 Citra Emulator Project / Azahar Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the misc/licenses/gplv2.txt file included.

#include <algorithm>
#include <mutex>
#include <camera/NdkCameraCaptureSession.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <camera/NdkCaptureRequest.h>
#include <libyuv.h>
#include <media/NdkImageReader.h>
#include "common/scope_exit.h"
#include "common/thread.h"
#include "core/frontend/camera/blank_camera.h"
#include "jni/camera/ndk_camera.h"
#include "jni/camera/still_image_camera.h"
#include "jni/id_cache.h"

namespace Camera::NDK {

/**
 * Implementation detail of NDK camera interface, holding a ton of different structs.
 * As long as the object lives, the camera is opened and capturing image. To turn off the camera,
 * one needs to destruct the object.
 * The pixel format is 'Android 420' which can contain a variety of YUV420 formats. The exact
 * format used can be determined by examing the 'pixel stride'.
 */
struct CaptureSession final {
    explicit CaptureSession(ACameraManager* manager, const std::string& id) {
        Load(manager, id);
    }

    void Load(ACameraManager* manager, const std::string& id);

    std::pair<int, int> selected_resolution{};

    ACameraDevice_StateCallbacks device_callbacks{};
    AImageReader_ImageListener listener{};
    ACameraCaptureSession_stateCallbacks session_callbacks{};
    std::array<ACaptureRequest*, 1> requests{};

#define MEMBER(type, name, func)                                                                   \
    struct type##Deleter {                                                                         \
        void operator()(type* ptr) {                                                               \
            type##_##func(ptr);                                                                    \
        }                                                                                          \
    };                                                                                             \
    std::unique_ptr<type, type##Deleter> name

    MEMBER(ACameraDevice, device, close);
    MEMBER(AImageReader, image_reader, delete);
    MEMBER(ANativeWindow, native_window, release);

    MEMBER(ACaptureSessionOutputContainer, outputs, free);
    MEMBER(ACaptureSessionOutput, output, free);
    MEMBER(ACameraOutputTarget, target, free);
    MEMBER(ACaptureRequest, request, free);

    // Put session last to close the session before we destruct everything else
    MEMBER(ACameraCaptureSession, session, close);
#undef MEMBER

    bool ready = false;

    std::mutex data_mutex;

    // Clang does not yet have shared_ptr to arrays support. Managed data are actually arrays.
    std::array<std::shared_ptr<u8>, 3> data{}; // I420 format, planes are Y, U, V.
    std::array<int, 3> row_stride{};           // Row stride for the planes.
    int pixel_stride{};                        // Pixel stride for the UV planes.
    Common::Event active_event;                // Signals that the session has become ready

    int sensor_orientation{}; // Sensor Orientation
    bool facing_front{};      // Whether this camera faces front. Used for handling device rotation.

    std::mutex status_mutex;
    bool disconnected{}; // Whether this device has been closed and should be reopened
    bool reload_requested{};
};

void OnDisconnected(void* context, ACameraDevice* device) {
    LOG_WARNING(Service_CAM, "Camera device disconnected");

    CaptureSession* that = reinterpret_cast<CaptureSession*>(context);
    {
        std::lock_guard lock{that->status_mutex};
        that->disconnected = true;
    }
}

static void OnError(void* context, ACameraDevice* device, int error) {
    LOG_ERROR(Service_CAM, "Camera device error {}", error);
}

#define MEDIA_CALL(func)                                                                           \
    {                                                                                              \
        auto ret = func;                                                                           \
        if (ret != AMEDIA_OK) {                                                                    \
            LOG_ERROR(Service_CAM, "Call " #func " returned error {}", ret);                       \
            return;                                                                                \
        }                                                                                          \
    }

#define CAMERA_CALL(func)                                                                          \
    {                                                                                              \
        auto ret = func;                                                                           \
        if (ret != ACAMERA_OK) {                                                                   \
            LOG_ERROR(Service_CAM, "Call " #func " returned error {}", ret);                       \
            return;                                                                                \
        }                                                                                          \
    }

void ImageCallback(void* context, AImageReader* reader) {
    AImage* image{};
    MEDIA_CALL(AImageReader_acquireLatestImage(reader, &image));
    SCOPE_EXIT({ AImage_delete(image); });

    std::array<std::shared_ptr<u8>, 3> data;
    std::array<int, 3> row_stride;
    for (const int plane : {0, 1, 2}) {
        u8* ptr;
        int size;
        MEDIA_CALL(AImage_getPlaneData(image, plane, &ptr, &size));
        data[plane].reset(new u8[size], std::default_delete<u8[]>());
        std::memcpy(data[plane].get(), ptr, static_cast<std::size_t>(size));

        MEDIA_CALL(AImage_getPlaneRowStride(image, plane, &row_stride[plane]));
    }

    CaptureSession* that = reinterpret_cast<CaptureSession*>(context);
    {
        std::lock_guard lock{that->data_mutex};
        that->data = data;
        that->row_stride = row_stride;
        MEDIA_CALL(AImage_getPlanePixelStride(image, 1, &that->pixel_stride));
    }
    {
        std::lock_guard lock{that->status_mutex};
        if (!that->ready) {
            that->active_event.Set(); // Mark the session as active
        }
    }
}

#define CREATE(type, name, statement)                                                              \
    {                                                                                              \
        type* raw;                                                                                 \
        statement;                                                                                 \
        name.reset(raw);                                                                           \
    }

// We have to define these no-op callbacks
static void OnClosed(void* context, ACameraCaptureSession* session) {}
static void OnReady(void* context, ACameraCaptureSession* session) {}
static void OnActive(void* context, ACameraCaptureSession* session) {}

void CaptureSession::Load(ACameraManager* manager, const std::string& id) {
    {
        std::lock_guard lock{status_mutex};
        ready = disconnected = reload_requested = false;
    }

    device_callbacks = {
        /*context*/ this,
        /*onDisconnected*/ &OnDisconnected,
        /*onError*/ &OnError,
    };

    CREATE(ACameraDevice, device,
           CAMERA_CALL(ACameraManager_openCamera(manager, id.c_str(), &device_callbacks, &raw)));

    ACameraMetadata* metadata;
    CAMERA_CALL(ACameraManager_getCameraCharacteristics(manager, id.c_str(), &metadata));

    ACameraMetadata_const_entry entry;
    CAMERA_CALL(ACameraMetadata_getConstEntry(
        metadata, ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry));

    // We select the minimum resolution larger than 640x640 if any, or the maximum resolution.
    selected_resolution = {};
    for (std::size_t i = 0; i < entry.count; i += 4) {
        // (format, width, height, input?)
        if (entry.data.i32[i + 3] & ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_INPUT) {
            // This is an input stream
            continue;
        }

        int format = entry.data.i32[i + 0];
        if (format == AIMAGE_FORMAT_YUV_420_888) {
            int width = entry.data.i32[i + 1];
            int height = entry.data.i32[i + 2];
            if (selected_resolution.first <= 640 || selected_resolution.second <= 640) {
                // Selected resolution is not large enough
                selected_resolution = std::max(selected_resolution, std::make_pair(width, height));
            } else if (width >= 640 && height >= 640) {
                // Selected resolution and this one are both large enough
                selected_resolution = std::min(selected_resolution, std::make_pair(width, height));
            }
        }
    }

    CAMERA_CALL(ACameraMetadata_getConstEntry(metadata, ACAMERA_SENSOR_ORIENTATION, &entry));
    sensor_orientation = entry.data.i32[0];

    CAMERA_CALL(ACameraMetadata_getConstEntry(metadata, ACAMERA_LENS_FACING, &entry));
    if (entry.data.i32[0] == ACAMERA_LENS_FACING_FRONT) {
        facing_front = true;
    }
    ACameraMetadata_free(metadata);

    if (selected_resolution == std::pair<int, int>{}) {
        LOG_ERROR(Service_CAM, "Device does not support any YUV output format");
        return;
    }

    CREATE(AImageReader, image_reader,
           MEDIA_CALL(AImageReader_new(selected_resolution.first, selected_resolution.second,
                                       AIMAGE_FORMAT_YUV_420_888, 4, &raw)));

    listener = {
        /*context*/ this,
        /*onImageAvailable*/ &ImageCallback,
    };
    MEDIA_CALL(AImageReader_setImageListener(image_reader.get(), &listener));

    CREATE(ANativeWindow, native_window,
           MEDIA_CALL(AImageReader_getWindow(image_reader.get(), &raw)));
    ANativeWindow_acquire(native_window.get());

    CREATE(ACaptureSessionOutput, output,
           CAMERA_CALL(ACaptureSessionOutput_create(native_window.get(), &raw)));

    CREATE(ACaptureSessionOutputContainer, outputs,
           CAMERA_CALL(ACaptureSessionOutputContainer_create(&raw)));
    CAMERA_CALL(ACaptureSessionOutputContainer_add(outputs.get(), output.get()));

    session_callbacks = {
        /*context*/ nullptr,
        /*onClosed*/ &OnClosed,
        /*onReady*/ &OnReady,
        /*onActive*/ &OnActive,
    };
    CREATE(ACameraCaptureSession, session,
           CAMERA_CALL(ACameraDevice_createCaptureSession(device.get(), outputs.get(),
                                                          &session_callbacks, &raw)));
    CREATE(ACaptureRequest, request,
           CAMERA_CALL(ACameraDevice_createCaptureRequest(device.get(), TEMPLATE_PREVIEW, &raw)));

    CREATE(ACameraOutputTarget, target,
           CAMERA_CALL(ACameraOutputTarget_create(native_window.get(), &raw)));
    CAMERA_CALL(ACaptureRequest_addTarget(request.get(), target.get()));

    requests = {request.get()};
    CAMERA_CALL(ACameraCaptureSession_setRepeatingRequest(session.get(), nullptr, 1,
                                                          requests.data(), nullptr));

    // Wait until the first image comes
    active_event.Wait();
    {
        std::lock_guard lock{status_mutex};
        ready = true;
    }
}

#undef MEDIA_CALL
#undef CAMERA_CALL
#undef CREATE

Interface::Interface(Factory& factory_, const std::string& id_, const Service::CAM::Flip& flip)
    : factory(factory_), id(id_) {
    mirror = base_mirror =
        flip == Service::CAM::Flip::Horizontal || flip == Service::CAM::Flip::Reverse;
    invert = base_invert =
        flip == Service::CAM::Flip::Vertical || flip == Service::CAM::Flip::Reverse;
}

Interface::~Interface() {
    factory.camera_permission_requested = false;
}

void Interface::StartCapture() {
    session = factory.CreateCaptureSession(id);
}

void Interface::StopCapture() {
    session.reset();
}

void Interface::SetResolution(const Service::CAM::Resolution& resolution_) {
    resolution = resolution_;
}

void Interface::SetFlip(Service::CAM::Flip flip) {
    mirror = base_mirror ^
             (flip == Service::CAM::Flip::Horizontal || flip == Service::CAM::Flip::Reverse);
    invert =
        base_invert ^ (flip == Service::CAM::Flip::Vertical || flip == Service::CAM::Flip::Reverse);
}

void Interface::SetFormat(Service::CAM::OutputFormat format_) {
    format = format_;
}

struct YUVImage {
    int width{};
    int height{};
    std::vector<u8> y;
    std::vector<u8> u;
    std::vector<u8> v;

    explicit YUVImage(int width_, int height_)
        : width(width_), height(height_), y(static_cast<std::size_t>(width * height)),
          u(static_cast<std::size_t>(width * height / 4)),
          v(static_cast<std::size_t>(width * height / 4)) {}

    void Swap(YUVImage& other) {
        y.swap(other.y);
        u.swap(other.u);
        v.swap(other.v);
        std::swap(width, other.width);
        std::swap(height, other.height);
    }

    void Clear() {
        y.clear();
        u.clear();
        v.clear();
        width = height = 0;
    }
};

#define YUV(image)                                                                                 \
    image.y.data(), image.width, image.u.data(), image.width / 2, image.v.data(), image.width / 2

struct CropRect {
    int x{};
    int y{};
    int width{};
    int height{};
};

// Largest centered area of the device frame that has the aspect ratio of the requested output
// image. This is the historical behaviour, used whenever the game captures the whole sensor.
static CropRect OutputAspectCrop(int frame_width, int frame_height, int out_width, int out_height) {
    CropRect crop;
    if (out_width <= 0 || out_height <= 0) {
        crop.width = frame_width;
        crop.height = frame_height;
        return crop;
    }
    if (out_width * frame_height > out_height * frame_width) {
        crop.width = frame_width;
        crop.height = frame_width * out_height / out_width;
    } else {
        crop.height = frame_height;
        crop.width = frame_height * out_width / out_height;
    }
    crop.x = (frame_width - crop.width) / 2;
    crop.y = (frame_height - crop.height) / 2;
    return crop;
}

// Games can capture a sub-rectangle of the 640x480 sensor (CAM::SetDetailSize) which the hardware
// then scales to the output size, e.g. to zoom in on a QR code. Emulate this by treating the
// centered 4:3 area of the device frame as the sensor and cutting the requested rectangle out of
// it. The crop coordinates are inclusive.
static CropRect SensorCrop(int frame_width, int frame_height,
                           const Service::CAM::Resolution& resolution) {
    constexpr int SensorWidth = 640;
    constexpr int SensorHeight = 480;

    const int crop_x0 = resolution.crop_x0;
    const int crop_y0 = resolution.crop_y0;
    const int crop_x1 = resolution.crop_x1;
    const int crop_y1 = resolution.crop_y1;

    const bool is_full_sensor =
        crop_x0 == 0 && crop_y0 == 0 && crop_x1 == SensorWidth - 1 && crop_y1 == SensorHeight - 1;
    const bool is_valid =
        crop_x0 < crop_x1 && crop_y0 < crop_y1 && crop_x1 < SensorWidth && crop_y1 < SensorHeight;
    if (is_full_sensor || !is_valid) {
        return OutputAspectCrop(frame_width, frame_height, resolution.width, resolution.height);
    }

    // Centered 4:3 area of the frame stands in for the sensor
    int sensor_width{};
    int sensor_height{};
    if (frame_width * SensorHeight > frame_height * SensorWidth) {
        sensor_height = frame_height;
        sensor_width = frame_height * SensorWidth / SensorHeight;
    } else {
        sensor_width = frame_width;
        sensor_height = frame_width * SensorHeight / SensorWidth;
    }
    const int sensor_x = (frame_width - sensor_width) / 2;
    const int sensor_y = (frame_height - sensor_height) / 2;

    CropRect crop;
    crop.x = sensor_x + crop_x0 * sensor_width / SensorWidth;
    crop.y = sensor_y + crop_y0 * sensor_height / SensorHeight;
    crop.width = (crop_x1 - crop_x0 + 1) * sensor_width / SensorWidth;
    crop.height = (crop_y1 - crop_y0 + 1) * sensor_height / SensorHeight;

    // I420 chroma planes are subsampled 2x2: keep the rectangle on even coordinates and in bounds
    crop.x &= ~1;
    crop.y &= ~1;
    crop.width = std::min(crop.width, frame_width - crop.x) & ~1;
    crop.height = std::min(crop.height, frame_height - crop.y) & ~1;
    if (crop.width < 2 || crop.height < 2) {
        return OutputAspectCrop(frame_width, frame_height, resolution.width, resolution.height);
    }
    return crop;
}

std::vector<u16> Interface::ReceiveFrame() {
    bool should_reload = false;
    {
        std::lock_guard lock{session->status_mutex};
        if (session->reload_requested) {
            session->reload_requested = false;
            should_reload = session->disconnected;
        }
    }
    if (should_reload) {
        LOG_INFO(Service_CAM, "Reloading camera session");
        session->Load(factory.manager.get(), id);
    }

    bool session_ready;
    {
        std::lock_guard lock{session->status_mutex};
        session_ready = session->ready;
    }
    if (!session_ready) { // Camera was not opened
        return std::vector<u16>(resolution.width * resolution.height);
    }

    std::array<std::shared_ptr<u8>, 3> data;
    std::array<int, 3> row_stride;
    {
        std::lock_guard lock{session->data_mutex};
        data = session->data;
        row_stride = session->row_stride;
    }

    ASSERT_MSG(data[0] && data[1] && data[2], "Data is not available");

    auto [width, height] = session->selected_resolution;

    YUVImage converted(width, height);
    libyuv::Android420ToI420(data[0].get(), row_stride[0], data[1].get(), row_stride[1],
                             data[2].get(), row_stride[2], session->pixel_stride, YUV(converted),
                             width, height);

    // Rotate the image to get it in upright position
    // The g_rotation here is the display rotation which is opposite of the device rotation
    const int rotation =
        (session->sensor_orientation + (session->facing_front ? g_rotation : 4 - g_rotation) * 90) %
        360;
    if (rotation == 90 || rotation == 270) {
        std::swap(width, height);
    }
    YUVImage rotated(width, height);
    libyuv::I420Rotate(YUV(converted), YUV(rotated), converted.width, converted.height,
                       static_cast<libyuv::RotationMode>(rotation));
    converted.Clear();

    // Calculate crop coordinates (honours the sensor region requested by the game, if any)
    const CropRect crop = SensorCrop(width, height, resolution);

    const int y_offset = crop.y * width + crop.x;
    const int uv_offset = crop.y / 2 * width / 2 + crop.x / 2;
    YUVImage scaled(resolution.width, resolution.height);
    // Crop and scale
    libyuv::I420Scale(rotated.y.data() + y_offset, width, rotated.u.data() + uv_offset, width / 2,
                      rotated.v.data() + uv_offset, width / 2, crop.width, crop.height, YUV(scaled),
                      resolution.width, resolution.height, libyuv::kFilterBilinear);
    rotated.Clear();

    if (mirror) {
        YUVImage mirrored(scaled.width, scaled.height);
        libyuv::I420Mirror(YUV(scaled), YUV(mirrored), resolution.width, resolution.height);
        scaled.Swap(mirrored);
    }

    std::vector<u16> output(resolution.width * resolution.height);
    if (format == Service::CAM::OutputFormat::RGB565) {
        libyuv::I420ToRGB565(YUV(scaled), reinterpret_cast<u8*>(output.data()),
                             resolution.width * 2, resolution.width,
                             invert ? -resolution.height : resolution.height);
    } else {
        libyuv::I420ToYUY2(YUV(scaled), reinterpret_cast<u8*>(output.data()), resolution.width * 2,
                           resolution.width, invert ? -resolution.height : resolution.height);
    }
    return output;
}

#undef YUV

bool Interface::IsPreviewAvailable() {
    if (!session) {
        return false;
    }
    std::lock_guard lock{session->status_mutex};
    return session->ready;
}

Factory::Factory() = default;

Factory::~Factory() = default;

std::shared_ptr<CaptureSession> Factory::CreateCaptureSession(const std::string& id) {
    if (opened_camera_map.count(id) && !opened_camera_map.at(id).expired()) {
        return opened_camera_map.at(id).lock();
    }
    const auto& session = std::make_shared<CaptureSession>(manager.get(), id);
    opened_camera_map.insert_or_assign(id, session);
    return session;
}

std::unique_ptr<CameraInterface> Factory::Create(const std::string& config,
                                                 const Service::CAM::Flip& flip) {

    manager.reset(ACameraManager_create());
    ACameraIdList* id_list = nullptr;

    auto ret = ACameraManager_getCameraIdList(manager.get(), &id_list);
    if (ret != ACAMERA_OK) {
        LOG_ERROR(Service_CAM, "Failed to get camera ID list: ret {}", ret);
        return std::make_unique<Camera::BlankCamera>();
    }

    SCOPE_EXIT({ ACameraManager_deleteCameraIdList(id_list); });

    if (id_list->numCameras <= 0) {
        LOG_WARNING(Service_CAM, "No camera devices found, falling back to StillImage");
        // TODO: A better way of doing this?
        return std::make_unique<StillImage::Factory>()->Create("", flip);
    }

    // Request camera permission
    if (!camera_permission_granted) {
        if (camera_permission_requested) { // Permissions already denied
            return std::make_unique<Camera::BlankCamera>();
        }
        camera_permission_requested = true;

        JNIEnv* env = IDCache::GetEnvForThread();
        jboolean result = env->CallStaticBooleanMethod(IDCache::GetNativeLibraryClass(),
                                                       IDCache::GetRequestCameraPermission());
        if (result != JNI_TRUE) {
            LOG_ERROR(Service_CAM, "Camera permissions denied");
            return std::make_unique<Camera::BlankCamera>();
        }
        camera_permission_granted = true;
    }

    if (config.empty()) {
        LOG_WARNING(Service_CAM, "Camera ID not set, using default camera");
        return std::make_unique<Interface>(*this, id_list->cameraIds[0], flip);
    }

    for (int i = 0; i < id_list->numCameras; ++i) {
        const char* id = id_list->cameraIds[i];
        if (config == id) {
            return std::make_unique<Interface>(*this, id, flip);
        }

        if (config != FrontCameraPlaceholder && config != BackCameraPlaceholder) {
            continue;
        }

        ACameraMetadata* metadata;
        ACameraManager_getCameraCharacteristics(manager.get(), id, &metadata);
        SCOPE_EXIT({ ACameraMetadata_free(metadata); });

        ACameraMetadata_const_entry entry;
        ACameraMetadata_getConstEntry(metadata, ACAMERA_LENS_FACING, &entry);
        if ((entry.data.i32[0] == ACAMERA_LENS_FACING_FRONT && config == FrontCameraPlaceholder) ||
            (entry.data.i32[0] == ACAMERA_LENS_FACING_BACK && config == BackCameraPlaceholder)) {
            return std::make_unique<Interface>(*this, id, flip);
        }
    }

    LOG_ERROR(Service_CAM, "Camera ID {} not found", config);
    return std::make_unique<Camera::BlankCamera>();
}

void Factory::ReloadCameraDevices() {
    for (const auto& [id, ptr] : opened_camera_map) {
        if (auto session = ptr.lock()) {
            std::lock_guard lock{session->status_mutex};
            session->reload_requested = true;
        }
    }
}

} // namespace Camera::NDK
