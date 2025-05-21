// ============================================================================
//  MicronTrackerWrapper.hpp   ―― 纯「位置 + 旋转矩阵」版本（不再使用四元数）
// ============================================================================
//
//  * 依赖：MicronTracker SDK v4.x   (MTC.h + Dist64MT4 运行时)
//  * 功能：
//      -  后台线程抓帧 + 识别模板
//      -  获取单个 marker 的位置 (mm) 与旋转矩阵 (行主序 3×3)
//      -  计算 markerB 在 markerA 坐标系下的：
//             - 相对位置   getRelPos()
//             - 相对旋转   getRelRotMat()
//
//  注：全部数学运算都基于旋转矩阵；完全去除了四元数相关代码。
// ============================================================================

#pragma once

#include <stdexcept>
#include <string>
#include "../../Dist64MT4/MTC.h"

#ifdef _WIN32
#   include <windows.h>
#endif

inline void mtCheck(mtCompletionCode cc, const char* fn)
{
    if (cc != mtOK)
        throw std::runtime_error(std::string("MTC error in ") + fn + ": " + MTLastErrorString());
}

#ifdef _WIN32
namespace {
    int getMTHome(char* buf, int size)
    {
        HKEY key;
        constexpr const char* kName = "MTHome";
        DWORD typ{}, len = size;
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE,
            "SYSTEM\\CurrentControlSet\\Control\\Session Manager\\Environment",
            0, KEY_QUERY_VALUE, &key) != ERROR_SUCCESS)
            return -1;

        auto err = RegQueryValueExA(key, kName, nullptr, &typ,
            reinterpret_cast<LPBYTE>(buf), &len);
        RegCloseKey(key);
        return (err == ERROR_SUCCESS && len > 1) ? 0 : -1;
    }
}
#endif

namespace mtw {

    struct Pose
    {
        double pos[3]{};
        double rot[9]{};
    };

    class MicronTracker
    {
    public:
        MicronTracker() = default;
        ~MicronTracker() { try { Cameras_Detach(); } catch (...) {} }
        double getFPS() const noexcept { return fps_; }



        void init(const std::string& mtHome = {})
        {
            if (initialised_) return;
            std::string base = mtHome;
            if (base.empty())
            {
#ifdef _WIN32
                char buf[512]{};
                if (getMTHome(buf, sizeof(buf)) != 0)
                    throw std::runtime_error("Cannot locate MTHome (registry)");
                base = buf;
#else
                const char* env = std::getenv("MTHome");
                if (!env) throw std::runtime_error("Cannot locate MTHome (env var)");
                base = env;
#endif
            }
            calibDir_ = base + "/CalibrationFiles";
            markerDir_ = base + "/Markers";

            mtCheck(Cameras_AttachAvailableCameras(calibDir_.c_str()),
                "Cameras_AttachAvailableCameras");
            if (Cameras_Count() < 1)
                throw std::runtime_error("No MicronTracker camera found");

            mtCheck(Cameras_ItemGet(0, &cam_), "Cameras_ItemGet");

            mtStreamingModeStruct mode{
                mtFrameType::Alternating,
                mtDecimation::Dec41,
                mtBitDepth::Bpp12
            };
            int serial{};
            mtCheck(Camera_SerialNumberGet(cam_, &serial), "Camera_SerialNumberGet");
            mtCheck(Cameras_StreamingModeSet(mode, serial), "Cameras_StreamingModeSet");
            //-------------------------------------------------
            // 4) PROI：中心 640×512
            //-------------------------------------------------

            //-------------------------------------------------
            // 5) 固定曝光 & 可选 HDR
            //-------------------------------------------------

            mtCheck(Camera_AutoExposureSet(cam_, 0), "AutoExposure off");
            mtCheck(Camera_ShutterMsecsSet(cam_, 3.0), "Shutter 3ms");
            mtCheck(Camera_GainFSet(cam_, 1.0), "Gain 1.0");


            mtCheck(Markers_LoadTemplates(const_cast<char*>(markerDir_.c_str())),
                "Markers_LoadTemplates");

            mtCheck(Markers_BackGroundProcessSet(true), "Markers_BackGroundProcessSet");

            identColl_ = Collection_New();
            xf_ = Xform3D_New();
            // (2) 把所有已加载模板的 Kalman 打开
            {
                mtHandle coll = Collection_New();
                mtCheck(Markers_TemplatesMarkersGet(coll), "TemplatesMarkersGet");

                const int n = Collection_Count(coll);
                for (int i = 1; i <= n; ++i)
                {
                    mtHandle mk = Collection_Int(coll, i);
                    Marker_KalmanNoiseFilterEnabledSet(mk, true);   // per-marker 打开
                    Marker_FilterNoiseCoeffSet(mk, cam_, 1);
                }
                Collection_Free(coll);
            }

            // (3) 可选：调 Jitter 参数（一次）
            Markers_JitterFilterCoefficientSet(0.8);       // 0.0-1.0，越大越平滑
            Markers_JitterFilterHistoryLengthSet(30);      // 历史帧长度
            
            initialised_ = true;



        }


        inline double getLastTimestamp() const { return lastTs_; }

        bool update()
        {
            if (!initialised_) return false;
            mtCheck(Markers_GetIdentifiedMarkersFromBackgroundThread(cam_),
                "Markers_GetIdentifiedMarkersFromBackgroundThread");
            // ---- 计算 FPS ----
            double ts{};
            mtCheck(Camera_FrameMTTimeSecsGet(cam_, &ts), "FrameTimeSecsGet");
            if (lastTs_ > 0.0)
            {
                const double dt = ts - lastTs_;
                if (dt > 1e-6) fps_ = 1.0 / dt;   // Hz
            }
            lastTs_ = ts;
            
            return true;
        }

        bool getPoseAndRotMat(const std::string& markerName, Pose& out)
        {
            if (!locateMarker(markerName)) return false;
            mtCheck(Xform3D_ShiftGet(xf_, out.pos), "Xform3D_ShiftGet");

            double R[9];
            mtCheck(Xform3D_RotMatGet(xf_, R), "Xform3D_RotMatGet");

            // 转置（从 row-major 转 col-major）
            out.rot[0] = R[0]; out.rot[1] = R[3]; out.rot[2] = R[6];
            out.rot[3] = R[1]; out.rot[4] = R[4]; out.rot[5] = R[7];
            out.rot[6] = R[2]; out.rot[7] = R[5]; out.rot[8] = R[8];
            return true;
        }

        bool getRelPos(const std::string& markerA, const std::string& markerB, double outPos[3])
        {
            Pose a, b;
            if (!getPoseAndRotMat(markerA, a) || !getPoseAndRotMat(markerB, b))
                return false;

            double dp[3]{  b.pos[0] - a.pos[0],  b.pos[1] - a.pos[1],  b.pos[2] - a.pos[2] };
            outPos[0] = a.rot[0] * dp[0] + a.rot[3] * dp[1] + a.rot[6] * dp[2];
            outPos[1] = a.rot[1] * dp[0] + a.rot[4] * dp[1] + a.rot[7] * dp[2];
            outPos[2] = a.rot[2] * dp[0] + a.rot[5] * dp[1] + a.rot[8] * dp[2];
            return true;
        }

        bool getRelRotMat(const std::string& markerA, const std::string& markerB, double outR[9])
        {
            Pose a, b;
            if (!getPoseAndRotMat(markerA, a) || !getPoseAndRotMat(markerB, b))
                return false;

            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                {
                    double v =
                        a.rot[0 * 3 + r] * b.rot[0 * 3 + c] +
                        a.rot[1 * 3 + r] * b.rot[1 * 3 + c] +
                        a.rot[2 * 3 + r] * b.rot[2 * 3 + c];
                    outR[c * 3 + r] = v;             // ★ 关键：写到 col-major 索引
                }
            return true;
        }

    private:
        double lastTs_{ 0.0 };   // 上一帧时间戳（秒）
        double fps_{ 0.0 };      // 计算出的实时 FPS


    private:
        bool locateMarker(const std::string& name)
        {
            mtCheck(Markers_IdentifiedMarkersGet(mtHandleNull, identColl_),
                "Markers_IdentifiedMarkersGet");
            const int n = Collection_Count(identColl_);
            for (int i = 1; i <= n; ++i)
            {
                mtHandle mk = Collection_Int(identColl_, i);
                char buf[MT_MAX_STRING_LENGTH]{};
                mtCheck(Marker_NameGet(mk, buf, sizeof(buf), nullptr), "Marker_NameGet");
                if (name == buf)
                {
                    mtHandle idCam{};
                    mtCheck(Marker_Marker2CameraXfGet(mk, cam_, xf_, &idCam), "Marker_Marker2CameraXfGet");
                    return idCam != 0;
                }
            }
            return false;
        }

    private:
        bool initialised_{ false };
        mtHandle cam_{ mtHandleNull };
        mtHandle identColl_{ mtHandleNull };
        mtHandle xf_{ mtHandleNull };
        std::string calibDir_, markerDir_;
    };

} // namespace mtw  