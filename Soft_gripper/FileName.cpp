#pragma once


#include <vector>
#include <array>
#include <iostream>
#include "../../Dist64MT4/MTC.h"
#include "string.h"
#include "stdlib.h"
#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
#endif

//Macro to check for and report MTC usage errors.
#define MTC(func) {int r = func; if (r!=mtOK) printf("MTC error: %s\n",MTLastErrorString()); };

#ifdef _WIN32
namespace {
    // ??????HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Environment\MTHome
    int getMTHome(char* buf, int bufSize)
    {
        LONG err;
        HKEY key;
        constexpr const char* kName = "MTHome";
        DWORD type = 0;
        DWORD len = bufSize;

        err = RegOpenKeyExA(HKEY_LOCAL_MACHINE,
            "SYSTEM\\CurrentControlSet\\Control\\Session Manager\\Environment",
            0, KEY_QUERY_VALUE, &key);
        if (err != ERROR_SUCCESS) return -1;

        err = RegQueryValueExA(key, kName, 0, &type,
            reinterpret_cast<LPBYTE>(buf), &len);
        RegCloseKey(key);
        return (err == ERROR_SUCCESS && len > 1) ? 0 : -1;
    }
}
#endif


#include <unordered_map>
#include <stdexcept>
#include <cstring>

namespace mtw {

    struct Pose
    {
        double pos[3];   // ?? (mm)
        double quat[4];  // ??? (w,x,y,z)
    };


    // ???MicronTracker ???? (x,y,z,w)????? (w,x,y,z)
    inline void toWxyz(double q[4])
    {
        std::swap(q[0], q[3]);          // (x,y,z,w) ? (w,y,z,x)
        std::swap(q[1], q[3]);          // (w,x,z,y)
        std::swap(q[2], q[3]);          // (w,x,y,z)
    }


    // Helper that throws std::runtime_error on non?mtOK return
    inline void check(mtCompletionCode cc, const char* fn)
    {
        if (cc != mtOK)
            throw std::runtime_error(std::string("MTC error in ") + fn + ": " + MTLastErrorString());
    }

    class MicronTracker
    {
    public:
        MicronTracker() = default;
        ~MicronTracker()
        {
            try { Cameras_Detach(); }
            catch (...) {}
        }

        // -------------------------------------------------------------------
        // Initialise – will attach first available camera and load templates
        // -------------------------------------------------------------------
        void init(const std::string& mtHome = {})
        {
            if (initialised_) return;

            // Resolve directories (CalibrationFiles / Markers)
            std::string base = mtHome;
            if (base.empty())
            {
#ifdef _WIN32
                char buf[512]{};
                if (getMTHome(buf, sizeof(buf)) < 0)
                    throw std::runtime_error("MTHome env var not set in registry");
                base = buf;
#else
                const char* env = std::getenv("MTHome");
                if (!env) throw std::runtime_error("MTHome env var not set");
                base = env;
#endif
            }
            calibDir_ = base + "/CalibrationFiles";
            markerDir_ = base + "/Markers";

            // Attach cameras
            check(Cameras_AttachAvailableCameras(calibDir_.c_str()), "Cameras_AttachAvailableCameras");
            if (Cameras_Count() < 1)
                throw std::runtime_error("No MicronTracker camera found");

            // first camera handle
            check(Cameras_ItemGet(0, &currCam_), "Cameras_ItemGet");

            // put camera in alternating/Dec41/14?bit (fast default)
            mtStreamingModeStruct mode{ mtFrameType::Alternating, mtDecimation::Dec41, mtBitDepth::Bpp14 };
            int serial{};  check(Camera_SerialNumberGet(currCam_, &serial), "Camera_SerialNumberGet");
            check(Cameras_StreamingModeSet(mode, serial), "Cameras_StreamingModeSet");

            // Load templates
            check(Markers_LoadTemplates(const_cast<char*>(markerDir_.c_str())), "Markers_LoadTemplates");

            identifiedColl_ = Collection_New();
            poseXf_ = Xform3D_New();

            initialised_ = true;
        }

        // Grab + process a single frame (all cameras)
        bool update(bool background = false)
        {
            if (!initialised_) return false;

            if (background)
            {
                check(Markers_GetIdentifiedMarkersFromBackgroundThread(currCam_), "Markers_GetIdentifiedMarkersFromBackgroundThread");
            }
            else
            {
                check(Cameras_GrabFrame(mtHandleNull), "Cameras_GrabFrame");
                check(Markers_ProcessFrame(mtHandleNull), "Markers_ProcessFrame");
            }
            return true;
        }

        /*-----------------------------------------------------------
         * 1) ??? Marker ? “???????” ?? Pose
         *----------------------------------------------------------*/
        bool getPose(mtHandle marker,          // Marker ??
            mtHandle camera,          // “??” Camera ??
            Pose& out)
        {
            bool wasId = false;
            if (Marker_WasIdentifiedGet(marker, camera, &wasId) != mtOK || !wasId)
                return false;                             // ?????????

            mtHandle xf = Xform3D_New();
            mtHandle identifyingCam = 0;
            if (Marker_Marker2CameraXfGet(marker, camera, xf, &identifyingCam) != mtOK ||
                identifyingCam == 0)
            {
                Xform3D_Free(xf);
                return false;                             // ??????????
            }

            Xform3D_ShiftGet(xf, out.pos);                // mm
            Xform3D_RotQuaternionsGet(xf, out.quat);      // (x,y,z,w)
            toWxyz(out.quat);

            Xform3D_Free(xf);
            return true;
        }

        /*-----------------------------------------------------------
         * 2) ? “target ? reference ?????????”
         *    ???   T_rel =  T_ref?¹  ·  T_tar
         *----------------------------------------------------------*/
        bool getRelativePose(mtHandle refMarker, mtHandle tarMarker,
            mtHandle camera, Pose& out)
        {
            // ---------- ????? Marker?Camera ?? ----------
            mtHandle ref2cam = Xform3D_New(), tar2cam = Xform3D_New();
            mtHandle tmpCam = 0;

            if (Marker_Marker2CameraXfGet(refMarker, camera, ref2cam, &tmpCam) != mtOK || tmpCam == 0 ||
                Marker_Marker2CameraXfGet(tarMarker, camera, tar2cam, &tmpCam) != mtOK || tmpCam == 0)
            {
                Xform3D_Free(ref2cam);  Xform3D_Free(tar2cam);
                return false;           // ????????????
            }

            // ---------- T_rel = ref?¹ * tar ----------
            mtHandle refInv = Xform3D_New();
            mtHandle relXf = Xform3D_New();
            Xform3D_Inverse(ref2cam, refInv);             // ref?¹
            Xform3D_Concatenate(refInv, tar2cam, relXf);  // ref?¹·tar

            // ---------- ??? ----------
            Xform3D_ShiftGet(relXf, out.pos);
            Xform3D_RotQuaternionsGet(relXf, out.quat);   // (x,y,z,w)
            toWxyz(out.quat);

            // ---------- ?? ----------
            Xform3D_Free(ref2cam);  Xform3D_Free(tar2cam);
            Xform3D_Free(refInv);   Xform3D_Free(relXf);
            return true;
        }





#ifdef _WIN32
        static int getMTHome(char* buf, int size)
        {
            DWORD len = GetEnvironmentVariableA("MTHome", buf, size);
            return (len && len < size) ? 0 : -1;
        }
#endif

    private:
        bool initialised_ = false;
        mtHandle currCam_ = mtHandleNull;
        mtHandle identifiedColl_ = mtHandleNull;
        mtHandle poseXf_ = mtHandleNull;

        std::string calibDir_;
        std::string markerDir_;
    };

} // namespace mtw