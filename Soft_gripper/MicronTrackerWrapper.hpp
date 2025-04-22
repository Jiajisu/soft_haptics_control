
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

//#ifdef WIN32
//int getMTHome(char* sMTHome, int size); //Forward declaration
//#endif


#include <unordered_map>
#include <stdexcept>
#include <cstring>

namespace mtw {

    struct Pose
    {
        double pos[3]{};   // millimetres (MicronTracker default)
        double quat[4]{};  // (w,x,y,z) from Xform3D
    };

    // Helper that throws std::runtime_error on non‑mtOK return
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
                if (getenv_s(nullptr, buf, sizeof(buf), "MTHome") != 0)
                    throw std::runtime_error("MTHome env var not set");
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

            // put camera in alternating/Dec41/14‑bit (fast default)
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

        // retrieve pose of a template marker by name (in current‑camera coords)
        bool getPose(const std::string& markerName, Pose& out)
        {
            if (!initialised_) return false;
            check(Markers_IdentifiedMarkersGet(mtHandleNull, identifiedColl_), "Markers_IdentifiedMarkersGet");

            int n = Collection_Count(identifiedColl_);
            for (int i = 1; i <= n; ++i)
            {
                mtHandle mk = Collection_Int(identifiedColl_, i);
                char name[MT_MAX_STRING_LENGTH]{};
                check(Marker_NameGet(mk, name, sizeof(name), mtHandleNull), "Marker_NameGet");
                if (markerName == name)
                {
                    mtHandle identifyingCam{};
                    check(Marker_Marker2CameraXfGet(mk, currCam_, poseXf_, &identifyingCam), "Marker_Marker2CameraXfGet");
                    if (identifyingCam == 0) return false; // not in current camera frame

                    check(Xform3D_ShiftGet(poseXf_, out.pos), "Xform3D_ShiftGet");
                    check(Xform3D_RotQuaternionsGet(poseXf_, out.quat), "Xform3D_RotQuaternionsGet");
                    // MicronTracker returns (x,y,z,w); convert to (w,x,y,z)
                    std::swap(out.quat[0], out.quat[3]);
                    return true;
                }
            }
            return false; // not found
        }

        // compute pose of marker2 expressed in marker1 frame
        bool getRelativePose(const std::string& marker1, const std::string& marker2, Pose& out)
        {
            Pose p1, p2;
            if (!getPose(marker1, p1) || !getPose(marker2, p2)) return false;

            // Build Xform3D for each, then concatenate inverse
            mtHandle xf1 = Xform3D_New();
            mtHandle xf2 = Xform3D_New();
            mtHandle rel = Xform3D_New();

            Xform3D_ShiftSet(xf1, p1.pos);
            double q1[4] = { p1.quat[1], p1.quat[2], p1.quat[3], p1.quat[0] }; // back to (x,y,z,w)
            Xform3D_RotQuaternionsSet(xf1, q1);

            Xform3D_ShiftSet(xf2, p2.pos);
            double q2[4] = { p2.quat[1], p2.quat[2], p2.quat[3], p2.quat[0] };
            Xform3D_RotQuaternionsSet(xf2, q2);

            Xform3D_Inverse(xf1, rel);
            Xform3D_Concatenate(rel, xf2, rel);

            check(Xform3D_ShiftGet(rel, out.pos), "Xform3D_ShiftGet rel");
            double qRel[4];  check(Xform3D_RotQuaternionsGet(rel, qRel), "Xform3D_RotQuaternionsGet rel");
            out.quat[0] = qRel[3]; out.quat[1] = qRel[0]; out.quat[2] = qRel[1]; out.quat[3] = qRel[2];

            Xform3D_Free(xf1); Xform3D_Free(xf2); Xform3D_Free(rel);
            return true;
        }

    private:
        bool initialised_ = false;
        mtHandle currCam_ = mtHandleNull;
        mtHandle identifiedColl_ = mtHandleNull;
        mtHandle poseXf_ = mtHandleNull;

        std::string calibDir_;
        std::string markerDir_;
    };

} // namespace mtw
