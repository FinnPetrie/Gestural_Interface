//------------------------------------------------------------------------------
// <copyright file="BodyBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once
#include <Windows.h>
#include "resource.h"
#define GLM_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/glm.hpp>
#include <ctime>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "GraphicsApplication.h"
#include "Plyfile.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>



struct PointingInfo {
    Eigen::Vector3f *previousDirection = nullptr;
    Eigen::Vector3f *startingPoints;
    bool firstPointer = false;
    bool calibrated = false;
    bool screenPlaneFound = false;
    clock_t beginTime;
    uint32_t index;
    std::vector<std::vector<Eigen::Vector3f>> points;
    std::vector<std::vector<Eigen::Vector3f>> directions;
    std::vector<std::vector<Eigen::Vector3f>> secondaryPoints;

};


class CBodyBasics
{
    static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;

public:
    /// <summary>
    /// Constructor
    /// </summary>
    CBodyBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CBodyBasics();

    bool isPointing(Eigen::Vector3f current, uint32_t index);
    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
    Eigen::Hyperplane<float, 3> screenPlane;
    GraphicsApplication* app;
    HWND                    m_hWnd;
    INT64                   m_nStartTime;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    INT64                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;
    std::vector<PointingInfo> pointingInfo;
    std::vector<std::vector<Eigen::Vector3f>> cornersForFrames;
    Eigen::Vector3f* previous = nullptr;
    Eigen::Vector3f centroid;
    bool firstPoint = false;
    bool savePlane = false;
    Eigen::Vector3f* startingPoint = nullptr;
    INT64 startTime = 0;
    clock_t beginTime;
    // Current Kinect
    IKinectSensor*          m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper;

    // Body reader
    IBodyFrameReader*       m_pBodyFrameReader;

    // Direct2D
    ID2D1Factory*           m_pD2DFactory;
    cv::Mat cvWindow;
    // Body/hand drawing
    ID2D1HwndRenderTarget*  m_pRenderTarget;
    ID2D1SolidColorBrush*   m_pBrushJointTracked;
    ID2D1SolidColorBrush*   m_pBrushJointInferred;
    ID2D1SolidColorBrush*   m_pBrushBoneTracked;
    ID2D1SolidColorBrush*   m_pBrushBoneInferred;
    ID2D1SolidColorBrush*   m_pBrushHandClosed;
    ID2D1SolidColorBrush*   m_pBrushHandOpen;
    ID2D1SolidColorBrush*   m_pBrushHandLasso;
    ID2D1SolidColorBrush*   m_intersectionPointerBrush;

    void                    writeScreenLSQ(IBody* pBody, uint32_t index);
    void                    savePlaneToPly(int samples, IBody* pBody);
    void                    calibration(IBody* pBody, uint32_t index);
    void                    approximateScreenPlane(uint32_t index);
    void                    findPointerInPlane(IBody* pBody, uint32_t index);
    void                    findIntersections(uint32_t index);
    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();

    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 InitializeDefaultSensor();
    
    /// <summary>
    /// Handle new body data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="nBodyCount">body data count</param>
    /// <param name="ppBodies">body data in frame</param>
    /// </summary>
    void                    ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);

    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    /// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
    /// <param name="bForce">force status update</param>
    bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

    /// <summary>
    /// Ensure necessary Direct2d resources are created
    /// </summary>
    /// <returns>S_OK if successful, otherwise an error code</returns>
    HRESULT EnsureDirect2DResources();

    /// <summary>
    /// Dispose Direct2d resources 
    /// </summary>
    void DiscardDirect2DResources();

    void GesturalUpdate();
    void calibration(uint32_t i);
    void calculatePointing(INT64 nTime, int nBodyCount, IBody** ppBodies);
    /// <summary>
    /// Converts a body point to screen space
    /// </summary>
    /// <param name="bodyPoint">body point to tranform</param>
    /// <param name="width">width (in pixels) of output buffer</param>
    /// <param name="height">height (in pixels) of output buffer</param>
    /// <returns>point in screen-space</returns>
    D2D1_POINT_2F           BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);

    /// <summary>
    /// Draws a body 
    /// </summary>
    /// <param name="pJoints">joint data</param>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    void                    DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);

    /// <summary>
    /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
    /// </summary>
    /// <param name="handState">state of the hand</param>
    /// <param name="handPosition">position of the hand</param>
    void                    DrawHand(HandState handState, const D2D1_POINT_2F& handPosition);

    /// <summary>
    /// Draws one bone of a body (joint to joint)
    /// </summary>
    /// <param name="pJoints">joint data</param>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    /// <param name="joint0">one joint of the bone to draw</param>
    /// <param name="joint1">other joint of the bone to draw</param>
    void                    DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);
};
  
