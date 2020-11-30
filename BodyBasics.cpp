//------------------------------------------------------------------------------
// <copyright file="BodyBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "BodyBasics.h"
#include <iostream>

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(    
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CBodyBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CBodyBasics::CBodyBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_pKinectSensor(NULL),
    m_pCoordinateMapper(NULL),
    m_pBodyFrameReader(NULL),
    m_pD2DFactory(NULL),
    m_pRenderTarget(NULL),
    m_pBrushJointTracked(NULL),
    m_pBrushJointInferred(NULL),
    m_pBrushBoneTracked(NULL),
    m_pBrushBoneInferred(NULL),
    m_pBrushHandClosed(NULL),
    m_pBrushHandOpen(NULL),
    m_pBrushHandLasso(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }
    
  //  app = new GraphicsApplication(1080, 720);
    

}
  

/// <summary>
/// Destructor
/// </summary>
CBodyBasics::~CBodyBasics()
{
    DiscardDirect2DResources();

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with body frame reader
    SafeRelease(m_pBodyFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CBodyBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"BodyBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CBodyBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);
    
    //initialise calibration information
    pointingInfo.resize(BODY_COUNT);
    cornersForFrames.resize(BODY_COUNT);
    for (size_t i = 0; i < BODY_COUNT; i++) {
        pointingInfo[i].points.resize(3);

        pointingInfo[i].directions.resize(3);
        cornersForFrames[i].resize(4);
    }
    // Main message loop
    while (WM_QUIT != msg.message)
    {
        //OutputDebugString(L"yay");

        GesturalUpdate(); 
       // Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}


void CBodyBasics::GesturalUpdate() {
    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    //if the body has been recorded
    if (SUCCEEDED(hr))
    {

        //we want to find the left arm, and consider where it points
    //    std::cout << "Successfully recorded body " << std::endl;
        INT64 nTime = 0;

        hr = pBodyFrame->get_RelativeTime(&nTime);

        IBody* ppBodies[BODY_COUNT] = { 0 };
        

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            //find arms
            calculatePointing(nTime, BODY_COUNT, ppBodies);
            ProcessBody(nTime, BODY_COUNT, ppBodies);
          //  ProcessBody(nTime, BODY_COUNT, ppBodies);
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }

    SafeRelease(pBodyFrame);


}
/// <summary>
/// Main processing function
/// </summary>
/// 
/// 
void CBodyBasics::Update()
{
    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {

    //    std::cout << "Successfully recorded body " << std::endl;
        INT64 nTime = 0;

        hr = pBodyFrame->get_RelativeTime(&nTime);

        IBody* ppBodies[BODY_COUNT] = {0};

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            ProcessBody(nTime, BODY_COUNT, ppBodies);
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }

    SafeRelease(pBodyFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CBodyBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CBodyBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CBodyBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CBodyBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* pBodyFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        SafeRelease(pBodyFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

std::wstring s2ws(const std::string& s)
{
    int len;
    int slength = (int)s.length() + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
    wchar_t* buf = new wchar_t[len];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring r(buf);
    delete[] buf;
    return r;
}

bool CBodyBasics::isPointing(Eigen::Vector3f current, uint32_t index) {
    if (pointingInfo[index].previousDirection == nullptr) {
        pointingInfo[index].previousDirection = new Eigen::Vector3f(current);
        return false;
    }
    else {

        Eigen::Vector3f difference = current - *pointingInfo[index].previousDirection;
        pointingInfo[index].previousDirection = new Eigen::Vector3f(current);  
        float l = difference.norm();

        if (abs(difference.norm()) < 0.05) {
            return true;
        }
        return false;

    }
}


void CBodyBasics::approximateScreenPlane(uint32_t index) {

    //need to iterate this at some point
    Eigen::Vector3f p_1 = cornersForFrames[index][0];
    Eigen::Vector3f p_2 = cornersForFrames[index][1];
    Eigen::Vector3f p_3 = cornersForFrames[index][2];


    Eigen::Vector3f x_1 = p_2 - p_1;
    Eigen::Vector3f x_2 = p_3 - p_1;

    Eigen::Matrix3f lSquares = Eigen::Matrix3f::Zero();
    Eigen::Vector3f B = Eigen::Vector3f::Zero();
    std::vector<float> mat;
    mat.resize(9);
    for (int i = 0; i < 4; i++) {
        Eigen::Vector3f corner = cornersForFrames[index][i];
        lSquares(0, 0) += corner.x() * corner.x();
        lSquares(0, 1) += corner.x() * corner.y();
        lSquares(0, 2) += corner.x();
        lSquares(1, 0) += corner.x() * corner.y();
        lSquares(1, 1) += corner.y() * corner.y();
        lSquares(1, 2) += corner.y();
        lSquares(2, 0) += corner.x();
        lSquares(2, 1) += corner.y();
        lSquares(2, 2) += 1;

        B.x() += corner.x() * corner.z();
        B.y() += corner.y() * corner.z();
        B.z() += corner.z();

    }

    Eigen::Matrix3f toinvert = lSquares.transpose() * lSquares;
    Eigen::Vector3f normal = toinvert.inverse() * lSquares.transpose() * B;
    screenPlane = Eigen::Hyperplane<float, 3>(normal, 0);

    //screenPlane = Eigen::Hyperplane<float, 3>::Through(x_1, x_2);
    pointingInfo[index].screenPlaneFound = true;
    //compute plane equation
    //igen::Hyperplane plane = Eigen::Hyperplane::Through(x_1, x_2);
    
    // Eigen::Hyperplane e = Eigen::Hyperplane::Through(x_1, x_2);

}

//compute a homography from the subject's 'frame' to the screen defined by the 4 least squares points found in the previous step (findInteresections)
//then use the resulting homography matrix to map the subject's current arm position to the screen
void CBodyBasics::findPointerInPlane(IBody* pBody, uint32_t index) {
    //calculate the intersection with the screen plane.
    Joint joints[JointType_Count];
    HRESULT hr = pBody->GetJoints(_countof(joints), joints);
    Eigen::Vector3f leftArm(joints[JointType_WristLeft].Position.X, joints[JointType_WristLeft].Position.Y, joints[JointType_WristLeft].Position.Z);
    Eigen::Vector3f leftElbow(joints[JointType_ElbowLeft].Position.X, joints[JointType_ElbowLeft].Position.Y, joints[JointType_ElbowLeft].Position.Z);
    Eigen::Vector3f direction = leftArm - leftElbow;


    //point == leftArm

  

   // lSquares << mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6], mat[7], mat[8];
            
    //direction == direction

    //defines line leftArm + t*direction
    //find intersection with screenPlane

    Eigen::ParametrizedLine<float, 3> pointingDirection = Eigen::ParametrizedLine<float, 3>::Through(leftElbow, leftArm);

    double parameter_step = pointingDirection.intersection(screenPlane);
    Eigen::Vector3f intersection = parameter_step * ((leftArm - leftElbow).normalized()) + leftElbow;
    std::cout << intersection << std::endl;

    std::stringstream ss;
    ss << intersection;
    std::wstring index_w = s2ws(ss.str());
    LPCWSTR re_index = index_w.c_str();
    OutputDebugString(re_index);
}

//solve for the closest point to a set of lines using least squares -- we use this as our corner.
void CBodyBasics::findIntersections(uint32_t index) {
    //solve for intersections of   

    std::vector<Eigen::Vector3f> cornerSolutions;
    //setup linear system
   

    //double check this works later
    //always 4 directions from the user's reference frame
    for (size_t j = 0; j < 4; j++) {
        Eigen::Matrix3f e = Eigen::Matrix3f::Identity();
        Eigen::Vector3f pointSum = Eigen::Vector3f::Zero();
        for (size_t i = 0; i < pointingInfo[index].directions.size(); i++) {
            Eigen::Matrix3f outerProduct = pointingInfo[index].directions[i][j] * pointingInfo[index].directions[i][j].transpose();
            e -= outerProduct;
            pointSum += pointingInfo[index].points[i][j];
        }
      //  Eigen::Matrix3f inverse = e.inverse();
        Eigen::Vector3f x = e.inverse() * pointSum;
        cornerSolutions.push_back(x);
        //find the lest squares point for this set of lines
    }
    cornersForFrames[index] = cornerSolutions;
}

void CBodyBasics::calibration(IBody* pBody, uint32_t i) {
    Joint joints[JointType_Count];
    HRESULT hr = pBody->GetJoints(_countof(joints), joints);
    Eigen::Vector3f leftArm(joints[JointType_WristLeft].Position.X, joints[JointType_WristLeft].Position.Y, joints[JointType_WristLeft].Position.Z);
    Eigen::Vector3f leftElbow(joints[JointType_ElbowLeft].Position.X, joints[JointType_ElbowLeft].Position.Y, joints[JointType_ElbowLeft].Position.Z);
    Eigen::Vector3f direction = leftArm - leftElbow;
    //Eigen::Vector3f point(leftArm);
    //not much change
    bool point = isPointing(direction, i);

    if (point && pointingInfo[i].firstPointer == false) {
        pointingInfo[i].firstPointer = true;
        pointingInfo[i].startingPoints = new Eigen::Vector3f(direction);
        pointingInfo[i].beginTime = clock();
    }

    else {
        if (pointingInfo[i].firstPointer == true) {
            clock_t end = clock();
            double elapsed_secs = double(end - pointingInfo[i].beginTime) / CLOCKS_PER_SEC;
            Eigen::Vector3f e = direction - *pointingInfo[i].startingPoints;
            if ((elapsed_secs >= 2) && (abs(e.norm()) < 0.05)) {
                OutputDebugString(L"\n\nPointed!\n\n");
                pointingInfo[i].firstPointer = false;
                firstPoint = false;
                pointingInfo[i].beginTime = clock();

                if (pointingInfo[i].index < 3) {
                    std::wstring index_w = s2ws(std::to_string(pointingInfo[i].index));
                    LPCWSTR re_index = index_w.c_str();
                    OutputDebugString(re_index);
                    //push the current pointing direction to the corner vector
                    pointingInfo[i].points[pointingInfo[i].index].push_back(leftArm);
                    pointingInfo[i].directions[pointingInfo[i].index].push_back(direction);
                    if (pointingInfo[i].directions[pointingInfo[i].index].size() >= 4) {
                        pointingInfo[i].index += 1;
                    }
                }
                else {
                    pointingInfo[i].calibrated = true;
                }
            }
            else if (elapsed_secs > 2) {
                pointingInfo[i].firstPointer = false;
                pointingInfo[i].beginTime = clock();
            }
        }
    }
}


void CBodyBasics::calculatePointing(INT64 nTime, int nBodyCount, IBody** ppBodies) {
    //get left wrist and elbow
    for (int i = 0; i < nBodyCount; i++) {
        IBody* pBody = ppBodies[i];
        if (pBody) {
            BOOLEAN tracked = false;
            HRESULT hr = pBody->get_IsTracked(&tracked);
            //if tracked and not calibrated
            if (SUCCEEDED(hr) && tracked) {
                //want to get the left arm, i.e., left-wrist - left-elbow of this body
                if (pointingInfo[i].calibrated == false) {
                    calibration(pBody, i);
                }
                else {
                    if (pointingInfo[i].screenPlaneFound == false) {
                        std::wstring index_w = s2ws("Finding screen plane");
                        LPCWSTR re_index = index_w.c_str();
                        OutputDebugString(re_index);                        
                        findIntersections(i);
                        //followed by find screen plane
                        approximateScreenPlane(i);
                        
                    }
                    //find the intersection of the pointing person with the screen plane
                    findPointerInPlane(pBody, i);

                    
                }
            }
        }
    }
}
/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void CBodyBasics::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
    if (m_hWnd)
    {
        HRESULT hr = EnsureDirect2DResources();

        if (SUCCEEDED(hr) && m_pRenderTarget && m_pCoordinateMapper)
        {
            m_pRenderTarget->BeginDraw();
            m_pRenderTarget->Clear();

            RECT rct;
            GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
            int width = rct.right;
            int height = rct.bottom;

            
            for (int i = 0; i < nBodyCount; ++i)
            {
                IBody* pBody = ppBodies[i];
                if (pBody)
                {
                    BOOLEAN bTracked = false;
                    hr = pBody->get_IsTracked(&bTracked);

                    if (SUCCEEDED(hr) && bTracked)
                    {
                        Joint joints[JointType_Count]; 

                      
                        D2D1_POINT_2F jointPoints[JointType_Count];
                        HandState leftHandState = HandState_Unknown;
                        HandState rightHandState = HandState_Unknown;

                        pBody->get_HandLeftState(&leftHandState);
                        pBody->get_HandRightState(&rightHandState);

                        hr = pBody->GetJoints(_countof(joints), joints);
                        if (SUCCEEDED(hr))
                        {
                          //  jointPoints[6] = BodyToScreen(joints[6].Position, width, height);
                            //jointPoints[7] = BodyToScreen(joints[7].Position, width, height);
                            for (int j = 0; j < _countof(joints); ++j)
                            {
                                jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
                               // std::cout << joints[j].Position << std::endl;
                            }

                            DrawBody(joints, jointPoints);

                            DrawHand(leftHandState, jointPoints[JointType_HandLeft]);
                            DrawHand(rightHandState, jointPoints[JointType_HandRight]);
                        }
                    }
                }
            }

            hr = m_pRenderTarget->EndDraw();

            // Device lost, need to recreate the render target
            // We'll dispose it now and retry drawing
            if (D2DERR_RECREATE_TARGET == hr)
            {
                hr = S_OK;
                DiscardDirect2DResources();
            }
        }

        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CBodyBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT CBodyBasics::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    if (m_pD2DFactory && !m_pRenderTarget)
    {
        RECT rc;
        GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);  

        int width = rc.right - rc.left;
        int height = rc.bottom - rc.top;
        D2D1_SIZE_U size = D2D1::SizeU(width, height);
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a Hwnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size),
            &m_pRenderTarget
        );

        if (FAILED(hr))
        {
            SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
            return hr;
        }

        // light green
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 0.5f), &m_pBrushHandClosed);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 0.5f), &m_pBrushHandOpen);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushHandLasso);
    }

    return hr;
}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void CBodyBasics::DiscardDirect2DResources()
{
    SafeRelease(m_pRenderTarget);

    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushJointInferred);
    SafeRelease(m_pBrushBoneTracked);
    SafeRelease(m_pBrushBoneInferred);

    SafeRelease(m_pBrushHandClosed);
    SafeRelease(m_pBrushHandOpen);
    SafeRelease(m_pBrushHandLasso);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F CBodyBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
    // Calculate the body's position on the screen
    DepthSpacePoint depthPoint = {0};
    m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

    float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
    float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

    return D2D1::Point2F(screenPointX, screenPointY);
}

/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void CBodyBasics::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints)
{
    // Draw the bones

    // Torso
    DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
    DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid);
    DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);
    
    // Right Arm    
    DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
    DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
    DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);

    // Left Arm
    DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
    DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
    DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);

    // Right Leg
    DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight);
    DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight);
    DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight);

    // Left Leg
    DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft);
    DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft);
    DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft);

    // Draw the joints
    for (int i = 0; i < JointType_Count; ++i)
    {
        D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);

        if (pJoints[i].TrackingState == TrackingState_Inferred)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
        }
        else if (pJoints[i].TrackingState == TrackingState_Tracked)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
        }
    }
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void CBodyBasics::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1)
{
    TrackingState joint0State = pJoints[joint0].TrackingState;
    TrackingState joint1State = pJoints[joint1].TrackingState;

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
    }
    else
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
    }
}

/// <summary>
/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
/// </summary>
/// <param name="handState">state of the hand</param>
/// <param name="handPosition">position of the hand</param>
void CBodyBasics::DrawHand(HandState handState, const D2D1_POINT_2F& handPosition)
{
    D2D1_ELLIPSE ellipse = D2D1::Ellipse(handPosition, c_HandSize, c_HandSize);

    switch (handState)
    {
        case HandState_Closed:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
            break;

        case HandState_Open:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandOpen);
            break;

        case HandState_Lasso:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandLasso);
            break;
    }
}
