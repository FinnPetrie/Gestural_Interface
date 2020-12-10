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
    
   // app = new GraphicsApplication(1080, 720);
    

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
        pointingInfo[i].secondaryPoints.resize(3);

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

static std::string toString(const Eigen::MatrixXf& mat) {
    std::stringstream ss;
    ss << mat;
    return ss.str();
}

void printMat(const Eigen::MatrixXf& mat) {
    std::string ma = toString(mat);
    std::wstring st = s2ws(ma);
    LPCWSTR s = st.c_str();
    OutputDebugString(s);
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



void CBodyBasics::savePlaneToPly(int samples, IBody* pBody) {
    std::vector<Eigen::Vector3f> planePoints;
    Eigen::Vector4f coeffs = screenPlane.coeffs();
    float maxValue = 0;
    float index = 0;
    for (int i = 0; i < 3; i++) {
        if (abs(coeffs(i)) > maxValue) {
            maxValue = abs(coeffs(i));
            index = i;
        }
    }

    Eigen::Vector3f i = Eigen::Vector3f(coeffs(3) / coeffs(index), 0, 0);
    Eigen::Vector3f j = i.cross(screenPlane.normal());

    //compute parametric plane form

    for (float x = -1; x < 1; x+= 0.1) {
        for (float y = -1; y < 1; y+=0.1) {
            Eigen::Vector3f point = (x) * i.normalized() + (y) * j.normalized();
            glm::vec3 glPoint(point.x(), point.y(), point.z());
            planePoints.push_back(point);
        }
    }

    std::vector<Vertex> vertices;
    for (int i = 0; i < planePoints.size(); i++) {
        Eigen::Vector3d vert = planePoints[i].cast<double>();
        Vertex v = { vert, Eigen::Vector3i(255, 0, 0), Eigen::Vector3d(0,0,0) };
        vertices.push_back(v);
    }

    Joint joints[JointType_Count];
    HRESULT hr = pBody->GetJoints(_countof(joints), joints);

    for (int i = 0; i < JointType_Count; i++) {
        Eigen::Vector3d joint(joints[i].Position.X, joints[i].Position.Y, joints[i].Position.Z);
        Vertex v = { joint, Eigen::Vector3i(255, 0, 255), Eigen::Vector3d(0,0,0) };
        vertices.push_back(v);
    }

    PlyFile plane(vertices);
    plane.write("Plane.ply");
}

void CBodyBasics::approximateScreenPlane(uint32_t index) {

    //need to iterate this at some point
    Eigen::Vector3f p_1 = cornersForFrames[index][0];
    Eigen::Vector3f p_2 = cornersForFrames[index][1];
    Eigen::Vector3f p_3 = cornersForFrames[index][2];

    Eigen::Vector3d cen = Eigen::Vector3d::Zero();

    for (int i = 0; i < cornersForFrames[index].size(); i++) {
        cen += cornersForFrames[index][i].cast<double>();
    }
    cen /= cornersForFrames[index].size();
    centroid = cen.cast<float>();
    Eigen::Matrix3Xd corners(3, cornersForFrames[index].size());
    for (int i = 0; i < cornersForFrames[index].size(); i++) {
        Eigen::Vector3d corner_prime = cornersForFrames[index][i].cast<double>() - cen;
        OutputDebugString(L"Printing vector: \n\n\n");
        printMat(corner_prime.cast<float>());
        OutputDebugString(L"\n\n\n");

        corners.col(i) = corner_prime;
    }

    OutputDebugString(L"Matrix for SVD\n\n");
    std::stringstream ss;
    ss << corners;
    std::wstring index_w = s2ws(ss.str());
    LPCWSTR re_index = index_w.c_str();
    OutputDebugString(re_index);


   Eigen::JacobiSVD<Eigen::Matrix3Xd> svd = corners.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd U = svd.matrixU();
    OutputDebugString(L"Matrix V\n");
    printMat(V.cast<float>());
    OutputDebugString(L"\n");

    OutputDebugString(L"Matrix U: \n");
    printMat(U.cast<float>());
    OutputDebugString(L"\n");
    screenPlane = Eigen::Hyperplane<float, 3>::Through(U.col(0).cast<float>(), U.col(1).cast<float>());
    pointingInfo[index].screenPlaneFound = true;

    //compute rotation matrix
    Eigen::Matrix3f rotationMat = Eigen::Matrix3f::Identity();
    Eigen::Vector4f coeffs = screenPlane.coeffs();
    
    float aSquared = pow(coeffs.x(), 2);
    float bSquared = pow(coeffs.y(), 2);
    float cSquared = pow(coeffs.z(), 2);
    float cosTheta = coeffs.z() / (sqrt(aSquared + bSquared + cSquared));
    float sinTheta = sqrt((aSquared + bSquared) / (aSquared + bSquared + cSquared));
    float uOne = coeffs.y() / (sqrt(aSquared + bSquared + cSquared));
    float uTwo = -coeffs.x() / (sqrt(aSquared + bSquared + cSquared));


    //need to also translate the pal
    rotationMat << cosTheta + uOne * uOne * (1 - cosTheta), uOne* uTwo* (1 - cosTheta), uTwo* sinTheta,
        uOne* uTwo* (1 - cosTheta), cosTheta* uTwo* uTwo* (1 - cosTheta), -uOne * sinTheta,
        -uTwo * sinTheta, uOne* sinTheta, cosTheta;


    


    Eigen::Vector3f i = U.col(0).cast<float>();
    Eigen::Vector3f j = U.col(1).cast<float>();
    std::vector<Eigen::Vector3f> planePoints;
    for (float x = -1; x < 1; x += 0.1) {
        for (float y = -1; y < 1; y += 0.1) {
            Eigen::Vector3f point = (x)*i.normalized() + (y)*j + centroid;
            glm::vec3 glPoint(point.x(), point.y(), point.z());
            planePoints.push_back(point);
        }
    }

    std::vector<Vertex> vertices;
    for (int i = 0; i < planePoints.size(); i++) {
        Eigen::Vector3d vert = planePoints[i].cast<double>();
        Vertex v = { vert, Eigen::Vector3i(255, 0, 0), Eigen::Vector3d(0,0,0) };
        vertices.push_back(v);
    }
    
    PlyFile p(vertices);
    p.write("Plane.ply");


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
    float x = intersection.x();
    float y = intersection.y();
    D2D1_POINT_2F point = D2D1::Point2F(x, y);
    D2D1_ELLIPSE ellipse = D2D1::Ellipse(point, 1, 1);
    m_pRenderTarget->BeginDraw();
    //m_pRenderTarget->Clear();

    RECT rct;
    GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
    int width = rct.right;
    int height = rct.bottom;
    m_pRenderTarget->DrawEllipse(ellipse, m_intersectionPointerBrush, 10.0f);
    m_pRenderTarget->FillEllipse(ellipse, m_intersectionPointerBrush);  

    std::vector<Eigen::Vector3f> cornerSolutions;
    //setup linear system
   

    //double check this works later
    //always 4 directions from the user's reference frame
    std::vector<Vertex> vertices;
    for (size_t j = 0; j < 4; j++) {
        Eigen::MatrixXf e_sum = Eigen::Matrix3f::Zero();
        Eigen::Vector3f pointSum = Eigen::Vector3f::Zero();
        for (size_t i = 0; i < pointingInfo[index].directions.size(); i++) {
            Eigen::Vector3f directionNorm = pointingInfo[index].directions[i][j].normalized();

            for (float p = -10; p < 10; p += 0.1) {
                Eigen::Vector3d q = directionNorm.cast<double>() * p + pointingInfo[index].points[i][j].cast<double>();
                Eigen::Vector3i colour = Eigen::Vector3i::Zero();
                switch (j) {
                case 0:
                    colour.x() = 255;
                    break;
                case 1:
                    colour.y() = 255;
                    break;
                case 2:
                    colour.z() = 255;
                    break;
                case 3:
                    colour.x() = 255;
                    colour.y() = 255;
                    colour.z() = 255;
                    break;
                default:
                    break;
                }
                Vertex v = { q, colour, Eigen::Vector3d(0,0,0) };
                vertices.push_back(v);
            }
           // Eigen::Vector3f directionNorm = pointingInfo[index].directions[i][j].normalized();
           // directionNorm = directionNorm.normalized();
            //Eigen::Matrix3f outerProduct = pointingInfo[index].directions[i][j] * pointingInfo[index].directions[i][j].transpose();
            Eigen::MatrixXf outerProduct = directionNorm * directionNorm.transpose();
            Eigen::MatrixXf I_e = Eigen::Matrix3f::Identity();
            I_e -= outerProduct;
            e_sum += I_e;
            //e -= outerProduct;
            pointSum += I_e*pointingInfo[index].secondaryPoints[i][j];
        }
      //  Eigen::Matrix3f inverse = e.inverse();
        Eigen::Vector3f x = e_sum.inverse() * pointSum;
            
        Eigen::VectorXf sol = e_sum.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(pointSum);
        Vertex v = { sol.cast<double>(), Eigen::Vector3i(255, 255, 0), Eigen::Vector3d(0,0,0) };
        vertices.push_back(v);
        OutputDebugString(L"Printing ith set of solutions \n");
        printMat(sol);
        OutputDebugString(L"Printed\n");
        cornerSolutions.push_back(sol);
        //find the least squares point for this set of lines
    }

    PlyFile p(vertices);
    p.write("LineSystem.ply");
    cornersForFrames[index] = cornerSolutions;
} 
void CBodyBasics::writeScreenLSQ(IBody* pBody, uint32_t index) {
    std::vector<Vertex> things;


    for (int i = 0;i < cornersForFrames[index].size(); i++) {
        Eigen::Vector3d vert = cornersForFrames[index][i].cast<double>();
        Vertex v = { vert, Eigen::Vector3i(0,0, 255), Eigen::Vector3d(0,0,0) };
        things.push_back(v);
    }

    Joint joints[JointType_Count];
    HRESULT hr = pBody->GetJoints(_countof(joints), joints);

    for (int i = 0; i < JointType_Count; i++) {
        Eigen::Vector3d joint(joints[i].Position.X, joints[i].Position.Y, joints[i].Position.Z);
        Vertex v = { joint, Eigen::Vector3i(0, 255, 0), Eigen::Vector3d(0,0,0) };
        things.push_back(v);
    }

    PlyFile corners(things);
    corners.write("Corners.ply");
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
            if ((elapsed_secs >= 3) && (abs(e.norm()) < 0.05)) {
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
                    pointingInfo[i].secondaryPoints[pointingInfo[i].index].push_back(leftElbow);

                    if (pointingInfo[i].directions[pointingInfo[i].index].size() >= 4) {
                        pointingInfo[i].index += 1;
                    }
                }
                else {
                    pointingInfo[i].calibrated = true;
                }
            }
            else if (elapsed_secs > 3) {
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
                    //check to see if the screen plane has already been stored.
                    //FILE *file = fopen("Plane.ply", )
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
                        writeScreenLSQ(pBody, i);
                    }
                    //find the intersection of the pointing person with the screen plane
                    findPointerInPlane(pBody, i);
                    OutputDebugString(L"Finding plane");
                    if (!savePlane) {
                        
                       // savePlaneToPly(60, pBody);
                        savePlane = true;
                    }
                    
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
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::MediumVioletRed, 1.0f), &m_intersectionPointerBrush);
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
