#include "../common/common.hpp"

static char buffer[1024*1024*20];
static int  n;
static volatile bool exit_main;
static volatile bool save_frame;

struct CallbackData {
    int             index;
    TY_DEV_HANDLE   hDevice;
    DepthRender*    render;

    TY_CAMERA_DISTORTION color_dist;
    TY_CAMERA_INTRINSIC color_intri;
};

void handleFrame(TY_FRAME_DATA* frame, void* userdata)
{
    CallbackData* pData = (CallbackData*) userdata;
    LOGD("=== Get frame %d", ++pData->index);

    cv::Mat depth, irl, irr, color, point3D;
    parseFrame(*frame, &depth, &irl, &irr, &color, &point3D);
    if(!depth.empty()){
        cv::Mat colorDepth = pData->render->Compute(depth);
        cv::imshow("ColorDepth", colorDepth);
    }
    if(!irl.empty()){ cv::imshow("LeftIR", irl); }
    if(!irr.empty()){ cv::imshow("RightIR", irr); }
    if(!color.empty()){
        cv::Mat undistort_result(color.size(), CV_8UC3);
        TY_IMAGE_DATA dst;
        dst.width = color.cols;
        dst.height = color.rows;
        dst.size = undistort_result.size().area() * 3;
        dst.buffer = undistort_result.data;
        dst.pixelFormat = TY_PIXEL_FORMAT_RGB;
        TY_IMAGE_DATA src;
        src.width = color.cols;
        src.height = color.rows;
        src.size = color.size().area() * 3;
        src.pixelFormat = TY_PIXEL_FORMAT_RGB;
        src.buffer = color.data;
        //undistort camera image 
        //TYUndistortImage accept TY_IMAGE_DATA from TY_FRAME_DATA , pixel format RGB888 or MONO8
        //you can also use opencv API cv::undistort to do this job.
        ASSERT_OK(TYUndistortImage(&pData->color_intri, &pData->color_dist, NULL, &src, &dst));
        color = undistort_result;
        cv::Mat resizedColor;
        cv::resize(color, resizedColor, depth.size(), 0, 0, CV_INTER_LINEAR);
        cv::imshow("color", resizedColor);
    }

    // do Registration
    cv::Mat newDepth;
    if(!point3D.empty() && !color.empty()) {
        ASSERT_OK( TYRegisterWorldToColor2(pData->hDevice, (TY_VECT_3F*)point3D.data, 0
                    , point3D.cols * point3D.rows, color.cols, color.rows, (uint16_t*)buffer, sizeof(buffer)
                    ));
        newDepth = cv::Mat(color.rows, color.cols, CV_16U, (uint16_t*)buffer);
        cv::Mat resized_color;
        cv::Mat temp;
        //you may want to use median filter to fill holes in projected depth image or do something else here
        cv::medianBlur(newDepth,temp,5);
        newDepth = temp;
        //resize to the same size for display
        cv::resize(newDepth, newDepth, depth.size(), 0, 0, 0);
        cv::resize(color, resized_color, depth.size());
        cv::Mat depthColor = pData->render->Compute(newDepth);
        depthColor = depthColor / 2 + resized_color / 2;
        cv::imshow("projected depth", depthColor);
    }

    if(save_frame){
        LOGD(">>>>>>>>>> write images");
        imwrite("depth.png", newDepth);
        imwrite("color.png", color);
        save_frame = false;
    }

    int key = cv::waitKey(1);
    switch(key){
        case -1:
            break;
        case 'q': case 1048576 + 'q':
            exit_main = true;
            break;
        case 's': case 1048576 + 's':
            save_frame = true;
            break;
        default:
            LOGD("Pressed key %d", key);
    }

    LOGD("=== Callback: Re-enqueue buffer(%p, %d)", frame->userBuffer, frame->bufferSize);
    ASSERT_OK( TYEnqueueBuffer(pData->hDevice, frame->userBuffer, frame->bufferSize) );
}

void eventCallback(TY_EVENT_INFO *event_info, void *userdata)
{
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
        LOGD("=== Event Callback: Device Offline!");
        // Note: 
        //     Please set TY_BOOL_KEEP_ALIVE_ONOFF feature to false if you need to debug with breakpoint!
    }
    else if (event_info->eventId == TY_EVENT_LICENSE_ERROR) {
        LOGD("=== Event Callback: License Error!");
    }
}


int main(int argc, char* argv[])
{
    const char* IP = NULL;
    const char* ID = NULL;
    TY_DEV_HANDLE hDevice;

    for(int i = 1; i < argc; i++){
        if(strcmp(argv[i], "-id") == 0){
            ID = argv[++i];
        }else if(strcmp(argv[i], "-ip") == 0){
            IP = argv[++i];
        }else if(strcmp(argv[i], "-h") == 0){
            LOGI("Usage: SimpleView_Callback [-h] [-ip <IP>]");
            return 0;
        }
    }
    
    LOGD("=== Init lib");
    ASSERT_OK( TYInitLib() );
    TY_VERSION_INFO* pVer = (TY_VERSION_INFO*)buffer;
    ASSERT_OK( TYLibVersion(pVer) );
    LOGD("     - lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch);

    if(IP) {
        LOGD("=== Open device %s", IP);
        ASSERT_OK( TYOpenDeviceWithIP(IP, &hDevice) );
    } else {
        if(ID == NULL){
            LOGD("=== Get device info");
            ASSERT_OK( TYGetDeviceNumber(&n) );
            LOGD("     - device number %d", n);

            TY_DEVICE_BASE_INFO* pBaseInfo = (TY_DEVICE_BASE_INFO*)buffer;
            ASSERT_OK( TYGetDeviceList(pBaseInfo, 100, &n) );

            if(n == 0){
                LOGD("=== No device got");
                return -1;
            }
            ID = pBaseInfo[0].id;
        }

        LOGD("=== Open device: %s", ID);
        ASSERT_OK( TYOpenDevice(ID, &hDevice) );
    }

    int32_t allComps;
    ASSERT_OK( TYGetComponentIDs(hDevice, &allComps) );
    if(!(allComps & TY_COMPONENT_RGB_CAM)){
        LOGE("=== Has no RGB camera, cant do registration");
        return -1;
    }

    LOGD("=== Configure components");
    int32_t componentIDs = TY_COMPONENT_POINT3D_CAM | TY_COMPONENT_RGB_CAM;
    ASSERT_OK( TYEnableComponents(hDevice, componentIDs) );

    LOGD("=== Prepare image buffer");
    int32_t frameSize;

    //frameSize = 1280 * 960 * (3 + 2 + 2);
    ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
    LOGD("     - Get size of framebuffer, %d", frameSize);
    LOGD("     - Allocate & enqueue buffers");
    char* frameBuffer[2];
    frameBuffer[0] = new char[frameSize];
    frameBuffer[1] = new char[frameSize];
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize) );
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize) );

    LOGD("=== Register callback");
    LOGD("Note: Callback may block internal data receiving,");
    LOGD("      so that user should not do long time work in callback.");
    LOGD("      To avoid copying data, we pop the framebuffer from buffer queue and");
    LOGD("      give it back to user, user should call TYEnqueueBuffer to re-enqueue it.");
    DepthRender render;
    CallbackData cb_data;
    cb_data.index = 0;
    cb_data.hDevice = hDevice;
    cb_data.render = &render;
    // ASSERT_OK( TYRegisterCallback(hDevice, frameCallback, &cb_data) );

    LOGD("=== Register event callback");
    LOGD("Note: Callback may block internal data receiving,");
    LOGD("      so that user should not do long time work in callback.");
    ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, NULL));

    LOGD("=== Disable trigger mode");
    ASSERT_OK( TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE, false) );

    LOGD("=== Start capture");
    ASSERT_OK( TYStartCapture(hDevice) );

    LOGD("=== Read color rectify matrix");
    {
        TY_CAMERA_DISTORTION color_dist;
        TY_CAMERA_INTRINSIC color_intri;
        TY_STATUS ret = TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_DISTORTION, &color_dist, sizeof(color_dist));
        ret |= TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_INTRINSIC, &color_intri, sizeof(color_intri));
        if (ret == TY_STATUS_OK)
        {
            cb_data.color_intri = color_intri;
            cb_data.color_dist= color_dist;
        }
        else
        { //reading data from device failed .set some default values....
            memset(cb_data.color_dist.data, 0, 12 * sizeof(float));
            memset(cb_data.color_intri.data, 0, 9 * sizeof(float));
            cb_data.color_intri.data[0] = 1000.f;
            cb_data.color_intri.data[4] = 1000.f;
            cb_data.color_intri.data[2] = 600.f;
            cb_data.color_intri.data[5] = 450.f;
        }
    }

    LOGD("=== Wait for callback");
    exit_main = false;
    while(!exit_main){
        TY_FRAME_DATA frame;
        int err = TYFetchFrame(hDevice, &frame, -1);
        if( err != TY_STATUS_OK ) {
            LOGE("Fetch frame error %d: %s", err, TYErrorString(err));
            break;
        } else {
            handleFrame(&frame, &cb_data);
        }
    }

    ASSERT_OK( TYStopCapture(hDevice) );
    ASSERT_OK( TYCloseDevice(hDevice) );
    ASSERT_OK( TYDeinitLib() );
    delete frameBuffer[0];
    delete frameBuffer[1];

    LOGD("=== Main done!");
    return 0;
}
