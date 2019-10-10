#include "../common/common.hpp"
#include <fstream>

static char buffer[1024*1024];

struct CamInfo
{
    char                sn[32];
    TY_DEV_HANDLE       hDev;
    char*               fb[2];
    TY_FRAME_DATA       frame;
    int                 idx;
    DepthRender         render;

    CamInfo() : hDev(0), idx(0) {fb[0]=0; fb[1]=0;}
};


void frameHandler(TY_FRAME_DATA* frame, void* userdata)
{
    CamInfo* pData = (CamInfo*) userdata;

    cv::Mat depth, irl, irr, color;
    parseFrame(*frame, &depth, &irl, &irr, &color, 0);

    char win[64];
    if(!depth.empty()){
        cv::Mat colorDepth = pData->render.Compute(depth);
        sprintf(win, "depth-%s", pData->sn);
        cv::imshow(win, colorDepth);
    }
    if(!irl.empty()){
        sprintf(win, "LeftIR-%s", pData->sn);
        cv::imshow(win, irl);
    }
    if(!irr.empty()){
        sprintf(win, "RightIR-%s", pData->sn);
        cv::imshow(win, irr);
    }
    if(!color.empty()){
        sprintf(win, "color-%s", pData->sn);
        cv::imshow(win, color);
    }

    pData->idx++;

    LOGD("=== Callback: Re-enqueue buffer(%p, %d)", frame->userBuffer, frame->bufferSize);
    ASSERT_OK( TYEnqueueBuffer(pData->hDev, frame->userBuffer, frame->bufferSize) );
}

int main()
{
    LOGD("=== Init lib");
    ASSERT_OK( TYInitLib() );
    TY_VERSION_INFO* pVer = (TY_VERSION_INFO*)buffer;
    ASSERT_OK( TYLibVersion(pVer) );
    LOGD("     - lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch);

    LOGD("=== Get device info");
    int n;
    ASSERT_OK( TYGetDeviceNumber(&n) );
    LOGD("     - device number %d", n);

    TY_DEVICE_BASE_INFO* pBaseInfo = (TY_DEVICE_BASE_INFO*)buffer;
    ASSERT_OK( TYGetDeviceList(pBaseInfo, 100, &n) );

    if(n < 1){
        LOGD("=== Need more than 1 devices");
        return -1;
    }

    std::vector<CamInfo> cams(n);
    for(int i = 0; i < n; i++){
        LOGD("=== Open device %d (id: %s)", i, pBaseInfo[i].id);
        strncpy(cams[i].sn, pBaseInfo[i].id, sizeof(cams[i].sn));

        ASSERT_OK( TYOpenDevice(pBaseInfo[i].id, &cams[i].hDev) );

        int32_t allComps;
        ASSERT_OK( TYGetComponentIDs(cams[i].hDev, &allComps) );
        if(0 && allComps & TY_COMPONENT_RGB_CAM){
            LOGD("=== Has RGB camera, open RGB cam");
            ASSERT_OK( TYEnableComponents(cams[i].hDev, TY_COMPONENT_RGB_CAM) );
        }

        LOGD("=== Configure components, open depth cam");
        int32_t componentIDs = TY_COMPONENT_DEPTH_CAM;
        ASSERT_OK( TYEnableComponents(cams[i].hDev, componentIDs) );

        LOGD("=== Configure feature, set resolution to 640x480.");
        int err = TYSetEnum(cams[i].hDev, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_640x480);
        ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);

        LOGD("=== Prepare image buffer");
        int32_t frameSize;
        ASSERT_OK( TYGetFrameBufferSize(cams[i].hDev, &frameSize) );
        LOGD("     - Get size of framebuffer, %d", frameSize);
        ASSERT( frameSize >= 640*480*2 );

        LOGD("     - Allocate & enqueue buffers");
        cams[i].fb[0] = new char[frameSize];
        cams[i].fb[1] = new char[frameSize];
        LOGD("     - Enqueue buffer (%p, %d)", cams[i].fb[0], frameSize);
        ASSERT_OK( TYEnqueueBuffer(cams[i].hDev, cams[i].fb[0], frameSize) );
        LOGD("     - Enqueue buffer (%p, %d)", cams[i].fb[1], frameSize);
        ASSERT_OK( TYEnqueueBuffer(cams[i].hDev, cams[i].fb[1], frameSize) );

        // bool triggerMode = true;
        bool triggerMode = false;
        LOGD("=== Set trigger mode %d", triggerMode);
        ASSERT_OK( TYSetBool(cams[i].hDev, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE, triggerMode) );

        LOGD("=== Start capture");
        ASSERT_OK( TYStartCapture(cams[i].hDev) );
    }

    LOGD("=== While loop to fetch frame");
    bool exit_main = false;

    while(!exit_main){
        for(int i = 0; i < cams.size(); i++) {
            int err = TYFetchFrame(cams[i].hDev, &cams[i].frame, 1000);
            if( err != TY_STATUS_OK ){
                LOGD("cam %s %d ... Drop one frame", cams[i].sn, cams[i].idx);
                continue;
            }

            frameHandler(&cams[i].frame, &cams[i]);
        }

        int key = cv::waitKey(1);
        switch(key & 0xff){
            case 0xff:
                break;
            case 'q':
                exit_main = true;
                break;
            default:
                LOGD("Unmapped key %d", key);
        }
    }

    for(int i = 0; i < cams.size(); i++){
        ASSERT_OK( TYStopCapture(cams[i].hDev) );
        ASSERT_OK( TYCloseDevice(cams[i].hDev) );
        // MSLEEP(10); // sleep to ensure buffer is not used any more
        delete cams[i].fb[0];
        delete cams[i].fb[1];
    }
    ASSERT_OK( TYDeinitLib() );

    LOGD("=== Main done!");
    return 0;
}
