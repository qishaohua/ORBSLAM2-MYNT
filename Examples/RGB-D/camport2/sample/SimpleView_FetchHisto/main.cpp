#include "../common/common.hpp"

static char buffer[1024 * 1024];
static int  n;
static volatile bool exit_main;


struct CallbackData {
    int             index;
    TY_DEV_HANDLE   hDevice;
    DepthRender*    render;
};

void dump_hist(cv::Mat hist) {
    printf("dump hist!\n");
    int i = 0;
    int *data = (int *)hist.data;
    for(i =0;i < 512; i++) {
        if (data[i]) {
            printf("%d %u!\n", i, data[i]);
        }
    }
}

void frameHandler(TY_FRAME_DATA* frame, void* userdata) {
    CallbackData* pData = (CallbackData*) userdata;
    LOGD("=== Get frame %d", ++pData->index);

    for( int i = 0; i < frame->validCount; i++ ){
        if(frame->image[i].componentID == TY_COMPONENT_BRIGHT_HISTO) {
            int32_t *ir_left_his, *ir_right_his;
            ir_left_his = (int32_t *)frame->image[i].buffer;
            ir_right_his = (int32_t *)frame->image[i].buffer + 256;
            
            int i;
            for( i=0; i<256; i++) {
                printf("ir_left_his[%d] = %u,         ir_right_his[%d] = %u\n", i, ir_left_his[i], i, ir_right_his[i]);
            }
        }
    }

    int key = cv::waitKey(1);
    switch(key & 0xff) {
    case 0xff:
        break;
    case 'q':
        exit_main = true;
        break;
    default:
        LOGD("Unmapped key %d", key);
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

int main(int argc, char* argv[]) {
    const char* IP = NULL;
    TY_DEV_HANDLE hDevice;

    for(int i = 1; i < argc; i++) {
        if(strcmp(argv[i], "-ip") == 0) {
            IP = argv[++i];
        } else if(strcmp(argv[i], "-h") == 0) {
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
        LOGD("=== Get device info");
        ASSERT_OK( TYGetDeviceNumber(&n) );
        LOGD("     - device number %d", n);

        TY_DEVICE_BASE_INFO* pBaseInfo = (TY_DEVICE_BASE_INFO*)buffer;
        ASSERT_OK( TYGetDeviceList(pBaseInfo, 100, &n) );

        if(n == 0) {
            LOGD("=== No device got");
            return -1;
        }

        LOGD("=== Open device 0, pBaseInfo[0].id = %s", pBaseInfo[0].id);
        ASSERT_OK( TYOpenDevice(pBaseInfo[0].id, &hDevice) );
    }

    int32_t allComps, componentIDs = 0;
    ASSERT_OK( TYGetComponentIDs(hDevice, &allComps) );
    if(allComps & TY_COMPONENT_BRIGHT_HISTO) {
        LOGD("=== Has bright histo component");
        componentIDs |= TY_COMPONENT_BRIGHT_HISTO; 
    }

    LOGD("=== Configure components, open depth cam");
    componentIDs |= TY_COMPONENT_IR_CAM_RIGHT| TY_COMPONENT_IR_CAM_LEFT; 
    //int32_t componentIDs = TY_COMPONENT_RGB_CAM | TY_COMPONENT_IR_CAM_LEFT;
    ASSERT_OK( TYEnableComponents(hDevice, componentIDs) );

    LOGD("=== Configure feature, set resolution to 640x480.");
    LOGD("Note: DM460 resolution feature is in component TY_COMPONENT_DEVICE,");
    LOGD("      other device may lays in some other components.");
    TY_FEATURE_INFO info;
    TY_STATUS ty_status = TYGetFeatureInfo(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, &info);
    if ((info.accessMode & TY_ACCESS_WRITABLE) && (ty_status == TY_STATUS_OK)) {
        int err = TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_640x480);
        ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);
    }

    LOGD("=== Prepare image buffer");
    int32_t frameSize;
    ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
    LOGD("     - Get size of framebuffer, %d", frameSize);
    ASSERT( frameSize >= 640 * 480 * 2 );

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
    // ASSERT_OK( TYRegisterCallback(hDevice, frameHandler, &cb_data) );

    LOGD("=== Register event callback");
    LOGD("Note: Callback may block internal data receiving,");
    LOGD("      so that user should not do long time work in callback.");
    ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, NULL));

    LOGD("=== Disable trigger mode");
    ty_status = TYGetFeatureInfo(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE, &info);
    if ((info.accessMode & TY_ACCESS_WRITABLE) && (ty_status == TY_STATUS_OK)) {
        ASSERT_OK(TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE, false));
    }

    LOGD("=== Start capture");
    ASSERT_OK( TYStartCapture(hDevice) );

    LOGD("=== While loop to fetch frame");
    exit_main = false;
    TY_FRAME_DATA frame;

    while(!exit_main) {
        int err = TYFetchFrame(hDevice, &frame, -1);
        if( err != TY_STATUS_OK ) {
            LOGD("... Drop one frame");
        } else {
            frameHandler(&frame, &cb_data);
        }
    }

    ASSERT_OK( TYStopCapture(hDevice) );
    ASSERT_OK( TYCloseDevice(hDevice) );
    ASSERT_OK( TYDeinitLib() );
    // MSLEEP(10); // sleep to ensure buffer is not used any more
    delete frameBuffer[0];
    delete frameBuffer[1];

    LOGD("=== Main done!");
    return 0;
}
