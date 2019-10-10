#include "../common/common.hpp"

static char     buffer[1024*1024];
static int32_t  n;

void dumpFeature(TY_DEV_HANDLE handle, TY_COMPONENT_ID compID, TY_FEATURE_ID featID, const char* name)
{
    TY_FEATURE_INFO featInfo;
    ASSERT_OK(TYGetFeatureInfo(handle, compID, featID, &featInfo));

    if(featInfo.isValid && (featInfo.accessMode&TY_ACCESS_READABLE)){
        LOGD("===         %s: comp(0x%x) feat(0x%x) name(%s) access(%d) bindComponent(0x%x) bindFeature(0x%x)"
                , name, featInfo.componentID, featInfo.featureID, featInfo.name
                , featInfo.accessMode, featInfo.bindComponentID, featInfo.bindFeatureID);

        if(TYFeatureType(featID) == TY_FEATURE_INT){
            ASSERT_OK(TYGetInt(handle, compID, featID, &n));
            LOGD("===         %14s: %d", "", n);
        }
        if(TYFeatureType(featID) == TY_FEATURE_FLOAT){
            float v;
            ASSERT_OK(TYGetFloat(handle, compID, featID, &v));
            LOGD("===         %14s: %f", "", v);
        }
        if(TYFeatureType(featID) == TY_FEATURE_ENUM){
            ASSERT_OK(TYGetEnumEntryCount(handle, compID, featID, &n));
            LOGD("===         %14s: entry count %d", "", n);
            if(n > 0){
                TY_ENUM_ENTRY* pEntry = new TY_ENUM_ENTRY[n];
                ASSERT_OK(TYGetEnumEntryInfo(handle, compID, featID, pEntry, n, &n));
                for(int i = 0; i < n; i++){
                    LOGD("===         %14s:     value(%d), desc(%s)", "", pEntry[i].value, pEntry[i].description);
                }
                delete [] pEntry;
            }
        }
        if(TYFeatureType(featID) == TY_FEATURE_BOOL){
            bool v;
            ASSERT_OK(TYGetBool(handle, compID, featID, &v));
            LOGD("===         %14s: %d", "", v);
        }
        if(TYFeatureType(featID) == TY_FEATURE_STRING){
            ASSERT_OK(TYGetStringBufferSize(handle, compID, featID, &n));
            LOGD("===         %14s: length(%d)", "", n);
            ASSERT_OK(TYGetString(handle, compID, featID, buffer, sizeof(buffer)));
            LOGD("===         %14s: content(%s)", "", buffer);
        }
#if 0
        if(TYFeatureType(featID) == TY_FEATURE_BYTEARRAY){
            ASSERT_OK(TYGetByteArraySize(handle, compID, featID, &n));
            LOGD("===         %14s: size(%d)", "", n);
            n = sizeof(buffer);
            ASSERT_OK(TYGetByteArray(handle, compID, featID, (uint8_t*)buffer, n, &n));
            LOGD("===         %14s: size %d", "", n);
        }
#endif
        if(TYFeatureType(featID) == TY_FEATURE_STRUCT){
            switch(featID){
                case TY_STRUCT_CAM_INTRINSIC:{
                    TY_CAMERA_INTRINSIC* p = (TY_CAMERA_INTRINSIC*)buffer;
                    ASSERT_OK(TYGetStruct(handle, compID, featID, (void*)p
                                , sizeof(TY_CAMERA_INTRINSIC)));
                    LOGD("===%23s%f %f %f", "", p->data[0], p->data[1], p->data[2]);
                    LOGD("===%23s%f %f %f", "", p->data[3], p->data[4], p->data[5]);
                    LOGD("===%23s%f %f %f", "", p->data[6], p->data[7], p->data[8]);
                    return;
                }
                case TY_STRUCT_EXTRINSIC_TO_LEFT_IR:
                case TY_STRUCT_EXTRINSIC_TO_LEFT_RGB: {
                    TY_CAMERA_EXTRINSIC* p = (TY_CAMERA_EXTRINSIC*)buffer;
                    ASSERT_OK(TYGetStruct(handle, compID, featID, (void*)p
                                , sizeof(TY_CAMERA_EXTRINSIC)));
                    LOGD("===%23s%f %f %f %f", "", p->data[0], p->data[1], p->data[2], p->data[3]);
                    LOGD("===%23s%f %f %f %f", "", p->data[4], p->data[5], p->data[6], p->data[7]);
                    LOGD("===%23s%f %f %f %f", "", p->data[8], p->data[9], p->data[10], p->data[11]);
                    LOGD("===%23s%f %f %f %f", "", p->data[12], p->data[13], p->data[14], p->data[15]);
                    return;
                }
                case TY_STRUCT_CAM_DISTORTION:{
                    TY_CAMERA_DISTORTION* p = (TY_CAMERA_DISTORTION*)buffer;
                    ASSERT_OK(TYGetStruct(handle, compID, featID, (void*)p
                                , sizeof(TY_CAMERA_DISTORTION)));
                    LOGD("===%23s%f %f %f %f", "", p->data[0], p->data[1], p->data[2], p->data[3]);
                    LOGD("===%23s%f %f %f %f", "", p->data[4], p->data[5], p->data[6], p->data[7]);
                    LOGD("===%23s%f %f %f %f", "", p->data[8], p->data[9], p->data[10], p->data[11]);
                    return;
                }
                case TY_STRUCT_WORK_MODE: {
                    TY_TRIGGER_MODE* p = (TY_TRIGGER_MODE*)buffer;
                    ASSERT_OK(TYGetStruct(handle, compID, featID, (void*)p
                                , sizeof(TY_TRIGGER_MODE)));
                    LOGD("===         %14s: mode(%d) fps(%d)", "", p->mode, p->fps);

                    return;
                }
                default:
                    LOGD("===         %s: Unknown struct", name);
                    return;
            }
        }
    }
}

#define DUMP_FEATURE(handle, compid, feature)  dumpFeature( (handle), (compid), (feature) , #feature );

void dumpComponentFeatures(TY_DEV_HANDLE handle, TY_COMPONENT_ID compID)
{
    DUMP_FEATURE(handle, compID, TY_STRUCT_CAM_INTRINSIC );
    DUMP_FEATURE(handle, compID, TY_STRUCT_EXTRINSIC_TO_LEFT_IR );
    DUMP_FEATURE(handle, compID, TY_STRUCT_EXTRINSIC_TO_LEFT_RGB );
    DUMP_FEATURE(handle, compID, TY_STRUCT_CAM_DISTORTION);
    DUMP_FEATURE(handle, compID, TY_STRUCT_WORK_MODE);

    DUMP_FEATURE(handle, compID, TY_INT_WIDTH_MAX);
    DUMP_FEATURE(handle, compID, TY_INT_HEIGHT_MAX);
    DUMP_FEATURE(handle, compID, TY_INT_OFFSET_X);
    DUMP_FEATURE(handle, compID, TY_INT_OFFSET_Y);
    DUMP_FEATURE(handle, compID, TY_INT_WIDTH);
    DUMP_FEATURE(handle, compID, TY_INT_HEIGHT);
    DUMP_FEATURE(handle, compID, TY_INT_IMAGE_SIZE);
    DUMP_FEATURE(handle, compID, TY_ENUM_PIXEL_FORMAT);
    DUMP_FEATURE(handle, compID, TY_ENUM_IMAGE_MODE);
                                                                
    DUMP_FEATURE(handle, compID, TY_BOOL_TRIGGER_MODE);
    DUMP_FEATURE(handle, compID, TY_ENUM_TRIGGER_ACTIVATION);
    DUMP_FEATURE(handle, compID, TY_INT_FRAME_PER_TRIGGER);
    DUMP_FEATURE(handle, compID, TY_BOOL_KEEP_ALIVE_ONOFF);
    DUMP_FEATURE(handle, compID, TY_INT_KEEP_ALIVE_TIMEOUT);
                                                                
    DUMP_FEATURE(handle, compID, TY_BOOL_AUTO_EXPOSURE);
    DUMP_FEATURE(handle, compID, TY_INT_EXPOSURE_TIME );
    DUMP_FEATURE(handle, compID, TY_BOOL_AUTO_GAIN);
    DUMP_FEATURE(handle, compID, TY_INT_GAIN);
                                                                
    DUMP_FEATURE(handle, compID, TY_BOOL_UNDISTORTION);

    DUMP_FEATURE(handle, compID, TY_INT_LASER_POWER);
    DUMP_FEATURE(handle, compID, TY_BOOL_LASER_AUTO_CTRL);

    DUMP_FEATURE(handle, compID, TY_INT_R_GAIN);
    DUMP_FEATURE(handle, compID, TY_INT_G_GAIN);
    DUMP_FEATURE(handle, compID, TY_INT_B_GAIN);
}

#define DUMP_COMPONENT(handle,compIds,id) do{\
                                                if(compIds & id){\
                                                    LOGD("===  %s:",#id);\
                                                    dumpComponentFeatures(handle, id);\
                                                  }\
                                            }while(0)

void dumpAllComponentFeatures(TY_DEV_HANDLE handle, int32_t compIDs)
{
    LOGD("=== Dump all components and features:");
    DUMP_COMPONENT(handle, compIDs, TY_COMPONENT_DEVICE);
    DUMP_COMPONENT(handle, compIDs, TY_COMPONENT_DEPTH_CAM);
    DUMP_COMPONENT(handle, compIDs, TY_COMPONENT_POINT3D_CAM);
    DUMP_COMPONENT(handle, compIDs, TY_COMPONENT_IR_CAM_LEFT);
    DUMP_COMPONENT(handle, compIDs, TY_COMPONENT_IR_CAM_RIGHT);
    DUMP_COMPONENT(handle, compIDs, TY_COMPONENT_RGB_CAM_LEFT);
    DUMP_COMPONENT(handle, compIDs, TY_COMPONENT_RGB_CAM_RIGHT);
    DUMP_COMPONENT(handle, compIDs, TY_COMPONENT_LASER);
    DUMP_COMPONENT(handle, compIDs, TY_COMPONENT_IMU);
}

int main(int argc, char* argv[])
{
    const char* IP = NULL;
    const char* ID = NULL;
    TY_DEV_HANDLE handle;

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
    
    // Init lib
    ASSERT_OK(TYInitLib());
    TY_VERSION_INFO* pVer = (TY_VERSION_INFO*)buffer;
    ASSERT_OK( TYLibVersion(pVer) );
    LOGD("=== lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch);

    TY_DEVICE_BASE_INFO* pBaseInfo = (TY_DEVICE_BASE_INFO*)buffer;
    if(IP) {
        LOGD("=== Open device %s", IP);
        ASSERT_OK( TYOpenDeviceWithIP(IP, &handle) );

        ASSERT_OK( TYGetDeviceInfo(handle, pBaseInfo) );
        LOGD("===   device %s:", IP);
        LOGD("===       interface  : %d", pBaseInfo[0].devInterface);
        LOGD("===       id         : %s", pBaseInfo[0].id);
        LOGD("===       vendor     : %s", pBaseInfo[0].vendorName);
        LOGD("===       model      : %s", pBaseInfo[0].modelName);
        LOGD("===       HW version : %d.%d.%d"
                , pBaseInfo[0].hardwareVersion.major
                , pBaseInfo[0].hardwareVersion.minor
                , pBaseInfo[0].hardwareVersion.patch
                );
        LOGD("===       FW version : %d.%d.%d"
                , pBaseInfo[0].firmwareVersion.major
                , pBaseInfo[0].firmwareVersion.minor
                , pBaseInfo[0].firmwareVersion.patch
                );

    } else {
        if(ID == NULL){
            // Get device info
            ASSERT_OK(TYGetDeviceNumber(&n));
            LOGD("=== device number %d", n);

            ASSERT_OK(TYGetDeviceList(pBaseInfo, 100, &n));

            if(n == 0){
                LOGD("=== No device got");
                return -1;
            }
            ID = pBaseInfo[0].id;
        }

        LOGD("=== get device list %d:", n);
        for(int i = 0; i < n; i++){
            LOGD("===   device %d:", i);
            LOGD("===       interface  : %d", pBaseInfo[i].devInterface);
            LOGD("===       id         : %s", pBaseInfo[i].id);
            LOGD("===       vendor     : %s", pBaseInfo[i].vendorName);
            LOGD("===       model      : %s", pBaseInfo[i].modelName);
            LOGD("===       HW version : %d.%d.%d"
                    , pBaseInfo[i].hardwareVersion.major
                    , pBaseInfo[i].hardwareVersion.minor
                    , pBaseInfo[i].hardwareVersion.patch
                    );
            LOGD("===       FW version : %d.%d.%d"
                    , pBaseInfo[i].firmwareVersion.major
                    , pBaseInfo[i].firmwareVersion.minor
                    , pBaseInfo[i].firmwareVersion.patch
                    );
        }

        LOGD("=== Open device: %s", ID);
        ASSERT_OK(TYOpenDevice(ID, &handle));
    }

    // List all components
    int32_t compIDs;
    std::string compNames;
    ASSERT_OK(TYGetComponentIDs(handle, &compIDs));
    dumpAllComponentFeatures(handle, compIDs);

    printf("Done!\n");
    TYDeinitLib();
    return 0;
}
