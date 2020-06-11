#ifndef TRJ_CONFIG_H
#define TRJ_CONFIG_H


#define DEFALUT_VV_LIMIT     (2.0)
#define DEFAULT_HV_LIMIT     (2.0)
#define DEFAULT_AV_LIMIT     (2.5)

#define DEFALUT_GeoFence_Xmin     (-5.0)
#define DEFALUT_GeoFence_Xmax     (5.0)
#define DEFALUT_GeoFence_Ymin     (-5.0)
#define DEFALUT_GeoFence_Ymax     (5.0)
#define DEFALUT_GeoFence_Zmin     (-5.0)
#define DEFALUT_GeoFence_Zmax     (5.0)

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

#define CIRCLE_TRJ_FACING_CENTER       (0)
#define CIRCLE_TRJ_FACING_PATH_FORWARD (1)
#define CIRCLE_TRJ_FACING_FIXED        (2)
//#define CIRCLE_TRJ_CLOCKWISE           (0)
//#define CIRCLE_TRJ_ANTICLOCKWISE       (1)

#endif // TRJ_CONFIG_H
