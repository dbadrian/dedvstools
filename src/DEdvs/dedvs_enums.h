#ifndef DEDVS_ENUMS_H
#define DEDVS_ENUMS_H

namespace dedvs
{
  enum DEdvsStatus {
    DEDVS_STATUS_OK             = 0,
    DEDVS_STATUS_ERROR          = 1,
    DEDVS_STATUS_BAD_PARAMETER  = 2,
    DEDVS_STATUS_DEVICE_ERROR   = 3,
    DEDVS_STATUS_STREAM_ERROR   = 4,
    DEDVS_STATUS_HW_ERROR       = 5
  };

  enum OpenNIStreamType {
    DEPTH,
    COLOR
    //IR, but we ignore that for now
  };
}

#endif