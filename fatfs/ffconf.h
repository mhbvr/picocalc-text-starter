#define FFCONF_DEF  80386   /* Must match FF_DEFINED in ff.h */

#define FF_FS_READONLY   0
#define FF_FS_MINIMIZE   0
#define FF_USE_STRFUNC   0
#define FF_USE_FIND      0
#define FF_USE_MKFS      0
#define FF_USE_FASTSEEK  0
#define FF_USE_EXPAND    0
#define FF_USE_CHMOD     0
#define FF_USE_LABEL     1   // f_getlabel
#define FF_USE_FORWARD   0
#define FF_USE_LFN       1   // stack allocation
#define FF_MAX_LFN       255
#define FF_LFN_UNICODE   0   // ANSI/OEM (char)
#define FF_LFN_BUF       255
#define FF_SFN_BUF       12
#define FF_FS_RPATH      2   // relative path + f_getcwd
#define FF_VOLUMES       1
#define FF_STR_VOLUME_ID 0
#define FF_MULTI_PARTITION 0
#define FF_MIN_SS        512
#define FF_MAX_SS        512
#define FF_LBA64         0
#define FF_FS_EXFAT      0
#define FF_FS_NORTC      1
#define FF_NORTC_MON     1
#define FF_NORTC_MDAY    1
#define FF_NORTC_YEAR    2025
#define FF_FS_LOCK       0
#define FF_FS_REENTRANT  0
#define FF_CODE_PAGE     437
