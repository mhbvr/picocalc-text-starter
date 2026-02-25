//
// clib.c - Interface to the C standard library functions for PicoCalc
//
// Provides file operations using FatFS.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "pico/stdlib.h"
#include "../fatfs/ff.h"

#define FD_FLAG_MASK    0x4000
#define MAX_OPEN_FILES  16

static FIL  files[MAX_OPEN_FILES];
static bool file_open[MAX_OPEN_FILES];
static int  initialized = 0;

static void init(void)
{
    if (!initialized) {
        for (int i = 0; i < MAX_OPEN_FILES; i++) file_open[i] = false;
        initialized = 1;
    }
}

static int fresult_to_errno(FRESULT r)
{
    switch (r) {
    case FR_OK:              return 0;
    case FR_NO_FILE:
    case FR_NO_PATH:         return ENOENT;
    case FR_EXIST:           return EEXIST;
    case FR_NOT_ENOUGH_CORE:
    case FR_DISK_ERR:
    case FR_INT_ERR:         return EIO;
    case FR_NOT_READY:       return ENODEV;
    case FR_WRITE_PROTECTED: return EACCES;
    case FR_INVALID_NAME:
    case FR_INVALID_PARAMETER: return EINVAL;
    case FR_DENIED:          return EACCES;
    default:                 return EIO;
    }
}

int _open(const char *filename, int oflag, ...)
{
    init();

    for (int i = 0; i < MAX_OPEN_FILES; i++) {
        if (!file_open[i]) {
            BYTE fa = 0;
            if ((oflag & O_ACCMODE) == O_RDONLY || (oflag & O_ACCMODE) == O_RDWR)
                fa |= FA_READ;
            if ((oflag & O_ACCMODE) == O_WRONLY || (oflag & O_ACCMODE) == O_RDWR)
                fa |= FA_WRITE;

            if (oflag & O_CREAT) {
                if (oflag & O_EXCL)
                    fa |= FA_CREATE_NEW;
                else if (oflag & O_TRUNC)
                    fa |= FA_CREATE_ALWAYS;
                else
                    fa |= FA_OPEN_ALWAYS;
            } else {
                fa |= FA_OPEN_EXISTING;
            }

            if (oflag & O_APPEND)
                fa = (fa & ~FA_OPEN_EXISTING) | FA_OPEN_APPEND;

            FRESULT res = f_open(&files[i], filename, fa);
            if (res != FR_OK) {
                errno = fresult_to_errno(res);
                return -1;
            }
            file_open[i] = true;
            return i | FD_FLAG_MASK;
        }
    }
    errno = EMFILE;
    return -1;
}

int _close(int fd)
{
    if ((fd & FD_FLAG_MASK) == 0) { errno = EBADF; return -1; }
    fd &= ~FD_FLAG_MASK;
    if (fd < 0 || fd >= MAX_OPEN_FILES || !file_open[fd]) { errno = EBADF; return -1; }

    FRESULT res = f_close(&files[fd]);
    file_open[fd] = false;
    if (res != FR_OK) { errno = fresult_to_errno(res); return -1; }
    return 0;
}

off_t _lseek(int fd, off_t offset, int whence)
{
    if ((fd & FD_FLAG_MASK) == 0) { errno = EBADF; return -1; }
    fd &= ~FD_FLAG_MASK;
    if (fd < 0 || fd >= MAX_OPEN_FILES || !file_open[fd]) { errno = EBADF; return -1; }

    FIL *fp = &files[fd];
    FSIZE_t new_pos;
    if (whence == SEEK_SET)       new_pos = (FSIZE_t)offset;
    else if (whence == SEEK_CUR)  new_pos = f_tell(fp) + (FSIZE_t)offset;
    else /* SEEK_END */           new_pos = f_size(fp) + (FSIZE_t)offset;

    FRESULT res = f_lseek(fp, new_pos);
    if (res != FR_OK) { errno = fresult_to_errno(res); return -1; }
    return (off_t)f_tell(fp);
}

int _read(int fd, char *buffer, int length)
{
    if (fd == 0) {
        return stdio_get_until(buffer, length, at_the_end_of_time);
    }

    if ((fd & FD_FLAG_MASK) == 0) { errno = EBADF; return -1; }
    fd &= ~FD_FLAG_MASK;
    if (fd < 0 || fd >= MAX_OPEN_FILES || !file_open[fd]) { errno = EBADF; return -1; }

    UINT br = 0;
    FRESULT res = f_read(&files[fd], buffer, (UINT)length, &br);
    if (res != FR_OK) { errno = fresult_to_errno(res); return -1; }
    return (int)br;
}

int _write(int fd, const char *buffer, int length)
{
    if (fd == 1 || fd == 2) {
        stdio_put_string(buffer, length, false, true);
        return length;
    }

    if (length == 0) return 0;

    if ((fd & FD_FLAG_MASK) == 0) { errno = EBADF; return -1; }
    fd &= ~FD_FLAG_MASK;
    if (fd < 0 || fd >= MAX_OPEN_FILES || !file_open[fd]) { errno = EBADF; return -1; }

    UINT bw = 0;
    FRESULT res = f_write(&files[fd], buffer, (UINT)length, &bw);
    if (res != FR_OK) { errno = fresult_to_errno(res); return -1; }
    if (bw == 0) { errno = EIO; return -1; }
    return (int)bw;
}

int _fstat(int fd, struct stat *buf)
{
    if ((fd & FD_FLAG_MASK) == 0) { errno = EBADF; return -1; }
    fd &= ~FD_FLAG_MASK;
    if (fd < 0 || fd >= MAX_OPEN_FILES || !file_open[fd]) { errno = EBADF; return -1; }

    FIL *fp = &files[fd];
    buf->st_size  = (off_t)f_size(fp);
    buf->st_mode  = S_IFREG | S_IRUSR | S_IWUSR;
    buf->st_nlink = 1;
    buf->st_uid   = 0;
    buf->st_gid   = 0;
    buf->st_atime = 0;
    buf->st_mtime = 0;
    buf->st_ctime = 0;
    buf->st_ino   = 0;
    return 0;
}

int _stat(const char *path, struct stat *buf)
{
    FILINFO fi;
    FRESULT res = f_stat(path, &fi);
    if (res != FR_OK) { errno = fresult_to_errno(res); return -1; }

    buf->st_size  = (off_t)fi.fsize;
    if (fi.fattrib & AM_DIR)
        buf->st_mode = S_IFDIR | S_IRUSR | S_IWUSR | S_IXUSR;
    else
        buf->st_mode = S_IFREG | S_IRUSR | S_IWUSR;
    buf->st_nlink = 1;
    buf->st_uid   = 0;
    buf->st_gid   = 0;
    buf->st_atime = 0;
    buf->st_mtime = 0;
    buf->st_ctime = 0;
    buf->st_ino   = 0;
    return 0;
}

int _link(const char *oldpath, const char *newpath)
{
    (void)oldpath; (void)newpath;
    return -1; // not supported on FAT
}

int _unlink(const char *filename)
{
    FRESULT res = f_unlink(filename);
    if (res != FR_OK) { errno = fresult_to_errno(res); return -1; }
    return 0;
}

int rename(const char *oldpath, const char *newpath)
{
    FRESULT res = f_rename(oldpath, newpath);
    if (res != FR_OK) { errno = fresult_to_errno(res); return -1; }
    return 0;
}
