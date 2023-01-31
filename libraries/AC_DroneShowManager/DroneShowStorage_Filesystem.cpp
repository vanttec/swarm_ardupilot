#include <AP_Filesystem/AP_Filesystem_Available.h>

#if HAVE_FILESYSTEM_SUPPORT

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>
#include <sys/stat.h>

#include "DroneShowStorage.h"

#ifndef HAL_BOARD_COLLMOT_DIRECTORY
#  if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#    define HAL_BOARD_COLLMOT_DIRECTORY "./collmot"
#  else
#    define HAL_BOARD_COLLMOT_DIRECTORY "/COLLMOT"
#  endif
#endif

#define SHOW_FILE (HAL_BOARD_COLLMOT_DIRECTORY "/show.skyb")

extern const AP_HAL::HAL &hal;

void DroneShowStorage::init()
{
    // AP::FS().mkdir() apparently needs lots of free memory, see:
    // https://github.com/ArduPilot/ardupilot/issues/16103
    EXPECT_DELAY_MS(3000);

    if (AP::FS().mkdir(HAL_BOARD_COLLMOT_DIRECTORY) < 0) {
        if (errno == EEXIST) {
            // Directory already exists, this is okay
        } else {
            hal.console->printf(
                "Failed to create directory %s: %s (code %d)\n",
                 HAL_BOARD_COLLMOT_DIRECTORY, strerror(errno), errno
            );
        }
    }
}

bool DroneShowStorage::load_show_file(uint8_t** ptr, size_t* length)
{
    int retval, fd;
    ssize_t to_read, actually_read;
    struct stat stat_data;
    uint8_t *show_data, *write_ptr, *end_ptr;

    // Check whether the show file exists
    retval = AP::FS().stat(SHOW_FILE, &stat_data);
    if (retval)
    {
        // Show file does not exist. This basically means that the operation
        // was successful, and the pointer should point nowhere.
        *ptr = NULL;
        *length = 0;
        return true;
    }

    // Ensure that we have a sensible block size that we will use when reading
    // the show file
    if (stat_data.st_blksize < 1)
    {
        stat_data.st_blksize = 4096;
    }

    // Allocate memory for the whole content of the file
    show_data = static_cast<uint8_t *>(calloc(stat_data.st_size, sizeof(uint8_t)));
    if (show_data == 0)
    {
        hal.console->printf(
            "Show file too large: %ld bytes\n",
            static_cast<long int>(stat_data.st_size));
        return false;
    }

    // Read the entire show file into memory
    fd = AP::FS().open(SHOW_FILE, O_RDONLY);
    if (fd < 0)
    {
        free(show_data);
        show_data = write_ptr = end_ptr = 0;
    }
    else
    {
        write_ptr = show_data;
        end_ptr = show_data + stat_data.st_size;
    }

    while (write_ptr < end_ptr)
    {
        to_read = end_ptr - write_ptr;
        if (to_read > stat_data.st_blksize)
        {
            to_read = stat_data.st_blksize;
        }

        if (to_read == 0)
        {
            break;
        }

        actually_read = AP::FS().read(fd, write_ptr, to_read);
        if (actually_read < 0)
        {
            /* Error while reading */
            hal.console->printf(
                "IO error while reading show file near byte %ld, errno = %d\n",
                static_cast<long int>(write_ptr - show_data),
                static_cast<int>(errno)
            );
            free(show_data);
            show_data = 0;
            break;
        }
        else if (actually_read == 0)
        {
            /* EOF */
            break;
        }
        else
        {
            write_ptr += actually_read;
        }
    }

    if (fd > 0)
    {
        AP::FS().close(fd);
    }

    if (show_data)
    {
        *ptr = show_data;
        *length = stat_data.st_size;
        return true;
    }
    else
    {
        return false;
    }
}

bool DroneShowStorage::remove_show_file()
{
    if (AP::FS().unlink(SHOW_FILE)) {
        // Error while removing the file; did it exist?
        if (errno == ENOENT) {
            // File was missing already, this is OK.
        } else {
            // This is a genuine failure
            return false;
        }
    }

    return true;
}

#endif