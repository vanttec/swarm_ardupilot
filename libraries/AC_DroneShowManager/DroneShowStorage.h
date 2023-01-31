#pragma once

#include <AP_Common/AP_Common.h>

/// @file   DroneShowStorage.h
/// @brief  Abstraction layer for the storage mechanism of show data

class DroneShowStorage
{
public:
    DroneShowStorage() {};
    virtual ~DroneShowStorage() {};

    /**
     * Initializes the storage layer early during boot.
     */
    void init();

    /**
     * Loads the stored show data into a newly allocated memory block.
     *
     * @param  ptr  the pointer to the newly allocated memory block will be
     *              returned here. May be NULL upon successful exit if the
     *              storage backend does not contain show data.
     * @param  length  the number of allocated bytes will be returned here
     * @return whether the operation was successful
     */
    bool load_show_file(uint8_t** ptr, size_t* length) WARN_IF_UNUSED;

    /**
     * Removes the stored show data from the storage.
     *
     * @return whether the removal succeeded
     */
    bool remove_show_file() WARN_IF_UNUSED;
};
