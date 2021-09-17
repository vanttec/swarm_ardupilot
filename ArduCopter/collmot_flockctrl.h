#pragma once

#include <AP_Param/AP_Param.h>

#if COLLMOT_EXTENSIONS_ENABLED == ENABLED

class CollMotFlockCtrl {

public:

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // constructor
    CollMotFlockCtrl();

    /* Do not allow copies */
    CollMotFlockCtrl(const CollMotFlockCtrl &other) = delete;
    CollMotFlockCtrl &operator=(const CollMotFlockCtrl&) = delete;

private:

    // Enum specifying the meaning of individual bits in the CM_FLOCK_OPTS
    // parameter
    enum class Options : int32_t {
        All = (1 << 0U)
    };

    // Placeholder for the value of the CM_FLOCK_OPTS parameter
    AP_Int32 _options;

    bool isOptionSet(Options option) {
        return _options & (
            static_cast<int32_t>(Options::All) |
            static_cast<int32_t>(option)
        );
    }
};

#endif
