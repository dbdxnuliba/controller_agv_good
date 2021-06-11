#include <mutex>
#include "common/common.h"
#include "common/data_types.h"
#include "common/geometry.h"

namespace bz_robot
{


class Localization
{
public:
    Localization() {}

    bool set_location(const Msg<Pose<FLOAT_T>> &msg_location)
    {
        std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
        m_msg_location = msg_location;
    }

    const Msg<Pose<FLOAT_T>>& location()
    {
        std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
        return m_msg_location;
    }

private:
        std::recursive_mutex m_mtx;
        Msg<Pose<FLOAT_T>> m_msg_location;
};

}
