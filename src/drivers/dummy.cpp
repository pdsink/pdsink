#include "dummy.h"
#include "sink.h"

namespace pd {

DummyDriver::DummyDriver(Sink& sink) : sink{sink} {
    sink.driver = this;
};

} // namespace pd