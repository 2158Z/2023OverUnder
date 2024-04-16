#include "logger/logger.hpp"
std::shared_ptr<InfoSink> infoSink() {
    static std::shared_ptr<InfoSink> infoSink = std::make_shared<InfoSink>();
    return infoSink;
}