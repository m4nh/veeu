#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <map>
#include <string>

//SPDLOG
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "spdlog/sinks/basic_file_sink.h"

#define DEFAULT_LOG_LEVEL "debug"

namespace veeu
{

class Logger
{  
  private:
    //static std::map<std::string, std::shared_ptr<spdlog::logger>> loggers;

  public:
    static auto getSimpleLogger(std::string name = "default")
    {
        static std::map<std::string, std::shared_ptr<spdlog::logger>> loggers;
        if (loggers.count(name) == 0)
        {
            loggers[name] = spdlog::stdout_color_mt(name);
        }

        loggers[name]->set_level(spdlog::level::from_str(DEFAULT_LOG_LEVEL));
        return loggers[name];
    }

    static void enableLogger(std::string name = "default", bool enabled = true)
    {

        auto logger = getSimpleLogger(name);
        if (enabled)
        {
            logger->set_level(spdlog::level::from_str("off"));
        }
    }
};

} // namespace atlas

#endif
