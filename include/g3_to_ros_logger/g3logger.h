#pragma once

#include <g3log/logworker.hpp>

#include <ros/console.h>

namespace libg3logger {

  template <class T>
  struct G3Logger {


    G3Logger( const std::string &appName, const LEVELS defaultLevel = WARNING )
      : worker( g3::LogWorker::createLogWorker() ),
        sinkHandle( worker->addSink(std::unique_ptr<T>( new T ), &T::ReceiveLogMessage) )
    {
      auto handle = worker->addDefaultLogger(appName, ".");
      setLevel( defaultLevel );

      g3::initializeLogging(worker.get());
      std::future<std::string> log_file_name = handle->call(&g3::FileSink::fileName);

      // This should be the only message written explicitly to std::cout
      // Everything else gets sent to the logger
      ROS_INFO_STREAM( "*   Log file: [" << log_file_name.get() << "] *");
    }

    void logBanner( void ) {
      LOG(INFO) << "Starting log.";

      #ifdef ENABLE_SSE
        LOG(INFO) << "With SSE optimizations.";
      #elif ENABLE_NEON
        LOG(INFO) << "With NEON optimizations.";
      #endif

    }

    void verbose( bool ) {
       sinkHandle->call( &T::setThreshold, DEBUG );
    }

    void setLevel( const LEVELS level ) {
       sinkHandle->call( &T::setThreshold, level );
    }


    std::unique_ptr<g3::LogWorker> worker;
    std::unique_ptr<g3::SinkHandle<T>> sinkHandle;
  };


}
