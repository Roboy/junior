/*Copyright (c) 2014, University of Zurich, Department of Informatics, Artificial Intelligence Laboratory
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors 
   may be used to endorse or promote products derived from this software without 
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef ROBOY_ROBOYLOGGER_H
#define ROBOY_ROBOYLOGGER_H

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

#define ROBOY_LOG_FATAL(msg) LOG4CPLUS_FATAL(RoboyLogger::getInstance().getLogger(), msg)

#define ROBOY_LOG_ERROR(msg) LOG4CPLUS_ERROR(RoboyLogger::getInstance().getLogger(), msg)
#define ROBOY_LOG_WARN(msg)  LOG4CPLUS_WARN (RoboyLogger::getInstance().getLogger(), msg)
#define ROBOY_LOG_INFO(msg)  LOG4CPLUS_INFO (RoboyLogger::getInstance().getLogger(), msg)
#define ROBOY_LOG_DEBUG(msg) LOG4CPLUS_DEBUG(RoboyLogger::getInstance().getLogger(), msg)

// Singleton pattern ensures that the logger is always configured properly
// before used.

class RoboyLogger
{
    public:
        static RoboyLogger &getInstance();
        log4cplus::Logger getLogger();
    private:
        RoboyLogger();
        RoboyLogger(const RoboyLogger &); // Do not remove this definition (singleton)
        RoboyLogger &operator=(RoboyLogger const &); // dito
        log4cplus::Logger l;
};
#endif
