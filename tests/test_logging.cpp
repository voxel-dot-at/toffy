/*
   Copyright 2023 Simon Vogl <svogl@voxel.at>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include <iostream>

#include <toffy/logging.hpp>

using namespace std;
using namespace toffy;

int local()
{
    log::Log logger("TEST");

    logger.error << " --------------- D"
                 << "X";
    logger.setLevel(log::debug);
    logger.debug << 123 << "abc d";
    logger.info << 123 << "abc i";
    logger.warning << 123 << "abc w";
    logger.error << 123 << "abc e";

    logger.error << " --------------- I";
    logger.setLevel(log::info);
    logger.debug << 123 << "abc d";
    logger.info << 123 << "abc i";
    logger.warning << 123 << "abc w";
    logger.error << 123 << "abc e";

    logger.error << " --------------- W";
    logger.setLevel(log::warning);
    logger.debug << 123 << "abc d";
    logger.info << 123 << "abc i";
    logger.warning << 123 << "abc w";
    logger.error << 123 << "abc e";

    logger.error << " --------------- E";
    logger.setLevel(log::error);
    logger.debug << 123 << "abc d";
    logger.info << 123 << "abc i";
    logger.warning << 123 << "abc w";
    logger.error << 123 << "abc e";
    return 0;
}

int glbl()
{
    toffy::log::theLogger->setLevel(toffy::log::debug);
    LOGD << "glbl d";
    LOGI << "glbl i";
    LOGW << "glbl w";
    LOGE << "glbl e";
    toffy::log::theLogger->setLevel(toffy::log::info);
    LOGD << "glbl d";
    LOGI << "glbl i";
    LOGW << "glbl w";
    LOGE << "glbl e";
    toffy::log::theLogger->setLevel(toffy::log::warning);
    LOGD << "glbl d";
    LOGI << "glbl i";
    LOGW << "glbl w";
    LOGE << "glbl e";
    toffy::log::theLogger->setLevel(toffy::log::error);
    LOGD << "glbl d";
    LOGI << "glbl i";
    LOGW << "glbl w";
    LOGE << "glbl e";
    return 0;
}

void multiLine()
{
    toffy::log::theLogger->setLevel(toffy::log::debug);
    LOGD << "multi" << toffy::log::endl << "line";
}

void doLog()
{
    // class Flt
    // {
    //    public:
    //     std::string id() { return "FLT"; }

    //     void test()
    //     {
    //         LOG(toffy::log::debug) << "log "
    //                    << "debug";
    //     }
    // };
    // Flt flt;
    // flt.test();
}
int main()
{
    local();
    glbl();
    doLog();
    multiLine();

    cout << "main() fin." << endl;
    return 0;
}
