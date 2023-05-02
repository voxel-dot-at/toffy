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

#include <toffy/player.hpp>
#include <toffy/base/cond.hpp>



using namespace std;
using namespace toffy;

int main(int argc, char** argv)
{
    std::string file = "tests/xml/cond_empty.xml";
    toffy::Player* player;
    player = new toffy::Player(boost::log::trivial::debug, false);
    FilterBank fb;

    if (argc >= 2) {
        file = argv[1];
    }

    cout << "LOADING" << endl;
    // player->loadConfig(file);
    fb.loadFileConfig(file);
    cout << "LOADED" << endl;
    // player->runOnce();

    cout << "main() fin." << endl;
}
