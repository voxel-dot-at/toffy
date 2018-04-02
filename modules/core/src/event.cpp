/*
   Copyright 2018 Simon Vogl <svogl@voxel.at>
                  Angel Merino-Sastre <amerino@voxel.at>

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
#include <boost/log/trivial.hpp>
#include "toffy/event.hpp"
#include "toffy/filter.hpp"
#include <boost/make_shared.hpp>

using namespace toffy;
namespace logging = boost::log;

Event::Event(Filter *sender, ReceiverType type, std::string receiver,
      std::string event): _sender_id(sender->name()),
	_receiver_id(receiver),
	_event_id(event),
	_sender(sender),
	_re_type(type) {}

void Event::data(boost::any new_data) {
    _data = boost::make_shared<boost::any>(new_data);
}
