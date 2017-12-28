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
