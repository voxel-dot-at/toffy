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
#pragma once

#include <boost/any.hpp>



namespace toffy {

class Filter;

/**
 * @brief The Event class allows to execute a filter inside another filter
 * @ingroup Core
 *
 * The events are defined as actions lunched from one filter to be executed in
 * another filter/s during the process time of the sender.
 *
 * The target filter or group of filters of the same type, should have declare
 * the event to be executed.
 *
 * @todo Implement the event logic (config, run) in Filter. Now there is only
 * implemented locally in the exposure filter.
 *
 * An example of the event definition in a config file could be:
 * @include event.xml
 *
 *
 *
 */
class Event {
public:
    /**
     * @brief Defines if you want a single filter to execute the Event or all
     * filters of one type.
     */
    enum ReceiverType {
	FILTER,
	FILTER_TYPE
    };

    /**
     * @brief Event
     */
    Event(){}

    /**
     * @brief Constructor with all elements
     * @param sender This is the filter that emits the event
     * @param type ReceiverType
     * @param receiver The id of the target if ReceiverType::FILTER or the
     *	type if ReceiverType::FILTER_TYPE
     * @param event The type of event to be execute. The receiver should
     * know it.
     *
     *
     */
    Event(Filter *sender, ReceiverType type, std::string receiver,
	  std::string event);

    virtual ~Event(){}

    /**
     * @brief Getter receiver
     * @return
     */
    std::string receiver() const {return _receiver_id;}

    /**
     * @brief Getter receiverType
     * @return
     */
    ReceiverType receiverType() {return _re_type;}

    /**
     * @brief Getter event
     * @return
     */
    std::string event() const {return _event_id;}

    /**
     * @brief Setter data
     * @param new_data
     */
    void data(boost::any new_data);

    /**
     * @brief Getter data
     * @return
     */
    boost::any data() const {return *_data;}

private:
    std::string _sender_id, ///< Id of the Filter that emits the event
	_receiver_id, ///< Id or type of the filters to execute the events
	_event_id; ///< Event to be execute in the receiver/s
    Filter * _sender; ///< Pointer to the sender
    std::shared_ptr<boost::any> _data; /**< Data to be past to the receiver/s.
	Together with the event_it, they should known to interpret the data
	content */

    ReceiverType _re_type; ///< Defines _receiver_id
};

}
