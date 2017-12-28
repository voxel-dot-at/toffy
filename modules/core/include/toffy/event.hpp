#pragma once

#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>


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
    boost::shared_ptr<boost::any> _data; /**< Data to be past to the receiver/s.
	Together with the event_it, they should known to interpret the data
	content */

    ReceiverType _re_type; ///< Defines _receiver_id
};

}
