#pragma once
#include <string>
#include <vector>

#include "toffy/filter.hpp"

namespace toffy
{
    /**
     * @brief class Mux multiplexes multiple input frames and creates one
     * output frame.
     * @ingroup Core
     *
     */
    class Mux: public Filter
    {
    public:
	/**
	 * @brief Mux
	 * @param name
	 *
	 */
	Mux(const std::string& name);

	virtual ~Mux();

	/**
	 * @brief The Filter::filter() is overwrite to treat a single input
	 * frame as a vector of frames with a single element.
	 * @param in
	 * @param out
	 * @return result of Mux::filter(const std::vector<Frame*>& in,
	 *				Frame& out)
	 */
	virtual bool filter(const Frame& in, Frame& out);

	//virtual int loadConfig(const boost::property_tree::ptree& pt)=0;
	//virtual void updateConfig(const boost::property_tree::ptree &pt)=0;

	/**
	 * @brief Extension of the Filter::filter() method to work with a group
	 * of input frames and merge all in a single out Frame.
	 * @param in std::vector<Frame*>&
	 * @param out
	 * @return True on success, False if failed.
	 */
	virtual bool filter(const std::vector<Frame*>& in, Frame& out) = 0;

    protected:
    };
}

