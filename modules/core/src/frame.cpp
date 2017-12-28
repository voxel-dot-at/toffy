#include "toffy/frame.hpp"
#include <boost/log/trivial.hpp>

using namespace toffy;

Frame::Frame() {}
Frame::Frame(const Frame& f): data(f.data) {}

Frame::~Frame() {
    data.clear();
}

bool Frame::hasKey(std::string key) const {
    if(data.find(key) != data.end())
        return true;
    return false;
}

boost::any Frame::getData(const std::string& key) const {
    //BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << ", key: " << key;
    boost::any out;
    if (data.find(key) != data.end()) {
        try  {
            out = data.at(key);
        } catch (std::out_of_range& e) {
            BOOST_LOG_TRIVIAL(warning) << "Not found key " << key << " in frame";
        }
    } else
        BOOST_LOG_TRIVIAL(warning) << "Not found key " << key << " in frame";
    return out;
}

void toffy::Frame::addData(std::string key, boost::any in) {
    data[key] = in;//.insert(std::pair<std::string, boost::any>(key,in));
    return;
}

bool toffy::Frame::removeData(std::string key) {

    if(data.find(key) != data.end()) {
        data.erase(data.find(key));
        return true;
    } else
        return false;
}
void Frame::clearData() {
    return data.clear();
}

/*
std::vector<std::string> Frame::keys() const
{
    std::vector<std::string> k(data.size());
    boost::container::flat_map< std::string, boost::any >::const_iterator it = data.begin();
    while (it!= data.end()) {
    k.push_back(it->first);
    it++;
    }
    return k;
}
*/
