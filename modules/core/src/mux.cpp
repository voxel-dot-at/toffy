#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>

#include "toffy/mux.hpp"


using namespace toffy;



Mux::Mux(const std::string& name) : Filter(name) {
}

Mux::~Mux() {
}

bool Mux::filter(const Frame& in, Frame& out)
{
	std::vector<Frame*> l;
	l.push_back((Frame*)&in);
	return filter(l,out);
}
