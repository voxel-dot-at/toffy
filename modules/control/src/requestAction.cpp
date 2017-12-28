
#include <iostream>
#include <map>

#include <boost/log/trivial.hpp>
#include <uri/grammar.hpp>

#include <toffy/web/requestAction.hpp>


#define BOOST_SPIRIT_DEBUG
#define BOOST_SPIRIT_QI_DEBUG

using namespace std;
namespace qi = boost::spirit::qi;

namespace toffy {
  namespace control {
    namespace uriparser {

      template <typename Iterator>
      struct key_grammar : boost::spirit::qi::grammar<Iterator, std::string()> {

        key_grammar(): key_grammar::base_type(key) {
          key
              =   +(unreserved
                    |   pct_encoded
                    |   sub_delims
                    |   qi::char_(":@"))
              ;
        }

        qi::rule<Iterator, std::string()> key;
        uri::sub_delims_grammar<Iterator> sub_delims;
        uri:: pct_encoded_grammar<Iterator> pct_encoded;
        uri::unreserved_grammar<Iterator> unreserved;
      };


      typedef std::vector<std::string> vertorStr;
      template <typename Iterator>
      struct path_values : qi::grammar<Iterator, vertorStr() > {
        path_values(): path_values::base_type(path) {

          path
              = '/' >> -(+key >> *('/' >> *key))
                       ;
          BOOST_SPIRIT_DEBUG_NODES((path) (key));
        }

        qi::rule<Iterator, vertorStr() > path;
        key_grammar<Iterator> key;
      };

      typedef std::pair<std::string, boost::optional<std::string> > pair_type;
      typedef boost::container::flat_map< std::string, boost::optional<std::string> > pairs_type;

      template <typename Iterator>
      struct query_values : qi::grammar<Iterator, pairs_type()> {
        query_values() : query_values::base_type(query) {
          query =  pair >> *((qi::lit('&')) >> pair);
          pair  =  key >> -('=' >> -value);
          key   =  qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z_0-9");
          value = +qi::char_("a-zA-Z_0-9./\"{}:%") ;
          BOOST_SPIRIT_DEBUG_NODE(query);
        }

        qi::rule<Iterator, pairs_type()> query;
        qi::rule<Iterator, pair_type()> pair;
        qi::rule<Iterator, std::string()> value, key;
        //key_grammar<Iterator> key;
      };



    } // namespace uriparser
  } // namespace control
} // namespace l3vel

using namespace toffy;
using namespace toffy::control;
Action::Action(const http::server::request &request) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
  //Get the path.
  url_decode(request.uri);

  req = &request;
  //else
  //	content_decode(request.content);
}

/*bool Action::content_decode(std::string in) {
  return true;
}*/

bool Action::url_decode(std::string in) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
  BOOST_LOG_TRIVIAL(debug) << in;

  iterator_type begin = in.begin();
  uri::components<iterator_type> c;
  uri::grammar<iterator_type> g(c);

  if ( begin != in.end() &&
      !qi::parse(begin, (iterator_type)in.end(), g)) {
      BOOST_LOG_TRIVIAL(warning) << "Parsing uri_grammar failed";
      return EXIT_FAILURE;
    }

  BOOST_LOG_TRIVIAL(debug)
      << "Parsing uri_grammar succeeded, found entries:";
  BOOST_LOG_TRIVIAL(debug)
      << "-path: "
      << std::string(c.path.begin(), c.path.end());
  BOOST_LOG_TRIVIAL(debug)
      << "-query: "
      << std::string(c.query.begin(), c.query.end());
  path_parse( c.path.begin(), c.path.end());
  if ( c.query.begin() != c.query.end() )
  params_parse( c.query.begin(), c.query.end() );
  return true;
}

bool Action::path_parse( iterator_type begin, iterator_type const &end) {

  uriparser::path_values<std::string::const_iterator> p;
  BOOST_LOG_TRIVIAL(debug) << string(begin, end);;
  if (!qi::parse(begin, end, p, _items)) {
      BOOST_LOG_TRIVIAL(warning) << "Parsing path failed";
      return EXIT_FAILURE;
    }
  BOOST_LOG_TRIVIAL(debug) << "Parsing path succeeded, found entries:";
  for (uriparser::vertorStr::iterator it = _items.begin(); it != _items.end(); ++it)
    BOOST_LOG_TRIVIAL(debug) << *it;
  return true;
}

bool Action::params_parse( iterator_type begin, const iterator_type &end) {
  uriparser::query_values<std::string::const_iterator> q;
  //begin = c.query.begin();
  if ( !qi::parse(begin, end, q, _parameters)) {

      BOOST_LOG_TRIVIAL(warning) << "Parsing params failed here";
      return EXIT_FAILURE;
    }
  BOOST_LOG_TRIVIAL(debug) << "Parsing params succeeded, found entries:";
  for (uriparser::pairs_type::iterator it = _parameters.begin(); it != _parameters.end(); ++it) {
      if ((*it).second) {
          BOOST_LOG_TRIVIAL(debug)
              << (*it).first
              << " = "
              << boost::get<std::string>((*it).second);
        } else
        BOOST_LOG_TRIVIAL(debug) << (*it).first;
    }
  return true;
}


