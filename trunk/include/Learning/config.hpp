#ifndef __WALKER_YARD_LEARNING_INI_HPP__
#define __WALKER_YARD_LEARNING_INI_HPP__

#include <boost/property_tree/ini_parser.hpp>

namespace learn {

// forward
template < typename ValueType,
           typename Environment,
           typename ActionFunction >
class plain_direct_learning;

template<typename T, typename E, typename A>
inline void init_from_ini(plain_direct_learning<T,E,A>& rl, const std::string& configFile)
{
    boost::property_tree::ptree properties;
	boost::property_tree::read_ini(configFile, properties);
	
    rl.sigma    = properties.get("Sigma",   rl.sigma);
    rl.beta     = properties.get("Beta",    rl.beta);
    rl.gamma    = properties.get("Gamma",   rl.gamma);
}

} // namespace learn

#endif // __WALKER_YARD_LEARNING_INI_HPP__