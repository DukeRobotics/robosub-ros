#ifndef VREPPLUSPLUS_HANDLE_H_INCLUDED
#define VREPPLUSPLUS_HANDLE_H_INCLUDED

#include <string>
#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

namespace vrep
{
	/*! \brief A tool for converting pointers to strings and vice versa.
     *
     * Usage: specialize the Handle<T>::tag() method for your class, e.g.:
     *
     * template<> std::string Handle<OcTree>::tag() { return "octomap.OcTree"; }
	 */
    template<typename T>
    struct Handle
    {
		static std::string str(const T *t)
		{
			static boost::format fmt("%s:%lld:%d");
			return (fmt % tag() % reinterpret_cast<long long int>(t) % crc_ptr(t)).str();
		}

		static T * obj(std::string h)
		{
			boost::cmatch m;
			static boost::regex re("([^:]+):([^:]+):([^:]+)");
			if(boost::regex_match(h.c_str(), m, re) && m[1] == tag())
			{
				T *t = reinterpret_cast<T*>(boost::lexical_cast<long long int>(m[2]));
				int crc = boost::lexical_cast<int>(m[3]);
				if(crc == crc_ptr(t)) return t;
			}
			return nullptr;
		}

	private:
		static std::string tag()
		{
			return "ptr";
		}

		static int crc_ptr(const T *t)
		{
			auto x = reinterpret_cast<long long int>(t);
			x = x ^ (x >> 32);
			x = x ^ (x >> 16);
			x = x ^ (x >> 8);
			x = x ^ (x >> 4);
			x = x & 0x000000000000000F;
			x = x ^ 0x0000000000000008;
			return int(x);
		}
    };
}

#endif // VREPPLUSPLUS_HANDLE_H_INCLUDED
