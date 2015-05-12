/* Created on: May 07, 2014. Author: Timm Linder */
#ifndef _STL_HELPERS_H
#define _STL_HELPERS_H


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// Appends the second STL container (e.g. vector) to the first
template <typename L, typename R> void appendTo(L& lhs, R const& rhs) {
    lhs.insert(lhs.end(), rhs.begin(), rhs.end());
}


using namespace std;


#endif // _STL_HELPERS_H