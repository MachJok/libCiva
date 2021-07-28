#include "Constants.hpp"
#include "Geocentric.hpp"
#include "Geodesic.hpp"
#include "sources.h"

static const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
static const GeographicLib::Geocentric& ecef = GeographicLib::Geocentric::WGS84();
