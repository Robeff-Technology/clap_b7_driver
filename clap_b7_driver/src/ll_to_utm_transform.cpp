//
// Created by elaydin on 12.07.2023.
//

#include <clap_b7_driver/ll_to_utm_transform.h>
#include <math.h>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace  clap_b7{
    void LlToUtmTransform::set_origin(double lat, double lon, double altitude) {
        initUTM(lat, lon, altitude);
        origin_is_set_ = true;
    }

    int LlToUtmTransform::find_zone(double Lat, double Long) const {
        double long_temp = (Long+180)-int((Long+180)/360)*360-180;
        int zone_number;
        zone_number = int((long_temp + 180)/6) + 1;

        if( Lat >= 56.0 && Lat < 64.0 && long_temp >= 3.0 && long_temp < 12.0 )
        {
            zone_number = 32;
        }

        // Special zones for Svalbard
        if( Lat >= 72.0 && Lat < 84.0 )
        {
            if(      long_temp >= 0.0  && long_temp <  9.0 ) zone_number = 31;
            else if( long_temp >= 9.0  && long_temp < 21.0 ) zone_number = 33;
            else if( long_temp >= 21.0 && long_temp < 33.0 ) zone_number = 35;
            else if( long_temp >= 33.0 && long_temp < 42.0 ) zone_number = 37;
        }

        return zone_number;
    }

    void LlToUtmTransform::transform_local(double lat, double lon, double altitude, double &x, double &y, double &z) const {
        if(!origin_is_set_){
            throw std::runtime_error("Origin is not set");
        }

        double northing = NAN;
        double easting = NAN;
        LLtoUTM(lat, lon, find_zone(lat, lon), northing, easting);
        x = easting - m_utm0_.easting;
        y = northing - m_utm0_.northing;
        z = altitude - m_utm0_.altitude;

    }

    void LlToUtmTransform::transform_global(double lat, double lon, double altitude, double &x, double &y, double &z) const {
        int zone;
        bool northp;
        try{
            GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
            z = altitude;
        }
        catch (const GeographicLib::GeographicErr & err) {
            throw std::runtime_error("Failed to convert from LLH to UTM = "  + std::string(err.what()));
        }
    }

    void LlToUtmTransform::initUTM(double Lat, double Long, double altitude)
    {
        m_utm0_.zone = find_zone(Lat, Long);
        m_utm0_.altitude = altitude;
        LLtoUTM(Lat, Long, m_utm0_.zone, m_utm0_.northing, m_utm0_.easting);

        RCLCPP_INFO(rclcpp::get_logger("Message wrapper"), "initialized from lat:%f long:%f UTM zone %d%c: easting:%fm (%dkm) northing:%fm (%dkm)"
                , Lat, Long, m_utm0_.zone, UTMLetterDesignator(Lat)
                , m_utm0_.easting, (int)(m_utm0_.easting)/1000
                , m_utm0_.northing, (int)(m_utm0_.northing)/1000
        );
    }

    char LlToUtmTransform::UTMLetterDesignator(double Lat)
    {
        char LetterDesignator;

        if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
        else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
        else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
        else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
        else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
        else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
        else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
        else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
        else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
        else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
        else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
        else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
        else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
        else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
        else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
        else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
        else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
        else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
        else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
        else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
            // 'Z' is an error flag, the Latitude is outside the UTM limits
        else LetterDesignator = 'Z';
        return LetterDesignator;
    }

    void LlToUtmTransform::LLtoUTM(double Lat, double Long, int zoneNumber, double &UTMNorthing, double &UTMEasting) const
    {
        const double RADIANS_PER_DEGREE = M_PI/180.0;

        // WGS84 Parameters
        const double WGS84_A = 6378137.0;        // major axis
        const double WGS84_E = 0.0818191908;     // first eccentricity

        // UTM Parameters
        const double UTM_K0 = 0.9996;            // scale factor
        const double UTM_E2 = (WGS84_E*WGS84_E); // e^2

        double a = WGS84_A;
        double eccSquared = UTM_E2;
        double k0 = UTM_K0;

        double LongOrigin;
        double eccPrimeSquared;
        double N, T, C, A, M;

        // Make sure the longitude is between -180.00 .. 179.9
        double LongTemp = (Long+180)-int((Long+180)/360)*360-180;

        double LatRad = Lat*RADIANS_PER_DEGREE;
        double LongRad = LongTemp*RADIANS_PER_DEGREE;
        double LongOriginRad;

        // +3 puts origin in middle of zone
        LongOrigin = (zoneNumber - 1)*6 - 180 + 3;
        LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

        eccPrimeSquared = (eccSquared)/(1-eccSquared);

        N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
        T = tan(LatRad)*tan(LatRad);
        C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
        A = cos(LatRad)*(LongRad-LongOriginRad);

        M = a*((1 - eccSquared/4      - 3*eccSquared*eccSquared/64     - 5*eccSquared*eccSquared*eccSquared/256)*LatRad
               - (3*eccSquared/8   + 3*eccSquared*eccSquared/32    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
               + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
               - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

        UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
                                    + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
                              + 500000.0);

        UTMNorthing = (double)(k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                                                    + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

        if(Lat < 0)
        {
            UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
        }
    }




} // namespace clap_b7