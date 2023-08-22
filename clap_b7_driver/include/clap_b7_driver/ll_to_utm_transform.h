//
// Created by elaydin on 12.07.2023.
//

#ifndef LL_TO_UTM_TRANSFORM_H
#define LL_TO_UTM_TRANSFORM_H

namespace clap_b7{

    class LlToUtmTransform {
    private:
        struct Origin{
            int zone;
            double northing;
            double easting;
            double altitude;
        };




        bool origin_is_set_{false};
    public:
        LlToUtmTransform() = default;
        ~LlToUtmTransform() = default;
        void set_origin(double lat, double lon, double altitude);
        void transform_local(double lat, double lon, double altitude, double& x, double& y, double& z) const;

        void transform_global(double lat, double lon, double altitude, double& x, double& y, double& z) const;

        void initUTM(double Lat, double Long, double altitude);

        char UTMLetterDesignator(double Lat);

        void LLtoUTM(double Lat, double Long, int zoneNumber, double &UTMNorthing, double &UTMEasting) const;

        int find_zone(double Lat, double long_temp) const;

        Origin m_utm0_;
    };

} // namespace clap_b7
#endif
