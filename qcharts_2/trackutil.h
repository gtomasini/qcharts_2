#ifndef TRACKUTIL_H
#define TRACKUTIL_H

#include <vector>
#include <string>
#include <utility>
#include <iostream>
#include <QtCharts\qlineseries.h>

#include "daa_datatypes_icd\trackICD.h"

typedef std::pair<QtCharts::QLineSeries, QtCharts::QLineSeries> seriesPair;
typedef std::pair<std::string, std::string> labelPair;

class tbase{
protected:
    trackICD::track t_;

public:
    tbase(trackICD::track const &t):t_(t){}
    virtual float getX(){ return t_.track_time/1000000;}//secs
    virtual std::string getName(){ return "axisY"; }
    virtual std::string getNameX(){ return "track time"; }
    virtual bool getAvailable(){return true;}
    virtual float getMagnitude()=0;
    bool getUNKNOWN_SOURCE(){ return t_.source_data.UNKNOWN_SOURCE;}
    bool getADSB_SOURCE(){ return t_.source_data.ADSB_TRACK;}
    bool getRADAR_SOURCE() { return t_.source_data.RADAR_TRACK; }
    virtual ~tbase(){}
};

class geometricAltitudeAdsb:public tbase{
public:
    geometricAltitudeAdsb(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return t_.aircraft_state.geometric_altitude; }
    bool getAvailable(){ return t_.aircraft_state.available_states.GEOMETRIC_ALTITUDE; }
    std::string getName(){ return "geometric altitude";}
};

class groundSpeedNorthAdsb:public tbase{
public:
    groundSpeedNorthAdsb(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return t_.aircraft_state.ground_speed_north; }
    bool getAvailable(){ return t_.aircraft_state.available_states.GROUND_SPEED_NORTH; }
    std::string getName(){ return "ground speed north";}
};

class groundSpeedEastAdsb:public tbase{
public:
    groundSpeedEastAdsb(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return t_.aircraft_state.ground_speed_east; }
    bool getAvailable(){ return t_.aircraft_state.available_states.GROUND_SPEED_EAST; }
    std::string getName(){ return "ground speed east";}
};

class groundTrackAngleAdsb:public tbase{
public:
    groundTrackAngleAdsb(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return t_.aircraft_state.ground_track_angle; }
    bool getAvailable(){ return t_.aircraft_state.available_states.GROUND_TRACK_ANGLE; }
    std::string getName(){ return "ground track angle";}
};

class latitudeAdsb:public tbase{
public:
    latitudeAdsb(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return t_.aircraft_state.latitude; }
    bool getAvailable(){ return t_.aircraft_state.available_states.LATITUDE; }
    std::string getName(){ return "latitude";}
};

class longitudeAdsb:public tbase{
public:
    longitudeAdsb(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return t_.aircraft_state.longitude; }
    bool getAvailable(){ return t_.aircraft_state.available_states.LONGITUDE; }
    std::string getName(){ return "longitude";}
};

class latitudeLongitudeAdsb:public tbase{
public:
    latitudeLongitudeAdsb(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return t_.aircraft_state.longitude; }
    std::string getName(){ return "longitude";}
    float getX(){ return t_.aircraft_state.latitude; }
    std::string getNameX(){ return "latitude"; }
    bool getAvailable(){ return t_.aircraft_state.available_states.LONGITUDE &&
                            t_.aircraft_state.available_states.LATITUDE; }
};

class relativeRangeFromRadar:public tbase{
public:
    relativeRangeFromRadar(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return t_.track_state.relative_range; }
    std::string getName(){ return "relative range";}
};

class relativeAltitudeFromRadar:public tbase{
public:
    relativeAltitudeFromRadar(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return (float)t_.track_state.relative_altitude;}
    std::string getName(){ return "relative altitude"; }
};

class relativeBearingFromRadar:public tbase{
public:
    relativeBearingFromRadar(trackICD::track const &t):tbase(t){}
    float getMagnitude(){ return t_.track_state.relative_bearing;}
    std::string getName(){ return "relative bearing"; }
};

class trackUtil{
public:
    enum Magnitude { geomAltADSB, groundSpeedNorthADSB, groundSpeedEastADSB,
                   groundTrackAngleADSB, latitudeADSB, longitudeADSB,
                   horizontalPositionADSB, relativeRangeRADAR,
                   relativeAltitudeRADAR, relativeBearingRADAR};

    trackUtil(const std::string &binfname);
    trackUtil(){}
    trackUtil(std::vector <trackICD::track> const &v){
        trackV_=v;
    }

    bool readBinFile(const std::string &binfname, bool clearVector=true);
    bool getFerror(){ return fError_; }
    std::string &getLastError(){ return lasterror_; }

    //sets track vector source...
    void setVectorSourceAsUnknown();

    template<class F>
    void getMagnitudeFromADSBorRADAR(seriesPair &series, bool adsb=true){
        for(auto &x:trackV_){
            F f(x);
            if (f.getAvailable()){
                std::cout<<f.getNameX()<<": "<<f.getX()
                    <<", "<<f.getName()<<": "<<f.getMagnitude()<<",";
                if ( f.getUNKNOWN_SOURCE()){
                    series.second.append(f.getX(), f.getMagnitude());
                    dataAppended_ = true;
                    std::cout<<"(TRUTH)"<<std::endl;
                }
                else if (adsb && f.getADSB_SOURCE() ){//adsb
                    series.first.append(f.getX(), f.getMagnitude());
                    dataAppended_ = true;
                    std::cout<<"(ADSB)"<<std::endl;
                }
                else if ( !adsb && f.getRADAR_SOURCE() ){//radar
                    series.first.append(f.getX(), f.getMagnitude());
                    dataAppended_ = true;
                    std::cout<<"(RADAR)"<<std::endl;
                }
                else if (f.getADSB_SOURCE())
                   std::cout<<"(ADSB source) not appended."<<std::endl;
                else if (f.getRADAR_SOURCE() )
                   std::cout<<"(RADAR source) not appended."<<std::endl;
            }
        }
    }

    template<class F>
    void getMagnitudeFromADSBorRADAR(QtCharts::QLineSeries *truth,
                                     QtCharts::QLineSeries *actual, bool adsb=true){
        for(auto &x:trackV_){
            F f(x);
            if (f.getAvailable()){
                std::cout<<f.getNameX()<<": "<<f.getX()
                    <<", "<<f.getName()<<": "<<f.getMagnitude()<<",";
                if ( f.getUNKNOWN_SOURCE()){
                    truth->append(f.getX(), f.getMagnitude());
                    dataAppended_ = true;
                    std::cout<<"(TRUTH)"<<std::endl;
                }
                else if (adsb && f.getADSB_SOURCE() ){//adsb
                    actual->append(f.getX(), f.getMagnitude());
                    dataAppended_ = true;
                    std::cout<<"(ADSB)"<<std::endl;
                }
                else if ( !adsb && f.getRADAR_SOURCE() ){//radar
                    actual->append(f.getX(), f.getMagnitude());
                    dataAppended_ = true;
                    std::cout<<"(RADAR)"<<std::endl;
                }
                else if (f.getADSB_SOURCE())
                   std::cout<<"(ADSB source) not appended."<<std::endl;
                else if (f.getRADAR_SOURCE() )
                   std::cout<<"(RADAR source) not appended."<<std::endl;
            }
        }
    }

    labelPair
    getGroundSpeedNorthSerieFromAdsb(seriesPair &series);

    std::pair<std::string, std::string>
    getGroundSpeedEastSerieFromAdsb(QtCharts::QLineSeries &actualSerie, QtCharts::QLineSeries &truthSerie);

    std::pair<std::string, std::string>
    getGroundTrackAngleSerieFromAdsb(QtCharts::QLineSeries &actualSerie, QtCharts::QLineSeries &truthSerie);

    std::pair<std::string, std::string>
    getLatitudeSerieFromAdsb(QtCharts::QLineSeries &actualSerie, QtCharts::QLineSeries &truthSerie);

    std::pair<std::string, std::string>
    getLongitudeSerieFromAdsb(QtCharts::QLineSeries &actualSerie, QtCharts::QLineSeries &truthSerie);

    labelPair
    getHorizontalPositionFromAdsb(seriesPair &series);

    //Values from RADAR track
    std::pair<std::string, std::string>
    getRelativeRangeFromRadar(QtCharts::QLineSeries &actualSerie, QtCharts::QLineSeries &truthSerie);

    std::pair<std::string, std::string>
    getRelativeAltitudeFromRadar(QtCharts::QLineSeries &actualSerie, QtCharts::QLineSeries &truthSerie);

    std::pair<std::string, std::string>
    getRelativeBearingFromRadar(QtCharts::QLineSeries &actualSerie, QtCharts::QLineSeries &truthSerie);

    bool getDataAppended(){ return dataAppended_; }
    int getVectorSize(){ return trackV_.size(); }

private:
    std::vector <trackICD::track>  trackV_;
    bool fError_=false;
    std::string lasterror_="";
    bool dataAppended_=false;
};

#endif // TRACKUTIL_H
