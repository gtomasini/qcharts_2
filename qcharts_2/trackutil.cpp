#include <fstream>
#include <iostream>
#include <string>
#include <errno.h>

#include "trackutil.h"

template <typename T>
inline void displayMag(const std::string &title, bool flag, T value) {
    std::cout << title;//  "TOA...................:\t";
    if (!flag) std::cout << "NOT available.";
    else std::cout << value;// track.aircraft_state.toa;
    std::cout << std::endl;
}

trackUtil::trackUtil(const std::string &binfname){
    fError_ = readBinFile(binfname);
}

bool trackUtil::readBinFile(const std::string &binfname,
                            bool clearVector){
    if (binfname.empty()){
        lasterror_="input file "+binfname+" is null";
        return true;
    }

    std::ifstream iFile(binfname, std::ios::in | std::ios::binary);

    if (!iFile.is_open()){
        lasterror_= strerror(errno);//TODO replace this because it is deprecated
        std::cerr << "Error: can't open " << binfname << std::endl;
        return true;
    }

    if (clearVector) trackV_.clear();

    for (;!iFile.eof();) {
        char buffer[192];
        iFile.read( buffer, sizeof(buffer) );
        if (!iFile) {
             std::cerr << "Warning: only " << iFile.gcount() << " bytes could be read." <<std::endl;;
             continue;
         }
         std::cout << "\n************" << iFile.gcount()<<" bytes read*******"<<std::endl;
         trackICD::track track;
         trackICD::read_track(track, buffer);
         std::cout << "Track Time(msecs)....:\t" << track.track_time / 1000 << std::endl;
         std::cout << "Track ID.............:\t" << track.track_id << std::endl;
         std::cout << "callsign.............:\t" << track.callsign << std::endl;

         trackV_.push_back(track);
    }
    iFile.close();
    return false;
}

void trackUtil::setVectorSourceAsUnknown(){
    for (auto &t:trackV_){
        t.source_data.ACAS_TRACK=false;
        t.source_data.ACOUSTIC=false;
        t.source_data.ADSB_TRACK=false;
        t.source_data.EO_IR_CAMERA_TRACK=false;
        t.source_data.FUSED_TRACK=false;
        t.source_data.LIDAR=false;
        t.source_data.RADAR_TRACK=false;
        t.source_data.UNFUSED_TRACK=false;
        t.source_data.UNKNOWN_SOURCE=true;
    }
}
