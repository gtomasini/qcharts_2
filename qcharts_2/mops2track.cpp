#include <iostream>
#include <sstream>
#include <time.h>
#include <chrono>
#include <thread>
#include <functional>

//#include "trackLib\daa_iotools\daa_udp.h"
#include "parseCsv.h"
#include "mops2track.h"


//some constant convertions===================================
static const float kts_2_m_per_sec = 1852.0f / 3600;		//OK
static const float ft_per_min_2_m_per_sec = 1.0f/196.85f;	//OK
static const float deg_2_rad = 3.141592653589793f / 180.0f;	//OK	
static const float rad_2_deg = 1 / deg_2_rad;				//OK
static const float nmi_2_m = 1852;							//OK
static const float ft_2_m = 1.0f/3.280839895f;				//OK			
static const uint64_t secs_2_usecs = 1000000;				//OK
static const double usec_2_secs = 1 / secs_2_usecs;			//OK
//============================================================


auto
mops2track::processCsvFile(const std::string &fname,
                           const std::string &ofname,
                           source src_data,
                           const std::string &callsign) noexcept -> int {

	if (fname.empty()) {
        std::cerr<<"Null input csv file name."<<std::endl;
		return -1;
	}

	parseCsv iCsv(fname);
	if (!iCsv.fileIsOpen()) {
        std::cerr<<"Can't open csv file "<<fname<<std::endl;
		return -1;
	}

    //daa::net::udp<trackICD::track_buffer> my_udp(server, port);
	//output binary file
	std::ofstream oFile;
	if (!ofname.empty())
		oFile.open(ofname, std::ios::out | std::ios::binary);

	//source type
	mops2track::source src = src_data;
	if (src_data == source::unknown) {
		if (fname.find("ADSB") != std::string::npos)  src = mops2track::source::adsb;
		else if (fname.find("RADAR") != std::string::npos) src = mops2track::source::radar;
		else if (fname.find("CAMERA") != std::string::npos) src = mops2track::source::camera;
		else if (fname.find("Truth") != std::string::npos) src = mops2track::source::truth;
	}
	
    std::cout<<"reading csv input file "<<fname<<std::endl;
	iCsv.readHeader();

	for (ROW row; iCsv.readRow(row);) {
		//process row and puts data into track
		trackICD::track track;
        mapsRow(row, track, src, callsign);

		//parse track struct to oBuffer
		trackICD::track_buffer oBuffer;
		trackICD::write_track(track, oBuffer);
		
        //logger.debug("------------->sending track id: %u...", track.track_id);
		
		//writing to binary file
		if (oFile.is_open()) oFile.write((char*)&oBuffer, sizeof(oBuffer));
	}

	if (oFile.is_open()) oFile.close();
	return 0;
}

void
mops2track::mapsSource2Flags(unsigned in, trackICD::source &source) {
	source.UNFUSED_TRACK = false;
	source.FUSED_TRACK = false;
	source.EO_IR_CAMERA_TRACK = false;
	source.ADSB_TRACK = false;
	source.ACAS_TRACK = false;
	source.RADAR_TRACK = false;
	source.LIDAR = false;
	source.ACOUSTIC = false;
	source.UNKNOWN_SOURCE = false;
	
	switch (in) {
	case 0:	source.UNFUSED_TRACK = true;		break;
	case 1: source.FUSED_TRACK = true;			break;
	case 2: source.EO_IR_CAMERA_TRACK = true; 	break;
	case 3: source.ADSB_TRACK = true; 			break;
	case 4: source.ACAS_TRACK = true;    		break;
	case 5: source.RADAR_TRACK = true;			break;
	case 6: source.LIDAR = true;				break;
	case 7: source.ACOUSTIC = true;				break;
	case 8:
	default:source.UNKNOWN_SOURCE = true;	  	break;
	}
}

template <typename T>
inline void assignMag(std::istringstream &iss, bool &state_flag, T &value){
	iss >> value;
	//value = aux;
	state_flag = true;
}

auto 
mops2track::mapsRow(std::map<std::string, std::string> const &i_row, 
				trackICD::track &track,
				source src, 
                const std::string &callsign) -> int {


	setDefaultValues(track);

	//callsign set
	strncpy(track.callsign, callsign.c_str(), sizeof(track.callsign));

	//track id
	std::string aux = callsign;
	track.track_id = std::hash<std::string>{}(aux);

	//Source and track id
	std::string source = "unknown";
	if (src == mops2track::source::adsb) {
		track.source_data.ADSB_TRACK = true;
		track.track_id += 1;
		track.aircraft_model = trackICD::airframe::B777;
		source = "adsb";
	}
	else if (src== mops2track::source::radar){
		track.source_data.RADAR_TRACK = true;
		track.track_id += 2;
		source = "radar";
	}
	else if (src == mops2track::source::camera) {
		track.source_data.EO_IR_CAMERA_TRACK = true;
		track.track_id += 3;
		source = "camera";
	}
	else {
		track.source_data.UNKNOWN_SOURCE = true;
	}
	
	//auxiliary vars for futher Radar calculations below
	float rea=0, rng=0, rr=0, eaa=0, sra=0, rra=0;
	//auxilary vars for ADSB
	uint16_t nacp, nic, nacv;

	int k(0);//mmapping no matches number
	
	bool relative_altitude=false;//when exists RA(ft) we could read from csv

	//walk the hole map row: key -> value
	for (auto x : i_row){
		std::string key(x.first), value(x.second);
		std::istringstream iss(value);

		if (iequals("SRC", key)){
			//SRC: source, overwrites SRC field (see above)
			unsigned ui; iss >> ui;
			if (track.source_data.UNKNOWN_SOURCE)
				mapsSource2Flags( ui, track.source_data);
		}
		else if (iequals("TID", key)) {
			uint32_t tid; iss >> tid;
			if (tid != 1) track.track_id = tid;
		}  
		else if (iequals("ToR", key)) {//ADSB, RADAR
			//Time of Report(sec)
			double t_aux;
			iss >> t_aux;//secs
			t_aux *= 1000;//msecs
			track.track_time = t_aux;//msecs
			track.track_time = t_aux;//msecs
			track.track_time *= 1000;//usecs
		}
		else if (iequals("ToA", key)) {//ADSB, RADAR
			//Time of applicability(sec)
			assignMag(iss, track.aircraft_state.available_states.TOA, track.aircraft_state.toa);
			//track.track_time = track.aircraft_state.toa;
		}
		else if ( iequals("Lat", key) ) {//ADSB, RADAR
			//latitude(deg)
			assignMag(iss, track.aircraft_state.available_states.LATITUDE, track.aircraft_state.latitude);
		}
		else if ( iequals("Lon", key) ) {//ADSB, RADAR
			//longitude(deg)
			assignMag(iss, track.aircraft_state.available_states.LONGITUDE, track.aircraft_state.longitude);
		}
		else if ( iequals("Alt", key) ) {//ADSB, RADAR
			//altitude(ft)
			assignMag(iss, track.aircraft_state.available_states.GEOMETRIC_ALTITUDE, track.aircraft_state.geometric_altitude);
			track.aircraft_state.geometric_altitude *= ft_2_m;//m
			track.aircraft_state.barometric_altitude = track.aircraft_state.geometric_altitude;
			track.aircraft_state.available_states.BAROMETRIC_ALTITUDE = true;
		}
		else if ( iequals("EWV", key) ) {//ADSB
			//East West Velocity(kts)
			assignMag(iss, track.aircraft_state.available_states.GROUND_SPEED_EAST, track.aircraft_state.ground_speed_east);
			track.aircraft_state.ground_speed_east *= kts_2_m_per_sec;//m/sec
		}
		else if ( iequals("NSV", key) ) {//ADSB
			//North South Velocity(kts)
			assignMag(iss, track.aircraft_state.available_states.GROUND_SPEED_NORTH, track.aircraft_state.ground_speed_north);
			track.aircraft_state.ground_speed_north *= kts_2_m_per_sec;//m/sec;
		}
		else if ( iequals("VR(", key) ) {//ADSB
			//vertical rate(ft/min) or VS(vertical speed)
			assignMag(iss, track.aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE, track.aircraft_state.geometric_vertical_rate);
			track.aircraft_state.geometric_vertical_rate *= ft_per_min_2_m_per_sec;//m per sec
		}
		else if ( iequals("HDG(", key) ) {//RADAR: C1_RADAR_NONE_Tracker_TVInt1
			//aircraft_state_t.true_heading
			assignMag(iss, track.aircraft_state.available_states.TRUE_HEADING, track.aircraft_state.true_heading);
		}
		else if (iequals("RA(", key)) {//RADAR: C1_RADAR_NONE_Tracker_TVInt1
			//RA(ft): Relative altitude
			iss >> track.track_state.relative_altitude;//ft
			track.track_state.relative_altitude *= ft_2_m;
			relative_altitude = true;
		}
		else if (iequals("BRG", key)) {//RADAR
			//BRG(deg) : relative bearing(deg)
			iss >> track.track_state.relative_bearing;
			track.track_state.bearing_invalid = false;
		}

		//TODO: check uncertainties...
		else if ( iequals("APA", key) ) {//RADAR track
			//(APA) absolute position accuracy (nmi)
			iss >> track.aircraft_state.horizonal_position_uncertainty;
			track.aircraft_state.horizonal_position_uncertainty *= nmi_2_m;//m
		}
		else if ( iequals("PAA", key) ) {//RADAR track
			//PAA(ft): presure altitude accuracy
			iss >> track.aircraft_state.pressure_altitude_uncertainty;
			track.aircraft_state.pressure_altitude_uncertainty *= ft_2_m;
		}
		else if ( iequals("GSA", key) ) {//RADAR track
			//GSA(kts): ground speed accuracy
			iss >> track.aircraft_state.ground_speed_uncertainty;
			track.aircraft_state.ground_speed_uncertainty *= kts_2_m_per_sec;
		}
		else if ( iequals("HDGA", key) ) {//RADAR track
			//HDGA(deg): heading track accuracy 
			iss >> track.aircraft_state.heading_uncertainty;
		}
		else if ( iequals( "RBA", key )) {//RADAR
			//RBA(deg): Relative Bearing Accuracy, RBA
			iss >> track.track_state.relative_bearing_uncertainty;
		}
		//end uncertainties

		//RADAR: some auxilary	values for futher calculations
		else if (iequals("REA", key)) {
			//REA(deg): Relative Elevation Angle
			iss >> rea;//deg
			rea *= deg_2_rad;//radians

		}
		else if (iequals("RNG", key)) {
			//RNG: slant range
			iss >> rng;	//nmi
			rng *= nmi_2_m; //m
		}
		else if (iequals("RR(", key)) {//RADAR
			//RR: range rate
			iss >> rr;//kts
			rr *= kts_2_m_per_sec;//m per sec
		}
		else if (iequals("EAA", key)) {//RADAR
			//EAA(deg) : Elevation Angle Accuracy, not used
			iss >> eaa;//deg
			eaa *= deg_2_rad;//radians
		}
		else if (iequals("SRA", key)) {//RADAR
			//SRA(nmi) : Slant Range Accuracy
			iss >> sra;//nmi
			sra *= nmi_2_m;//m
			//std::cout << "SRA(m): " << sra << std::endl;
		}
		else if (iequals("RRA", key)) {//RADAR
			//RRA(knots) : Range Rate Accuracy
			iss >> rra;
			rra *= kts_2_m_per_sec;//m/sec
		}
		else if (iequals("NACp", key)) {//ADSB
			//Navigation Accuracy Category for Position
			iss >> nacp;
			track.aircraft_state.horizonal_position_uncertainty = NACp2EPU(nacp);
		}
		else if (iequals("NIC", key)) {
			iss >> nic;
			track.aircraft_state.pressure_altitude_uncertainty = NIC2VPL(nic);
			track.aircraft_state.geometric_altitude_uncertainty = NIC2VPL(nic);
		}
		else if (iequals("NACv", key)) {
			iss >> nacv;
			track.aircraft_state.ground_speed_uncertainty = NACv2horVelErr(nacv);
		}
		else {
			//key not mapped yet...
            std::cerr<<key<<" :\tNOT MAPPED. source:"<<source<<std::endl;
			k++;
		}
	}//for(;;)

	{	//RADAR
		//relative_range, relative_altitude, relative_ground_speedn (radar) CALCULATION
		//using RNG, REA && RR 

		track.track_state.relative_range = rng * cos(rea);
		if (!relative_altitude)
			track.track_state.relative_altitude = track.track_state.relative_range*tan(rea);
				
		track.track_state.relative_range_uncertainty = round(abs(sra * cos(rea))+0.50);
		track.track_state.relative_altitude_uncertainty = round(abs(sra * sin(rea))+0.50);
		
		//std::cout << "track.track_state.relative_range_uncertainty:" << track.track_state.relative_range_uncertainty << std::endl;
		//std::cout << "track_state.relative_altitude_uncertainty:" << track.track_state.relative_altitude_uncertainty << std::endl;

		track.track_state.relative_ground_speed_north = rr*sin(track.track_state.relative_bearing*deg_2_rad);
		track.track_state.relative_ground_speed_east = rr*cos(track.track_state.relative_bearing*deg_2_rad);
		//ground track angle: arctg(EWV/NSV) or arct(ground_speed_north/ground_speed_east)
		track.aircraft_state.available_states.GROUND_TRACK_ANGLE = true;
		if (track.aircraft_state.ground_speed_east == 0)
			track.aircraft_state.ground_track_angle = 0;
		else
			track.aircraft_state.ground_track_angle = atan(track.aircraft_state.ground_speed_east / track.aircraft_state.ground_speed_north);
		track.aircraft_state.ground_track_angle *= rad_2_deg;//rad
	}
	
	{	//track_time
		//if TOR not availabe ===> track_time := TOA
		if (track.track_time == 0 && track.aircraft_state.available_states.TOA)
			track.track_time = track.aircraft_state.toa*1000*1000;//usecs
	}
	
//	logger.debug("=======>track_time("+ source+"):\t%ld (msecs)",  (long)track.track_time/1000);
	return i_row.size() - k;//number matches
}

void
mops2track::setDefaultValues(trackICD::track &track) {
	//default values for some things
	memset(&track, sizeof(track), 0);

	track.source_data.UNFUSED_TRACK = false;
	track.source_data.FUSED_TRACK = false;
	track.source_data.EO_IR_CAMERA_TRACK = false;
	track.source_data.ADSB_TRACK = false;
	track.source_data.ACAS_TRACK = false;
	track.source_data.RADAR_TRACK = false;
	track.source_data.LIDAR = false;
	track.source_data.ACOUSTIC = false;
	track.source_data.UNKNOWN_SOURCE = false;

	track.aircraft_state.available_states.TOA = false;    // bit at 0 position
	track.aircraft_state.available_states.LONGITUDE = false;    // bit at 1 position
	track.aircraft_state.available_states.LATITUDE = false;    // bit at 2 position
	track.aircraft_state.available_states.GEOMETRIC_ALTITUDE = false;    // bit at 3 position
	track.aircraft_state.available_states.BAROMETRIC_ALTITUDE = false;    // bit at 4 position
	track.aircraft_state.available_states.TRUE_HEADING = false;    // bit at 5 position
	track.aircraft_state.available_states.MAGNETIC_HEADING = false;    // bit at 6 position
	track.aircraft_state.available_states.GROUND_TRACK_ANGLE = false;    // bit at 7 position
	track.aircraft_state.available_states.TRUE_AIRSPEED = false;    // bit at 8 position
	track.aircraft_state.available_states.INDICATED_AISPEED = false;    // bit at 9 position
	track.aircraft_state.available_states.GROUND_SPEED_NORTH = false;    // bit at 10 position
	track.aircraft_state.available_states.GROUND_SPEED_EAST = false;    // bit at 11 position
	track.aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = false;    // bit at 12 position
	track.aircraft_state.available_states.PRESSURE_VERTICAL_RATE = false;    // bit at 13 position
	track.aircraft_state.available_states.HEADING_RATE = false;    // bit at 14 position
	track.aircraft_state.available_states.GROUND_TRACK_RATE = false;    // NO, bit at 15 position

	track.aircraft_category = trackICD::aircraft_category_t::NA;
	track.aircraft_model = trackICD::airframe::UNKNOWN_AIRFRAME;
	track.aircraft_state.mode = trackICD::flight_mode::CRUISE;
	track.aircraft_state.available_states.MODE = true;

	track.track_state.track_size = trackICD::track_size_t::UNKNOWN_TRACK_SIZE;
	track.track_state.bearing_invalid = true;

	track.track_time = 0;	//usecs

	track.track_state.relative_bearing = 0;
	track.track_state.relative_range = 0;
	track.track_state.relative_altitude = 0;
	track.track_state.relative_ground_speed_north = 0;
	track.track_state.relative_ground_speed_east = 0;

	track.aircraft_state.horizonal_position_uncertainty = 0;
	track.aircraft_state.geometric_altitude_uncertainty = 0;
	track.aircraft_state.pressure_altitude_uncertainty = 0;
	track.aircraft_state.ground_speed_uncertainty = 0;
	track.aircraft_state.heading_uncertainty = 0;

	track.track_state.relative_range = 0;//Relative State variables as detailed in MOPS (section 2.2.3.2.2)
	track.track_state.relative_bearing = 0;
	track.track_state.relative_ground_speed_north = 0;
	track.track_state.relative_ground_speed_east = 0;
	track.track_state.relative_altitude = 0;
	track.track_state.relative_vertical_speed = 0;

	track.track_state.relative_range_uncertainty = 0;
	track.track_state.relative_bearing_uncertainty = 0;
	track.track_state.relative_ground_speed_north_uncertainty = 0;
	track.track_state.relative_ground_speed_east_uncertainty = 0;
	track.track_state.relative_altitude_uncertainty = 0;
	track.track_state.relative_vertical_speed_uncertainty = 0;

	//callsign="-End."
	track.callsign[0] = '-';
	track.callsign[1] = 'E';
	track.callsign[2] = 'n';
	track.callsign[3] = 'd';
	track.callsign[4] = '.';
	track.callsign[5] = 0;
}

int
mops2track::readCsvFileAndSave(const std::string &iCsvFname,
                std::vector <trackICD::track> &oTrackVec,
                source src,
                const std::string &callsign) {

    if (iCsvFname.empty()) {
        std::cerr<<"Warning: null file name."<<std::endl;
        return -1;
    }

    parseCsv iCsv(iCsvFname);
    if (!iCsv.fileIsOpen()) {
        std::cerr<<"Can't open input csv file "<<iCsvFname<<std::endl;
        return -1;
    }

    //logger.debug("reading csv input file %s...", iCsvFname);
    iCsv.readHeader();

    for (ROW row; iCsv.readRow(row);) {//process row and puts data into track
        trackICD::track track;
        mapsRow(row, track, src, callsign);
        std::cout<<track.track_time<<std::endl;
        oTrackVec.push_back(track);
    }

    return 0;
}

int
mops2track::orchestrator(std::vector<trackICD::track> &iTrackVec) {

    std::sort(iTrackVec.begin(), iTrackVec.end(),
            [](trackICD::track lhs, trackICD::track rhs) { return lhs.track_time < rhs.track_time; });

    return 0;
}

int
mops2track::save2Bin(std::vector<trackICD::track> &iTrackVec, const std::string &oFname){
    std::ofstream oFile;
    if (!oFname.empty()) {//open oFile only if oBinFname != ""
        oFile.open(oFname, std::ios::out | std::ios::binary);
    }
    else{
       std::cerr<<"bin output file error not opened"<<std::endl;
       return -1;
    }

    for (int i=0; i<iTrackVec.size(); ++i){
        //parse track struct to oBuffer
        trackICD::track_buffer oBuffer;
        trackICD::write_track(iTrackVec[i], oBuffer);
        oFile.write((char*)&oBuffer, sizeof(oBuffer));
    }
    oFile.close();

    return 0;
}
