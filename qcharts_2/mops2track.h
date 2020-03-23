#pragma once
#include <map>
#include <set>
#include <string>
#include <algorithm>
#include <fstream>
#include <vector>

#include "daa_datatypes_icd\trackICD.h"

//auto cmp = [](trackICD::track lhs, trackICD::track rhs) { return lhs.track_time < rhs.track_time; };

struct timeCompare{
	bool operator()(const trackICD::track& lhs, const trackICD::track& rhs)	{
		return lhs.track_time < rhs.track_time;
	}
};

typedef std::set<trackICD::track, timeCompare> trackSetType;

class mops2track {
public:
	enum class source { adsb=0, radar=1, camera, truth, unknown };
	typedef std::map<std::string, std::string> ROW;

public:
	inline static
	bool iequals(const std::string& a, const std::string& b){
		auto n=(std::min)(a.length(), b.length());

		return std::equal(a.begin(), a.begin()+n,
			b.begin(), [](char a, char b) {
				return tolower(a) == tolower(b);
		});
	}
	
	static void
	mapsSource2Flags(unsigned i, trackICD::source &source);

	static auto
    mapsRow(std::map<std::string, std::string> const &i_row,
        trackICD::track &track,
        source src,
        const std::string &callsign) -> int;

	//auto foo() const noexcept override -> int;
	static auto
	processCsvFile(const std::string &iCsvFname, 
		const std::string &oBinFname = "",
		source = source::unknown,
        const std::string &callsign = std::string("C1_INTR")) noexcept -> int;

	//csvFname vector: truth[0], adsb[1], radar[2] 
	//port vector: truth[0], adsb[1], radar[2]
	static int
    orchestrator(std::vector <trackICD::track> &iTrackVec);

    static int
    save2Bin(std::vector <trackICD::track> &iTrackVec, const std::string &oFname);

    //reads data from iCsvFname and puts in oTrackVec
	static int
	readCsvFileAndSave(const std::string &iCsvFname,
			std::vector <trackICD::track> &oTrackVec,
			source src,
            const std::string &callsign = std::string("C1_INTR"));

	//TODO: define this function
	void writeBuffer(const trackICD::track_buffer &oBuffer);

	//return meters
	static inline float
	NACp2EPU(uint16_t nacp) {
		switch (nacp) {
		default:
		case 0:
		case 1: return 18.52f * 1000;//(10 Nautical Miles)
		case 2: return 7.408f * 1000;//(4 Nautical Miles)
		case 3: return 3.704f * 1000;//(2 Nautical Miles)
		case 4: return 1852.0f;// (1 Nautical Miles)
		case 5: return 926.0f; // (0.5 Nautical Miles)
		case 6: return 555.6f; //(0.3 Nautical Miles)
		case 7: return 185.2f; //(0.1 Nautical Miles)
		case 8: return 92.6f; //(0.05 Nautical Miles)
		case 9: return 30.0f; 
		case 10: return 10.0f;
		case 11: return 3.0f;
		}
	}

	//return meters
	static inline float
	NIC2VPL(uint16_t nic) {
		switch (nic) {
		default:
		case 9: return 112.0f;
		case 10: return 37.5f;
		case 11: return 7.5f;
		}
	}

	//return meters/sec
	static inline float
	NACv2horVelErr(uint16_t nacv) {
		switch (nacv) {
		default:
		case 0: return 100.0f;
		case 1: return 10.0f;
		case 2: return 3.0f;
		case 4: return 0.3f;
		}
	}

	static inline void
		setDefaultValues(trackICD::track &track);

};



