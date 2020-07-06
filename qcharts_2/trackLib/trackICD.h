#ifndef trackICD_H
#define trackICD_H
#include <stdint.h>

namespace trackICD {
//Type hash codes

//Type definitions
   enum aircraft_category_t   {
      NA = 0,
      LIGHT = 1,
      SMALL = 2,
      LARGE = 3,
      HVORTEX = 4,
      HEAVY = 5,
      HPERFORMANCE = 6,
      ROTORCRAFT = 7,
      GLIDER = 8,
      LTA = 9,
      PARACHUTE = 10,
      UAV = 11,
      SPACE = 12,
      aircraft_category_t_end = 13
   };

   enum airframe   {
      B717 = 0,
      B727 = 1,
      B737 = 2,
      B747 = 3,
      B757 = 4,
      B767 = 5,
      B777 = 6,
      B787 = 7,
      A310 = 8,
      A319 = 9,
      A320 = 10,
      A321 = 11,
      A330 = 12,
      A340 = 13,
      C172 = 14,
      SCE = 15,
      MUG = 16,
      UNKNOWN_AIRFRAME = 17,
      airframe_end = 18
   };

   struct source   {
      bool UNFUSED_TRACK;    // bit at 0 position
      bool FUSED_TRACK;    // bit at 1 position
      bool EO_IR_CAMERA_TRACK;    // bit at 2 position
      bool ADSB_TRACK;    // bit at 3 position
      bool ACAS_TRACK;    // bit at 4 position
      bool RADAR_TRACK;    // bit at 5 position
      bool LIDAR;    // bit at 6 position
      bool ACOUSTIC;    // bit at 7 position
      bool UNKNOWN_SOURCE;    // bit at 8 position
   };

   enum flight_mode   {
      AT_GATE_HANGAR = 0,
      TAXI = 1,
      HOLD = 2,
      TAKE_OFF = 3,
      CLIMB = 4,
      CRUISE = 5,
      DESCENT = 6,
      LAND = 7,
      flight_mode_end = 8
   };

   struct state_variable  {
      bool TOA;    // bit at 0 position
      bool LONGITUDE;    // bit at 1 position
      bool LATITUDE;    // bit at 2 position
      bool GEOMETRIC_ALTITUDE;    // bit at 3 position
      bool BAROMETRIC_ALTITUDE;    // bit at 4 position
      bool TRUE_HEADING;    // bit at 5 position
      bool MAGNETIC_HEADING;    // bit at 6 position
      bool GROUND_TRACK_ANGLE;    // bit at 7 position
      bool TRUE_AIRSPEED;    // bit at 8 position
      bool INDICATED_AISPEED;    // bit at 9 position
      bool GROUND_SPEED_NORTH;    // bit at 10 position
      bool GROUND_SPEED_EAST;    // bit at 11 position
      bool GEOMETRIC_VERTICAL_RATE;    // bit at 12 position
      bool PRESSURE_VERTICAL_RATE;    // bit at 13 position
      bool HEADING_RATE;    // bit at 14 position
      bool GROUND_TRACK_RATE;    // bit at 15 position
      bool MODE;    // bit at 16 position
   };

   typedef char aircraft_state_t_buffer[113];
   struct aircraft_state_t {
      state_variable available_states;      //Bit mask of available states
      uint64_t toa;			//Time of applicability. Units=[sec]
      double longitude;		//Longitude in degrees referenced to WGS84. Units=[deg]
      double latitude;		//Latitude in degrees referenced to WGS84. Units=[deg]
      float geometric_altitude;	      //height above WGS84 ellipsoid. Units=[m]
      float barometric_altitude;      //Barometric pressure altitude relative to 1013.25 millibars. Units=[m]
      float true_heading;			  //Units=[deg]
      float magnetic_heading;		  //Units=[deg]
      float ground_track_angle;	      //Units=[deg]
      float true_air_speed;			  //Units=[m/s]
      float indicated_air_speed;      //Units=[m/s]
      float ground_speed_north;       //Units=[m/s]
      float ground_speed_east;        //Units=[m/s]
      float geometric_vertical_rate;  //Units=[m/s]
      float pressure_vertical_rate;	  //Units=[m/s]
      float heading_rate;			  //Units=[deg/s]
      float ground_track_rate;	      //Units=[deg/s]//lastone 
      flight_mode mode;				//fix it
      uint16_t horizonal_position_uncertainty;	   //Units=[m]
      uint16_t pressure_altitude_uncertainty;      //Units=[m]
      uint16_t geometric_altitude_uncertainty;     //Units=[m]
      float ground_speed_uncertainty;     //Units=[m/s]
      float airspeed_uncertainty;	      //Units=[m/s]
      float heading_uncertainty;			//Units=[deg]
      float ground_track_angle_uncertainty;     //Units=[deg]
      float heading_rate_uncertainty;			//Units=[deg/s]
      float ground_track_rate_uncertainty;      //Units=[deg/s]
   };

   enum track_size_t   {
      UNKNOWN_TRACK_SIZE = 0,
      SMALL_TRACK = 1,
      MEDIUM_TRACK = 2,
      LARGE_TRACK = 3,
      track_size_t_end = 4};

   typedef char track_state_t_buffer[49];

   struct track_state_t {
      float relative_range;//Relative State variables as detailed in MOPS (section 2.2.3.2.2)
      float relative_bearing;
      uint8_t bearing_invalid;
      float relative_ground_speed_north;
      float relative_ground_speed_east;
      int16_t relative_altitude;
      float relative_vertical_speed;
      track_size_t track_size;
      uint32_t relative_range_uncertainty;
      float relative_bearing_uncertainty;
      float relative_ground_speed_north_uncertainty;
      float relative_ground_speed_east_uncertainty;
      int16_t relative_altitude_uncertainty;
      float relative_vertical_speed_uncertainty;
   };

   typedef char track_buffer[192];
   struct track   {
      uint32_t track_id;     //Unique ID for a track
      char callsign[8];      //8 character traffic callsign (if from cooperative report)
      uint64_t track_time;   //time at which the track was generated
      aircraft_category_t aircraft_category;//Aircraft category (according to ADSB report)
      airframe aircraft_model; //Model of aircraft if known
      source source_data;      //Sensor information generating the track data
      track_state_t track_state;   //Relative state vector between intruder and own-ship as detailed in the Track State data structure
      aircraft_state_t aircraft_state;//Intruder state vector as detailed in the Aircraft State data structure
   };

   typedef char track_list_buffer[1921];
   struct track_list  {
      track tracks[10];
      uint8_t trackCount;
   };


//Reader Functions
   void read_aircraft_state_t(aircraft_state_t& data, const aircraft_state_t_buffer& raw);
   void read_aircraft_state_t_ptr(aircraft_state_t& data, const char* raw);
   void read_track(track& data, const track_buffer& raw);
   void read_track_ptr(track& data, const char* raw);
   void read_track_list(track_list& data, const track_list_buffer& raw);
   void read_track_list_ptr(track_list& data, const char* raw);

//Writer functions
   void write_aircraft_state_t(const aircraft_state_t& data, aircraft_state_t_buffer& raw);
   void write_aircraft_state_t_ptr(const aircraft_state_t& data, char* raw);
   void write_track(const track& data, track_buffer& raw);
   void write_track_ptr(const track& data, char* raw);
   void write_track_list(const track_list& data, track_list_buffer& raw);
   void write_track_list_ptr(const track_list& data, char* raw);

}
#endif //trackICD_H
