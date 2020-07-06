#include "trackICD.h"
namespace trackICD {
   union ___int32_to_bytes { int32_t a;  char b[4]; };  static ___int32_to_bytes __convert_int32_to_bytes;
   union ___int64_to_bytes { int64_t a;  char b[8]; };  static ___int64_to_bytes __convert_int64_to_bytes;
   union ___int8_to_bytes { int8_t a;  char b[1]; };  static ___int8_to_bytes __convert_int8_to_bytes;
   union ___uint64_to_bytes { uint64_t a;  char b[8]; };  static ___uint64_to_bytes __convert_uint64_to_bytes;
   union ___float32_to_bytes { float a;  char b[4]; };  static ___float32_to_bytes __convert_float32_to_bytes;
   union ___uint32_to_bytes { uint32_t a;  char b[4]; };  static ___uint32_to_bytes __convert_uint32_to_bytes;
   union ___float64_to_bytes { double a;  char b[8]; };  static ___float64_to_bytes __convert_float64_to_bytes;
   union ___text_to_bytes { char a;  char b[1]; };  static ___text_to_bytes __convert_text_to_bytes;
   union ___uint8_to_bytes { uint8_t a;  char b[1]; };  static ___uint8_to_bytes __convert_uint8_to_bytes;
   union ___uint16_to_bytes { uint16_t a;  char b[2]; };  static ___uint16_to_bytes __convert_uint16_to_bytes;
   union ___int16_to_bytes { int16_t a;  char b[2]; };  static ___int16_to_bytes __convert_int16_to_bytes;
//Writer functions
   void write_aircraft_state_t(const aircraft_state_t& data, aircraft_state_t_buffer& raw) {
      write_aircraft_state_t_ptr(data, &(raw[0]));
   };
   void write_aircraft_state_t_ptr(const aircraft_state_t& data,  char* raw) {
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[0] = bits[0];
         raw[1] = bits[1];
         raw[2] = bits[2];
      } 
      { //write data.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.toa;
         raw[3] = __convert_uint64_to_bytes.b[0];
         raw[4] = __convert_uint64_to_bytes.b[1];
         raw[5] = __convert_uint64_to_bytes.b[2];
         raw[6] = __convert_uint64_to_bytes.b[3];
         raw[7] = __convert_uint64_to_bytes.b[4];
         raw[8] = __convert_uint64_to_bytes.b[5];
         raw[9] = __convert_uint64_to_bytes.b[6];
         raw[10] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.longitude;
         raw[11] = __convert_float64_to_bytes.b[0];
         raw[12] = __convert_float64_to_bytes.b[1];
         raw[13] = __convert_float64_to_bytes.b[2];
         raw[14] = __convert_float64_to_bytes.b[3];
         raw[15] = __convert_float64_to_bytes.b[4];
         raw[16] = __convert_float64_to_bytes.b[5];
         raw[17] = __convert_float64_to_bytes.b[6];
         raw[18] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.latitude;
         raw[19] = __convert_float64_to_bytes.b[0];
         raw[20] = __convert_float64_to_bytes.b[1];
         raw[21] = __convert_float64_to_bytes.b[2];
         raw[22] = __convert_float64_to_bytes.b[3];
         raw[23] = __convert_float64_to_bytes.b[4];
         raw[24] = __convert_float64_to_bytes.b[5];
         raw[25] = __convert_float64_to_bytes.b[6];
         raw[26] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.geometric_altitude;
         raw[27] = __convert_float32_to_bytes.b[0];
         raw[28] = __convert_float32_to_bytes.b[1];
         raw[29] = __convert_float32_to_bytes.b[2];
         raw[30] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.barometric_altitude;
         raw[31] = __convert_float32_to_bytes.b[0];
         raw[32] = __convert_float32_to_bytes.b[1];
         raw[33] = __convert_float32_to_bytes.b[2];
         raw[34] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.true_heading;
         raw[35] = __convert_float32_to_bytes.b[0];
         raw[36] = __convert_float32_to_bytes.b[1];
         raw[37] = __convert_float32_to_bytes.b[2];
         raw[38] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.magnetic_heading;
         raw[39] = __convert_float32_to_bytes.b[0];
         raw[40] = __convert_float32_to_bytes.b[1];
         raw[41] = __convert_float32_to_bytes.b[2];
         raw[42] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.ground_track_angle;
         raw[43] = __convert_float32_to_bytes.b[0];
         raw[44] = __convert_float32_to_bytes.b[1];
         raw[45] = __convert_float32_to_bytes.b[2];
         raw[46] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.true_air_speed;
         raw[47] = __convert_float32_to_bytes.b[0];
         raw[48] = __convert_float32_to_bytes.b[1];
         raw[49] = __convert_float32_to_bytes.b[2];
         raw[50] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.indicated_air_speed;
         raw[51] = __convert_float32_to_bytes.b[0];
         raw[52] = __convert_float32_to_bytes.b[1];
         raw[53] = __convert_float32_to_bytes.b[2];
         raw[54] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.ground_speed_north;
         raw[55] = __convert_float32_to_bytes.b[0];
         raw[56] = __convert_float32_to_bytes.b[1];
         raw[57] = __convert_float32_to_bytes.b[2];
         raw[58] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.ground_speed_east;
         raw[59] = __convert_float32_to_bytes.b[0];
         raw[60] = __convert_float32_to_bytes.b[1];
         raw[61] = __convert_float32_to_bytes.b[2];
         raw[62] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.geometric_vertical_rate;
         raw[63] = __convert_float32_to_bytes.b[0];
         raw[64] = __convert_float32_to_bytes.b[1];
         raw[65] = __convert_float32_to_bytes.b[2];
         raw[66] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.pressure_vertical_rate;
         raw[67] = __convert_float32_to_bytes.b[0];
         raw[68] = __convert_float32_to_bytes.b[1];
         raw[69] = __convert_float32_to_bytes.b[2];
         raw[70] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.heading_rate;
         raw[71] = __convert_float32_to_bytes.b[0];
         raw[72] = __convert_float32_to_bytes.b[1];
         raw[73] = __convert_float32_to_bytes.b[2];
         raw[74] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.ground_track_rate;
         raw[75] = __convert_float32_to_bytes.b[0];
         raw[76] = __convert_float32_to_bytes.b[1];
         raw[77] = __convert_float32_to_bytes.b[2];
         raw[78] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[79] = u.b[0];
         raw[80] = u.b[1];
         raw[81] = u.b[2];
         raw[82] = u.b[3];
      } 
      { //write data.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.horizonal_position_uncertainty;
         raw[83] = __convert_uint16_to_bytes.b[0];
         raw[84] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.pressure_altitude_uncertainty;
         raw[85] = __convert_uint16_to_bytes.b[0];
         raw[86] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.geometric_altitude_uncertainty;
         raw[87] = __convert_uint16_to_bytes.b[0];
         raw[88] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.ground_speed_uncertainty;
         raw[89] = __convert_float32_to_bytes.b[0];
         raw[90] = __convert_float32_to_bytes.b[1];
         raw[91] = __convert_float32_to_bytes.b[2];
         raw[92] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.airspeed_uncertainty;
         raw[93] = __convert_float32_to_bytes.b[0];
         raw[94] = __convert_float32_to_bytes.b[1];
         raw[95] = __convert_float32_to_bytes.b[2];
         raw[96] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.heading_uncertainty;
         raw[97] = __convert_float32_to_bytes.b[0];
         raw[98] = __convert_float32_to_bytes.b[1];
         raw[99] = __convert_float32_to_bytes.b[2];
         raw[100] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.ground_track_angle_uncertainty;
         raw[101] = __convert_float32_to_bytes.b[0];
         raw[102] = __convert_float32_to_bytes.b[1];
         raw[103] = __convert_float32_to_bytes.b[2];
         raw[104] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.heading_rate_uncertainty;
         raw[105] = __convert_float32_to_bytes.b[0];
         raw[106] = __convert_float32_to_bytes.b[1];
         raw[107] = __convert_float32_to_bytes.b[2];
         raw[108] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.ground_track_rate_uncertainty;
         raw[109] = __convert_float32_to_bytes.b[0];
         raw[110] = __convert_float32_to_bytes.b[1];
         raw[111] = __convert_float32_to_bytes.b[2];
         raw[112] = __convert_float32_to_bytes.b[3];
      } 
   }
   void write_track(const track& data, track_buffer& raw) {
      write_track_ptr(data, &(raw[0]));
   };
   void write_track_ptr(const track& data,  char* raw) {
      { //write data.track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.track_id;
         raw[0] = __convert_uint32_to_bytes.b[0];
         raw[1] = __convert_uint32_to_bytes.b[1];
         raw[2] = __convert_uint32_to_bytes.b[2];
         raw[3] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.callsign        text        1 bytes
         __convert_text_to_bytes.a = data.callsign[0];
         raw[4] = __convert_text_to_bytes.b[0];
      } 
      { //write data.callsign        text        1 bytes
         __convert_text_to_bytes.a = data.callsign[1];
         raw[5] = __convert_text_to_bytes.b[0];
      } 
      { //write data.callsign        text        1 bytes
         __convert_text_to_bytes.a = data.callsign[2];
         raw[6] = __convert_text_to_bytes.b[0];
      } 
      { //write data.callsign        text        1 bytes
         __convert_text_to_bytes.a = data.callsign[3];
         raw[7] = __convert_text_to_bytes.b[0];
      } 
      { //write data.callsign        text        1 bytes
         __convert_text_to_bytes.a = data.callsign[4];
         raw[8] = __convert_text_to_bytes.b[0];
      } 
      { //write data.callsign        text        1 bytes
         __convert_text_to_bytes.a = data.callsign[5];
         raw[9] = __convert_text_to_bytes.b[0];
      } 
      { //write data.callsign        text        1 bytes
         __convert_text_to_bytes.a = data.callsign[6];
         raw[10] = __convert_text_to_bytes.b[0];
      } 
      { //write data.callsign        text        1 bytes
         __convert_text_to_bytes.a = data.callsign[7];
         raw[11] = __convert_text_to_bytes.b[0];
      } 
      { //write data.track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.track_time;
         raw[12] = __convert_uint64_to_bytes.b[0];
         raw[13] = __convert_uint64_to_bytes.b[1];
         raw[14] = __convert_uint64_to_bytes.b[2];
         raw[15] = __convert_uint64_to_bytes.b[3];
         raw[16] = __convert_uint64_to_bytes.b[4];
         raw[17] = __convert_uint64_to_bytes.b[5];
         raw[18] = __convert_uint64_to_bytes.b[6];
         raw[19] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[20] = u.b[0];
         raw[21] = u.b[1];
         raw[22] = u.b[2];
         raw[23] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[24] = u.b[0];
         raw[25] = u.b[1];
         raw[26] = u.b[2];
         raw[27] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[28] = bits[0];
         raw[29] = bits[1];
      } 
      { //write data.track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.track_state.relative_range;
         raw[30] = __convert_float32_to_bytes.b[0];
         raw[31] = __convert_float32_to_bytes.b[1];
         raw[32] = __convert_float32_to_bytes.b[2];
         raw[33] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.track_state.relative_bearing;
         raw[34] = __convert_float32_to_bytes.b[0];
         raw[35] = __convert_float32_to_bytes.b[1];
         raw[36] = __convert_float32_to_bytes.b[2];
         raw[37] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.track_state.bearing_invalid;
         raw[38] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.track_state.relative_ground_speed_north;
         raw[39] = __convert_float32_to_bytes.b[0];
         raw[40] = __convert_float32_to_bytes.b[1];
         raw[41] = __convert_float32_to_bytes.b[2];
         raw[42] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.track_state.relative_ground_speed_east;
         raw[43] = __convert_float32_to_bytes.b[0];
         raw[44] = __convert_float32_to_bytes.b[1];
         raw[45] = __convert_float32_to_bytes.b[2];
         raw[46] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.track_state.relative_altitude;
         raw[47] = __convert_int16_to_bytes.b[0];
         raw[48] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.track_state.relative_vertical_speed;
         raw[49] = __convert_float32_to_bytes.b[0];
         raw[50] = __convert_float32_to_bytes.b[1];
         raw[51] = __convert_float32_to_bytes.b[2];
         raw[52] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[53] = u.b[0];
         raw[54] = u.b[1];
         raw[55] = u.b[2];
         raw[56] = u.b[3];
      } 
      { //write data.track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.track_state.relative_range_uncertainty;
         raw[57] = __convert_uint32_to_bytes.b[0];
         raw[58] = __convert_uint32_to_bytes.b[1];
         raw[59] = __convert_uint32_to_bytes.b[2];
         raw[60] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.track_state.relative_bearing_uncertainty;
         raw[61] = __convert_float32_to_bytes.b[0];
         raw[62] = __convert_float32_to_bytes.b[1];
         raw[63] = __convert_float32_to_bytes.b[2];
         raw[64] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.track_state.relative_ground_speed_north_uncertainty;
         raw[65] = __convert_float32_to_bytes.b[0];
         raw[66] = __convert_float32_to_bytes.b[1];
         raw[67] = __convert_float32_to_bytes.b[2];
         raw[68] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.track_state.relative_ground_speed_east_uncertainty;
         raw[69] = __convert_float32_to_bytes.b[0];
         raw[70] = __convert_float32_to_bytes.b[1];
         raw[71] = __convert_float32_to_bytes.b[2];
         raw[72] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.track_state.relative_altitude_uncertainty;
         raw[73] = __convert_int16_to_bytes.b[0];
         raw[74] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.track_state.relative_vertical_speed_uncertainty;
         raw[75] = __convert_float32_to_bytes.b[0];
         raw[76] = __convert_float32_to_bytes.b[1];
         raw[77] = __convert_float32_to_bytes.b[2];
         raw[78] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[79] = bits[0];
         raw[80] = bits[1];
         raw[81] = bits[2];
      } 
      { //write data.aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.aircraft_state.toa;
         raw[82] = __convert_uint64_to_bytes.b[0];
         raw[83] = __convert_uint64_to_bytes.b[1];
         raw[84] = __convert_uint64_to_bytes.b[2];
         raw[85] = __convert_uint64_to_bytes.b[3];
         raw[86] = __convert_uint64_to_bytes.b[4];
         raw[87] = __convert_uint64_to_bytes.b[5];
         raw[88] = __convert_uint64_to_bytes.b[6];
         raw[89] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.aircraft_state.longitude;
         raw[90] = __convert_float64_to_bytes.b[0];
         raw[91] = __convert_float64_to_bytes.b[1];
         raw[92] = __convert_float64_to_bytes.b[2];
         raw[93] = __convert_float64_to_bytes.b[3];
         raw[94] = __convert_float64_to_bytes.b[4];
         raw[95] = __convert_float64_to_bytes.b[5];
         raw[96] = __convert_float64_to_bytes.b[6];
         raw[97] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.aircraft_state.latitude;
         raw[98] = __convert_float64_to_bytes.b[0];
         raw[99] = __convert_float64_to_bytes.b[1];
         raw[100] = __convert_float64_to_bytes.b[2];
         raw[101] = __convert_float64_to_bytes.b[3];
         raw[102] = __convert_float64_to_bytes.b[4];
         raw[103] = __convert_float64_to_bytes.b[5];
         raw[104] = __convert_float64_to_bytes.b[6];
         raw[105] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.geometric_altitude;
         raw[106] = __convert_float32_to_bytes.b[0];
         raw[107] = __convert_float32_to_bytes.b[1];
         raw[108] = __convert_float32_to_bytes.b[2];
         raw[109] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.barometric_altitude;
         raw[110] = __convert_float32_to_bytes.b[0];
         raw[111] = __convert_float32_to_bytes.b[1];
         raw[112] = __convert_float32_to_bytes.b[2];
         raw[113] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.true_heading;
         raw[114] = __convert_float32_to_bytes.b[0];
         raw[115] = __convert_float32_to_bytes.b[1];
         raw[116] = __convert_float32_to_bytes.b[2];
         raw[117] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.magnetic_heading;
         raw[118] = __convert_float32_to_bytes.b[0];
         raw[119] = __convert_float32_to_bytes.b[1];
         raw[120] = __convert_float32_to_bytes.b[2];
         raw[121] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.ground_track_angle;
         raw[122] = __convert_float32_to_bytes.b[0];
         raw[123] = __convert_float32_to_bytes.b[1];
         raw[124] = __convert_float32_to_bytes.b[2];
         raw[125] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.true_air_speed;
         raw[126] = __convert_float32_to_bytes.b[0];
         raw[127] = __convert_float32_to_bytes.b[1];
         raw[128] = __convert_float32_to_bytes.b[2];
         raw[129] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.indicated_air_speed;
         raw[130] = __convert_float32_to_bytes.b[0];
         raw[131] = __convert_float32_to_bytes.b[1];
         raw[132] = __convert_float32_to_bytes.b[2];
         raw[133] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.ground_speed_north;
         raw[134] = __convert_float32_to_bytes.b[0];
         raw[135] = __convert_float32_to_bytes.b[1];
         raw[136] = __convert_float32_to_bytes.b[2];
         raw[137] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.ground_speed_east;
         raw[138] = __convert_float32_to_bytes.b[0];
         raw[139] = __convert_float32_to_bytes.b[1];
         raw[140] = __convert_float32_to_bytes.b[2];
         raw[141] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.geometric_vertical_rate;
         raw[142] = __convert_float32_to_bytes.b[0];
         raw[143] = __convert_float32_to_bytes.b[1];
         raw[144] = __convert_float32_to_bytes.b[2];
         raw[145] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.pressure_vertical_rate;
         raw[146] = __convert_float32_to_bytes.b[0];
         raw[147] = __convert_float32_to_bytes.b[1];
         raw[148] = __convert_float32_to_bytes.b[2];
         raw[149] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.heading_rate;
         raw[150] = __convert_float32_to_bytes.b[0];
         raw[151] = __convert_float32_to_bytes.b[1];
         raw[152] = __convert_float32_to_bytes.b[2];
         raw[153] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.ground_track_rate;
         raw[154] = __convert_float32_to_bytes.b[0];
         raw[155] = __convert_float32_to_bytes.b[1];
         raw[156] = __convert_float32_to_bytes.b[2];
         raw[157] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[158] = u.b[0];
         raw[159] = u.b[1];
         raw[160] = u.b[2];
         raw[161] = u.b[3];
      } 
      { //write data.aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.aircraft_state.horizonal_position_uncertainty;
         raw[162] = __convert_uint16_to_bytes.b[0];
         raw[163] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.aircraft_state.pressure_altitude_uncertainty;
         raw[164] = __convert_uint16_to_bytes.b[0];
         raw[165] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.aircraft_state.geometric_altitude_uncertainty;
         raw[166] = __convert_uint16_to_bytes.b[0];
         raw[167] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.ground_speed_uncertainty;
         raw[168] = __convert_float32_to_bytes.b[0];
         raw[169] = __convert_float32_to_bytes.b[1];
         raw[170] = __convert_float32_to_bytes.b[2];
         raw[171] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.airspeed_uncertainty;
         raw[172] = __convert_float32_to_bytes.b[0];
         raw[173] = __convert_float32_to_bytes.b[1];
         raw[174] = __convert_float32_to_bytes.b[2];
         raw[175] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.heading_uncertainty;
         raw[176] = __convert_float32_to_bytes.b[0];
         raw[177] = __convert_float32_to_bytes.b[1];
         raw[178] = __convert_float32_to_bytes.b[2];
         raw[179] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.ground_track_angle_uncertainty;
         raw[180] = __convert_float32_to_bytes.b[0];
         raw[181] = __convert_float32_to_bytes.b[1];
         raw[182] = __convert_float32_to_bytes.b[2];
         raw[183] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.heading_rate_uncertainty;
         raw[184] = __convert_float32_to_bytes.b[0];
         raw[185] = __convert_float32_to_bytes.b[1];
         raw[186] = __convert_float32_to_bytes.b[2];
         raw[187] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.aircraft_state.ground_track_rate_uncertainty;
         raw[188] = __convert_float32_to_bytes.b[0];
         raw[189] = __convert_float32_to_bytes.b[1];
         raw[190] = __convert_float32_to_bytes.b[2];
         raw[191] = __convert_float32_to_bytes.b[3];
      } 
   }
   void write_track_list(const track_list& data, track_list_buffer& raw) {
      write_track_list_ptr(data, &(raw[0]));
   };
   void write_track_list_ptr(const track_list& data,  char* raw) {
      { //write data.tracks[0].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[0].track_id;
         raw[0] = __convert_uint32_to_bytes.b[0];
         raw[1] = __convert_uint32_to_bytes.b[1];
         raw[2] = __convert_uint32_to_bytes.b[2];
         raw[3] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[0].callsign[0];
         raw[4] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[0].callsign[1];
         raw[5] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[0].callsign[2];
         raw[6] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[0].callsign[3];
         raw[7] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[0].callsign[4];
         raw[8] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[0].callsign[5];
         raw[9] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[0].callsign[6];
         raw[10] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[0].callsign[7];
         raw[11] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[0].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[0].track_time;
         raw[12] = __convert_uint64_to_bytes.b[0];
         raw[13] = __convert_uint64_to_bytes.b[1];
         raw[14] = __convert_uint64_to_bytes.b[2];
         raw[15] = __convert_uint64_to_bytes.b[3];
         raw[16] = __convert_uint64_to_bytes.b[4];
         raw[17] = __convert_uint64_to_bytes.b[5];
         raw[18] = __convert_uint64_to_bytes.b[6];
         raw[19] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[0].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[20] = u.b[0];
         raw[21] = u.b[1];
         raw[22] = u.b[2];
         raw[23] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[0].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[24] = u.b[0];
         raw[25] = u.b[1];
         raw[26] = u.b[2];
         raw[27] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[0].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[0].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[0].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[0].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[0].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[0].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[0].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[0].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[0].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[28] = bits[0];
         raw[29] = bits[1];
      } 
      { //write data.tracks[0].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].track_state.relative_range;
         raw[30] = __convert_float32_to_bytes.b[0];
         raw[31] = __convert_float32_to_bytes.b[1];
         raw[32] = __convert_float32_to_bytes.b[2];
         raw[33] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].track_state.relative_bearing;
         raw[34] = __convert_float32_to_bytes.b[0];
         raw[35] = __convert_float32_to_bytes.b[1];
         raw[36] = __convert_float32_to_bytes.b[2];
         raw[37] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[0].track_state.bearing_invalid;
         raw[38] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[0].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].track_state.relative_ground_speed_north;
         raw[39] = __convert_float32_to_bytes.b[0];
         raw[40] = __convert_float32_to_bytes.b[1];
         raw[41] = __convert_float32_to_bytes.b[2];
         raw[42] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].track_state.relative_ground_speed_east;
         raw[43] = __convert_float32_to_bytes.b[0];
         raw[44] = __convert_float32_to_bytes.b[1];
         raw[45] = __convert_float32_to_bytes.b[2];
         raw[46] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[0].track_state.relative_altitude;
         raw[47] = __convert_int16_to_bytes.b[0];
         raw[48] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[0].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].track_state.relative_vertical_speed;
         raw[49] = __convert_float32_to_bytes.b[0];
         raw[50] = __convert_float32_to_bytes.b[1];
         raw[51] = __convert_float32_to_bytes.b[2];
         raw[52] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[0].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[53] = u.b[0];
         raw[54] = u.b[1];
         raw[55] = u.b[2];
         raw[56] = u.b[3];
      } 
      { //write data.tracks[0].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[0].track_state.relative_range_uncertainty;
         raw[57] = __convert_uint32_to_bytes.b[0];
         raw[58] = __convert_uint32_to_bytes.b[1];
         raw[59] = __convert_uint32_to_bytes.b[2];
         raw[60] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[0].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].track_state.relative_bearing_uncertainty;
         raw[61] = __convert_float32_to_bytes.b[0];
         raw[62] = __convert_float32_to_bytes.b[1];
         raw[63] = __convert_float32_to_bytes.b[2];
         raw[64] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].track_state.relative_ground_speed_north_uncertainty;
         raw[65] = __convert_float32_to_bytes.b[0];
         raw[66] = __convert_float32_to_bytes.b[1];
         raw[67] = __convert_float32_to_bytes.b[2];
         raw[68] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].track_state.relative_ground_speed_east_uncertainty;
         raw[69] = __convert_float32_to_bytes.b[0];
         raw[70] = __convert_float32_to_bytes.b[1];
         raw[71] = __convert_float32_to_bytes.b[2];
         raw[72] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[0].track_state.relative_altitude_uncertainty;
         raw[73] = __convert_int16_to_bytes.b[0];
         raw[74] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[0].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].track_state.relative_vertical_speed_uncertainty;
         raw[75] = __convert_float32_to_bytes.b[0];
         raw[76] = __convert_float32_to_bytes.b[1];
         raw[77] = __convert_float32_to_bytes.b[2];
         raw[78] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[0].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[0].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[0].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[0].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[0].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[0].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[0].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[0].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[0].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[0].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[0].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[0].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[0].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[0].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[0].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[0].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[0].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[79] = bits[0];
         raw[80] = bits[1];
         raw[81] = bits[2];
      } 
      { //write data.tracks[0].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[0].aircraft_state.toa;
         raw[82] = __convert_uint64_to_bytes.b[0];
         raw[83] = __convert_uint64_to_bytes.b[1];
         raw[84] = __convert_uint64_to_bytes.b[2];
         raw[85] = __convert_uint64_to_bytes.b[3];
         raw[86] = __convert_uint64_to_bytes.b[4];
         raw[87] = __convert_uint64_to_bytes.b[5];
         raw[88] = __convert_uint64_to_bytes.b[6];
         raw[89] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[0].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[0].aircraft_state.longitude;
         raw[90] = __convert_float64_to_bytes.b[0];
         raw[91] = __convert_float64_to_bytes.b[1];
         raw[92] = __convert_float64_to_bytes.b[2];
         raw[93] = __convert_float64_to_bytes.b[3];
         raw[94] = __convert_float64_to_bytes.b[4];
         raw[95] = __convert_float64_to_bytes.b[5];
         raw[96] = __convert_float64_to_bytes.b[6];
         raw[97] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[0].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[0].aircraft_state.latitude;
         raw[98] = __convert_float64_to_bytes.b[0];
         raw[99] = __convert_float64_to_bytes.b[1];
         raw[100] = __convert_float64_to_bytes.b[2];
         raw[101] = __convert_float64_to_bytes.b[3];
         raw[102] = __convert_float64_to_bytes.b[4];
         raw[103] = __convert_float64_to_bytes.b[5];
         raw[104] = __convert_float64_to_bytes.b[6];
         raw[105] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[0].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.geometric_altitude;
         raw[106] = __convert_float32_to_bytes.b[0];
         raw[107] = __convert_float32_to_bytes.b[1];
         raw[108] = __convert_float32_to_bytes.b[2];
         raw[109] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.barometric_altitude;
         raw[110] = __convert_float32_to_bytes.b[0];
         raw[111] = __convert_float32_to_bytes.b[1];
         raw[112] = __convert_float32_to_bytes.b[2];
         raw[113] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.true_heading;
         raw[114] = __convert_float32_to_bytes.b[0];
         raw[115] = __convert_float32_to_bytes.b[1];
         raw[116] = __convert_float32_to_bytes.b[2];
         raw[117] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.magnetic_heading;
         raw[118] = __convert_float32_to_bytes.b[0];
         raw[119] = __convert_float32_to_bytes.b[1];
         raw[120] = __convert_float32_to_bytes.b[2];
         raw[121] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.ground_track_angle;
         raw[122] = __convert_float32_to_bytes.b[0];
         raw[123] = __convert_float32_to_bytes.b[1];
         raw[124] = __convert_float32_to_bytes.b[2];
         raw[125] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.true_air_speed;
         raw[126] = __convert_float32_to_bytes.b[0];
         raw[127] = __convert_float32_to_bytes.b[1];
         raw[128] = __convert_float32_to_bytes.b[2];
         raw[129] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.indicated_air_speed;
         raw[130] = __convert_float32_to_bytes.b[0];
         raw[131] = __convert_float32_to_bytes.b[1];
         raw[132] = __convert_float32_to_bytes.b[2];
         raw[133] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.ground_speed_north;
         raw[134] = __convert_float32_to_bytes.b[0];
         raw[135] = __convert_float32_to_bytes.b[1];
         raw[136] = __convert_float32_to_bytes.b[2];
         raw[137] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.ground_speed_east;
         raw[138] = __convert_float32_to_bytes.b[0];
         raw[139] = __convert_float32_to_bytes.b[1];
         raw[140] = __convert_float32_to_bytes.b[2];
         raw[141] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.geometric_vertical_rate;
         raw[142] = __convert_float32_to_bytes.b[0];
         raw[143] = __convert_float32_to_bytes.b[1];
         raw[144] = __convert_float32_to_bytes.b[2];
         raw[145] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.pressure_vertical_rate;
         raw[146] = __convert_float32_to_bytes.b[0];
         raw[147] = __convert_float32_to_bytes.b[1];
         raw[148] = __convert_float32_to_bytes.b[2];
         raw[149] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.heading_rate;
         raw[150] = __convert_float32_to_bytes.b[0];
         raw[151] = __convert_float32_to_bytes.b[1];
         raw[152] = __convert_float32_to_bytes.b[2];
         raw[153] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.ground_track_rate;
         raw[154] = __convert_float32_to_bytes.b[0];
         raw[155] = __convert_float32_to_bytes.b[1];
         raw[156] = __convert_float32_to_bytes.b[2];
         raw[157] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[0].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[158] = u.b[0];
         raw[159] = u.b[1];
         raw[160] = u.b[2];
         raw[161] = u.b[3];
      } 
      { //write data.tracks[0].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[0].aircraft_state.horizonal_position_uncertainty;
         raw[162] = __convert_uint16_to_bytes.b[0];
         raw[163] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[0].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[0].aircraft_state.pressure_altitude_uncertainty;
         raw[164] = __convert_uint16_to_bytes.b[0];
         raw[165] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[0].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[0].aircraft_state.geometric_altitude_uncertainty;
         raw[166] = __convert_uint16_to_bytes.b[0];
         raw[167] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[0].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.ground_speed_uncertainty;
         raw[168] = __convert_float32_to_bytes.b[0];
         raw[169] = __convert_float32_to_bytes.b[1];
         raw[170] = __convert_float32_to_bytes.b[2];
         raw[171] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.airspeed_uncertainty;
         raw[172] = __convert_float32_to_bytes.b[0];
         raw[173] = __convert_float32_to_bytes.b[1];
         raw[174] = __convert_float32_to_bytes.b[2];
         raw[175] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.heading_uncertainty;
         raw[176] = __convert_float32_to_bytes.b[0];
         raw[177] = __convert_float32_to_bytes.b[1];
         raw[178] = __convert_float32_to_bytes.b[2];
         raw[179] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.ground_track_angle_uncertainty;
         raw[180] = __convert_float32_to_bytes.b[0];
         raw[181] = __convert_float32_to_bytes.b[1];
         raw[182] = __convert_float32_to_bytes.b[2];
         raw[183] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.heading_rate_uncertainty;
         raw[184] = __convert_float32_to_bytes.b[0];
         raw[185] = __convert_float32_to_bytes.b[1];
         raw[186] = __convert_float32_to_bytes.b[2];
         raw[187] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[0].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[0].aircraft_state.ground_track_rate_uncertainty;
         raw[188] = __convert_float32_to_bytes.b[0];
         raw[189] = __convert_float32_to_bytes.b[1];
         raw[190] = __convert_float32_to_bytes.b[2];
         raw[191] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[1].track_id;
         raw[192] = __convert_uint32_to_bytes.b[0];
         raw[193] = __convert_uint32_to_bytes.b[1];
         raw[194] = __convert_uint32_to_bytes.b[2];
         raw[195] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[1].callsign[0];
         raw[196] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[1].callsign[1];
         raw[197] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[1].callsign[2];
         raw[198] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[1].callsign[3];
         raw[199] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[1].callsign[4];
         raw[200] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[1].callsign[5];
         raw[201] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[1].callsign[6];
         raw[202] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[1].callsign[7];
         raw[203] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[1].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[1].track_time;
         raw[204] = __convert_uint64_to_bytes.b[0];
         raw[205] = __convert_uint64_to_bytes.b[1];
         raw[206] = __convert_uint64_to_bytes.b[2];
         raw[207] = __convert_uint64_to_bytes.b[3];
         raw[208] = __convert_uint64_to_bytes.b[4];
         raw[209] = __convert_uint64_to_bytes.b[5];
         raw[210] = __convert_uint64_to_bytes.b[6];
         raw[211] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[1].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[212] = u.b[0];
         raw[213] = u.b[1];
         raw[214] = u.b[2];
         raw[215] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[1].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[216] = u.b[0];
         raw[217] = u.b[1];
         raw[218] = u.b[2];
         raw[219] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[1].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[1].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[1].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[1].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[1].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[1].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[1].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[1].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[1].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[220] = bits[0];
         raw[221] = bits[1];
      } 
      { //write data.tracks[1].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].track_state.relative_range;
         raw[222] = __convert_float32_to_bytes.b[0];
         raw[223] = __convert_float32_to_bytes.b[1];
         raw[224] = __convert_float32_to_bytes.b[2];
         raw[225] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].track_state.relative_bearing;
         raw[226] = __convert_float32_to_bytes.b[0];
         raw[227] = __convert_float32_to_bytes.b[1];
         raw[228] = __convert_float32_to_bytes.b[2];
         raw[229] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[1].track_state.bearing_invalid;
         raw[230] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[1].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].track_state.relative_ground_speed_north;
         raw[231] = __convert_float32_to_bytes.b[0];
         raw[232] = __convert_float32_to_bytes.b[1];
         raw[233] = __convert_float32_to_bytes.b[2];
         raw[234] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].track_state.relative_ground_speed_east;
         raw[235] = __convert_float32_to_bytes.b[0];
         raw[236] = __convert_float32_to_bytes.b[1];
         raw[237] = __convert_float32_to_bytes.b[2];
         raw[238] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[1].track_state.relative_altitude;
         raw[239] = __convert_int16_to_bytes.b[0];
         raw[240] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[1].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].track_state.relative_vertical_speed;
         raw[241] = __convert_float32_to_bytes.b[0];
         raw[242] = __convert_float32_to_bytes.b[1];
         raw[243] = __convert_float32_to_bytes.b[2];
         raw[244] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[1].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[245] = u.b[0];
         raw[246] = u.b[1];
         raw[247] = u.b[2];
         raw[248] = u.b[3];
      } 
      { //write data.tracks[1].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[1].track_state.relative_range_uncertainty;
         raw[249] = __convert_uint32_to_bytes.b[0];
         raw[250] = __convert_uint32_to_bytes.b[1];
         raw[251] = __convert_uint32_to_bytes.b[2];
         raw[252] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[1].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].track_state.relative_bearing_uncertainty;
         raw[253] = __convert_float32_to_bytes.b[0];
         raw[254] = __convert_float32_to_bytes.b[1];
         raw[255] = __convert_float32_to_bytes.b[2];
         raw[256] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].track_state.relative_ground_speed_north_uncertainty;
         raw[257] = __convert_float32_to_bytes.b[0];
         raw[258] = __convert_float32_to_bytes.b[1];
         raw[259] = __convert_float32_to_bytes.b[2];
         raw[260] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].track_state.relative_ground_speed_east_uncertainty;
         raw[261] = __convert_float32_to_bytes.b[0];
         raw[262] = __convert_float32_to_bytes.b[1];
         raw[263] = __convert_float32_to_bytes.b[2];
         raw[264] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[1].track_state.relative_altitude_uncertainty;
         raw[265] = __convert_int16_to_bytes.b[0];
         raw[266] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[1].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].track_state.relative_vertical_speed_uncertainty;
         raw[267] = __convert_float32_to_bytes.b[0];
         raw[268] = __convert_float32_to_bytes.b[1];
         raw[269] = __convert_float32_to_bytes.b[2];
         raw[270] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[1].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[1].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[1].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[1].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[1].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[1].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[1].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[1].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[1].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[1].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[1].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[1].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[1].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[1].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[1].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[1].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[1].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[271] = bits[0];
         raw[272] = bits[1];
         raw[273] = bits[2];
      } 
      { //write data.tracks[1].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[1].aircraft_state.toa;
         raw[274] = __convert_uint64_to_bytes.b[0];
         raw[275] = __convert_uint64_to_bytes.b[1];
         raw[276] = __convert_uint64_to_bytes.b[2];
         raw[277] = __convert_uint64_to_bytes.b[3];
         raw[278] = __convert_uint64_to_bytes.b[4];
         raw[279] = __convert_uint64_to_bytes.b[5];
         raw[280] = __convert_uint64_to_bytes.b[6];
         raw[281] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[1].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[1].aircraft_state.longitude;
         raw[282] = __convert_float64_to_bytes.b[0];
         raw[283] = __convert_float64_to_bytes.b[1];
         raw[284] = __convert_float64_to_bytes.b[2];
         raw[285] = __convert_float64_to_bytes.b[3];
         raw[286] = __convert_float64_to_bytes.b[4];
         raw[287] = __convert_float64_to_bytes.b[5];
         raw[288] = __convert_float64_to_bytes.b[6];
         raw[289] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[1].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[1].aircraft_state.latitude;
         raw[290] = __convert_float64_to_bytes.b[0];
         raw[291] = __convert_float64_to_bytes.b[1];
         raw[292] = __convert_float64_to_bytes.b[2];
         raw[293] = __convert_float64_to_bytes.b[3];
         raw[294] = __convert_float64_to_bytes.b[4];
         raw[295] = __convert_float64_to_bytes.b[5];
         raw[296] = __convert_float64_to_bytes.b[6];
         raw[297] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[1].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.geometric_altitude;
         raw[298] = __convert_float32_to_bytes.b[0];
         raw[299] = __convert_float32_to_bytes.b[1];
         raw[300] = __convert_float32_to_bytes.b[2];
         raw[301] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.barometric_altitude;
         raw[302] = __convert_float32_to_bytes.b[0];
         raw[303] = __convert_float32_to_bytes.b[1];
         raw[304] = __convert_float32_to_bytes.b[2];
         raw[305] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.true_heading;
         raw[306] = __convert_float32_to_bytes.b[0];
         raw[307] = __convert_float32_to_bytes.b[1];
         raw[308] = __convert_float32_to_bytes.b[2];
         raw[309] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.magnetic_heading;
         raw[310] = __convert_float32_to_bytes.b[0];
         raw[311] = __convert_float32_to_bytes.b[1];
         raw[312] = __convert_float32_to_bytes.b[2];
         raw[313] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.ground_track_angle;
         raw[314] = __convert_float32_to_bytes.b[0];
         raw[315] = __convert_float32_to_bytes.b[1];
         raw[316] = __convert_float32_to_bytes.b[2];
         raw[317] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.true_air_speed;
         raw[318] = __convert_float32_to_bytes.b[0];
         raw[319] = __convert_float32_to_bytes.b[1];
         raw[320] = __convert_float32_to_bytes.b[2];
         raw[321] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.indicated_air_speed;
         raw[322] = __convert_float32_to_bytes.b[0];
         raw[323] = __convert_float32_to_bytes.b[1];
         raw[324] = __convert_float32_to_bytes.b[2];
         raw[325] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.ground_speed_north;
         raw[326] = __convert_float32_to_bytes.b[0];
         raw[327] = __convert_float32_to_bytes.b[1];
         raw[328] = __convert_float32_to_bytes.b[2];
         raw[329] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.ground_speed_east;
         raw[330] = __convert_float32_to_bytes.b[0];
         raw[331] = __convert_float32_to_bytes.b[1];
         raw[332] = __convert_float32_to_bytes.b[2];
         raw[333] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.geometric_vertical_rate;
         raw[334] = __convert_float32_to_bytes.b[0];
         raw[335] = __convert_float32_to_bytes.b[1];
         raw[336] = __convert_float32_to_bytes.b[2];
         raw[337] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.pressure_vertical_rate;
         raw[338] = __convert_float32_to_bytes.b[0];
         raw[339] = __convert_float32_to_bytes.b[1];
         raw[340] = __convert_float32_to_bytes.b[2];
         raw[341] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.heading_rate;
         raw[342] = __convert_float32_to_bytes.b[0];
         raw[343] = __convert_float32_to_bytes.b[1];
         raw[344] = __convert_float32_to_bytes.b[2];
         raw[345] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.ground_track_rate;
         raw[346] = __convert_float32_to_bytes.b[0];
         raw[347] = __convert_float32_to_bytes.b[1];
         raw[348] = __convert_float32_to_bytes.b[2];
         raw[349] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[1].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[350] = u.b[0];
         raw[351] = u.b[1];
         raw[352] = u.b[2];
         raw[353] = u.b[3];
      } 
      { //write data.tracks[1].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[1].aircraft_state.horizonal_position_uncertainty;
         raw[354] = __convert_uint16_to_bytes.b[0];
         raw[355] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[1].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[1].aircraft_state.pressure_altitude_uncertainty;
         raw[356] = __convert_uint16_to_bytes.b[0];
         raw[357] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[1].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[1].aircraft_state.geometric_altitude_uncertainty;
         raw[358] = __convert_uint16_to_bytes.b[0];
         raw[359] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[1].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.ground_speed_uncertainty;
         raw[360] = __convert_float32_to_bytes.b[0];
         raw[361] = __convert_float32_to_bytes.b[1];
         raw[362] = __convert_float32_to_bytes.b[2];
         raw[363] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.airspeed_uncertainty;
         raw[364] = __convert_float32_to_bytes.b[0];
         raw[365] = __convert_float32_to_bytes.b[1];
         raw[366] = __convert_float32_to_bytes.b[2];
         raw[367] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.heading_uncertainty;
         raw[368] = __convert_float32_to_bytes.b[0];
         raw[369] = __convert_float32_to_bytes.b[1];
         raw[370] = __convert_float32_to_bytes.b[2];
         raw[371] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.ground_track_angle_uncertainty;
         raw[372] = __convert_float32_to_bytes.b[0];
         raw[373] = __convert_float32_to_bytes.b[1];
         raw[374] = __convert_float32_to_bytes.b[2];
         raw[375] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.heading_rate_uncertainty;
         raw[376] = __convert_float32_to_bytes.b[0];
         raw[377] = __convert_float32_to_bytes.b[1];
         raw[378] = __convert_float32_to_bytes.b[2];
         raw[379] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[1].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[1].aircraft_state.ground_track_rate_uncertainty;
         raw[380] = __convert_float32_to_bytes.b[0];
         raw[381] = __convert_float32_to_bytes.b[1];
         raw[382] = __convert_float32_to_bytes.b[2];
         raw[383] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[2].track_id;
         raw[384] = __convert_uint32_to_bytes.b[0];
         raw[385] = __convert_uint32_to_bytes.b[1];
         raw[386] = __convert_uint32_to_bytes.b[2];
         raw[387] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[2].callsign[0];
         raw[388] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[2].callsign[1];
         raw[389] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[2].callsign[2];
         raw[390] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[2].callsign[3];
         raw[391] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[2].callsign[4];
         raw[392] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[2].callsign[5];
         raw[393] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[2].callsign[6];
         raw[394] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[2].callsign[7];
         raw[395] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[2].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[2].track_time;
         raw[396] = __convert_uint64_to_bytes.b[0];
         raw[397] = __convert_uint64_to_bytes.b[1];
         raw[398] = __convert_uint64_to_bytes.b[2];
         raw[399] = __convert_uint64_to_bytes.b[3];
         raw[400] = __convert_uint64_to_bytes.b[4];
         raw[401] = __convert_uint64_to_bytes.b[5];
         raw[402] = __convert_uint64_to_bytes.b[6];
         raw[403] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[2].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[404] = u.b[0];
         raw[405] = u.b[1];
         raw[406] = u.b[2];
         raw[407] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[2].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[408] = u.b[0];
         raw[409] = u.b[1];
         raw[410] = u.b[2];
         raw[411] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[2].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[2].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[2].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[2].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[2].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[2].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[2].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[2].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[2].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[412] = bits[0];
         raw[413] = bits[1];
      } 
      { //write data.tracks[2].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].track_state.relative_range;
         raw[414] = __convert_float32_to_bytes.b[0];
         raw[415] = __convert_float32_to_bytes.b[1];
         raw[416] = __convert_float32_to_bytes.b[2];
         raw[417] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].track_state.relative_bearing;
         raw[418] = __convert_float32_to_bytes.b[0];
         raw[419] = __convert_float32_to_bytes.b[1];
         raw[420] = __convert_float32_to_bytes.b[2];
         raw[421] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[2].track_state.bearing_invalid;
         raw[422] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[2].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].track_state.relative_ground_speed_north;
         raw[423] = __convert_float32_to_bytes.b[0];
         raw[424] = __convert_float32_to_bytes.b[1];
         raw[425] = __convert_float32_to_bytes.b[2];
         raw[426] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].track_state.relative_ground_speed_east;
         raw[427] = __convert_float32_to_bytes.b[0];
         raw[428] = __convert_float32_to_bytes.b[1];
         raw[429] = __convert_float32_to_bytes.b[2];
         raw[430] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[2].track_state.relative_altitude;
         raw[431] = __convert_int16_to_bytes.b[0];
         raw[432] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[2].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].track_state.relative_vertical_speed;
         raw[433] = __convert_float32_to_bytes.b[0];
         raw[434] = __convert_float32_to_bytes.b[1];
         raw[435] = __convert_float32_to_bytes.b[2];
         raw[436] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[2].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[437] = u.b[0];
         raw[438] = u.b[1];
         raw[439] = u.b[2];
         raw[440] = u.b[3];
      } 
      { //write data.tracks[2].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[2].track_state.relative_range_uncertainty;
         raw[441] = __convert_uint32_to_bytes.b[0];
         raw[442] = __convert_uint32_to_bytes.b[1];
         raw[443] = __convert_uint32_to_bytes.b[2];
         raw[444] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[2].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].track_state.relative_bearing_uncertainty;
         raw[445] = __convert_float32_to_bytes.b[0];
         raw[446] = __convert_float32_to_bytes.b[1];
         raw[447] = __convert_float32_to_bytes.b[2];
         raw[448] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].track_state.relative_ground_speed_north_uncertainty;
         raw[449] = __convert_float32_to_bytes.b[0];
         raw[450] = __convert_float32_to_bytes.b[1];
         raw[451] = __convert_float32_to_bytes.b[2];
         raw[452] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].track_state.relative_ground_speed_east_uncertainty;
         raw[453] = __convert_float32_to_bytes.b[0];
         raw[454] = __convert_float32_to_bytes.b[1];
         raw[455] = __convert_float32_to_bytes.b[2];
         raw[456] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[2].track_state.relative_altitude_uncertainty;
         raw[457] = __convert_int16_to_bytes.b[0];
         raw[458] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[2].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].track_state.relative_vertical_speed_uncertainty;
         raw[459] = __convert_float32_to_bytes.b[0];
         raw[460] = __convert_float32_to_bytes.b[1];
         raw[461] = __convert_float32_to_bytes.b[2];
         raw[462] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[2].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[2].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[2].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[2].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[2].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[2].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[2].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[2].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[2].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[2].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[2].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[2].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[2].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[2].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[2].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[2].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[2].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[463] = bits[0];
         raw[464] = bits[1];
         raw[465] = bits[2];
      } 
      { //write data.tracks[2].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[2].aircraft_state.toa;
         raw[466] = __convert_uint64_to_bytes.b[0];
         raw[467] = __convert_uint64_to_bytes.b[1];
         raw[468] = __convert_uint64_to_bytes.b[2];
         raw[469] = __convert_uint64_to_bytes.b[3];
         raw[470] = __convert_uint64_to_bytes.b[4];
         raw[471] = __convert_uint64_to_bytes.b[5];
         raw[472] = __convert_uint64_to_bytes.b[6];
         raw[473] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[2].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[2].aircraft_state.longitude;
         raw[474] = __convert_float64_to_bytes.b[0];
         raw[475] = __convert_float64_to_bytes.b[1];
         raw[476] = __convert_float64_to_bytes.b[2];
         raw[477] = __convert_float64_to_bytes.b[3];
         raw[478] = __convert_float64_to_bytes.b[4];
         raw[479] = __convert_float64_to_bytes.b[5];
         raw[480] = __convert_float64_to_bytes.b[6];
         raw[481] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[2].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[2].aircraft_state.latitude;
         raw[482] = __convert_float64_to_bytes.b[0];
         raw[483] = __convert_float64_to_bytes.b[1];
         raw[484] = __convert_float64_to_bytes.b[2];
         raw[485] = __convert_float64_to_bytes.b[3];
         raw[486] = __convert_float64_to_bytes.b[4];
         raw[487] = __convert_float64_to_bytes.b[5];
         raw[488] = __convert_float64_to_bytes.b[6];
         raw[489] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[2].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.geometric_altitude;
         raw[490] = __convert_float32_to_bytes.b[0];
         raw[491] = __convert_float32_to_bytes.b[1];
         raw[492] = __convert_float32_to_bytes.b[2];
         raw[493] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.barometric_altitude;
         raw[494] = __convert_float32_to_bytes.b[0];
         raw[495] = __convert_float32_to_bytes.b[1];
         raw[496] = __convert_float32_to_bytes.b[2];
         raw[497] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.true_heading;
         raw[498] = __convert_float32_to_bytes.b[0];
         raw[499] = __convert_float32_to_bytes.b[1];
         raw[500] = __convert_float32_to_bytes.b[2];
         raw[501] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.magnetic_heading;
         raw[502] = __convert_float32_to_bytes.b[0];
         raw[503] = __convert_float32_to_bytes.b[1];
         raw[504] = __convert_float32_to_bytes.b[2];
         raw[505] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.ground_track_angle;
         raw[506] = __convert_float32_to_bytes.b[0];
         raw[507] = __convert_float32_to_bytes.b[1];
         raw[508] = __convert_float32_to_bytes.b[2];
         raw[509] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.true_air_speed;
         raw[510] = __convert_float32_to_bytes.b[0];
         raw[511] = __convert_float32_to_bytes.b[1];
         raw[512] = __convert_float32_to_bytes.b[2];
         raw[513] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.indicated_air_speed;
         raw[514] = __convert_float32_to_bytes.b[0];
         raw[515] = __convert_float32_to_bytes.b[1];
         raw[516] = __convert_float32_to_bytes.b[2];
         raw[517] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.ground_speed_north;
         raw[518] = __convert_float32_to_bytes.b[0];
         raw[519] = __convert_float32_to_bytes.b[1];
         raw[520] = __convert_float32_to_bytes.b[2];
         raw[521] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.ground_speed_east;
         raw[522] = __convert_float32_to_bytes.b[0];
         raw[523] = __convert_float32_to_bytes.b[1];
         raw[524] = __convert_float32_to_bytes.b[2];
         raw[525] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.geometric_vertical_rate;
         raw[526] = __convert_float32_to_bytes.b[0];
         raw[527] = __convert_float32_to_bytes.b[1];
         raw[528] = __convert_float32_to_bytes.b[2];
         raw[529] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.pressure_vertical_rate;
         raw[530] = __convert_float32_to_bytes.b[0];
         raw[531] = __convert_float32_to_bytes.b[1];
         raw[532] = __convert_float32_to_bytes.b[2];
         raw[533] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.heading_rate;
         raw[534] = __convert_float32_to_bytes.b[0];
         raw[535] = __convert_float32_to_bytes.b[1];
         raw[536] = __convert_float32_to_bytes.b[2];
         raw[537] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.ground_track_rate;
         raw[538] = __convert_float32_to_bytes.b[0];
         raw[539] = __convert_float32_to_bytes.b[1];
         raw[540] = __convert_float32_to_bytes.b[2];
         raw[541] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[2].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[542] = u.b[0];
         raw[543] = u.b[1];
         raw[544] = u.b[2];
         raw[545] = u.b[3];
      } 
      { //write data.tracks[2].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[2].aircraft_state.horizonal_position_uncertainty;
         raw[546] = __convert_uint16_to_bytes.b[0];
         raw[547] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[2].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[2].aircraft_state.pressure_altitude_uncertainty;
         raw[548] = __convert_uint16_to_bytes.b[0];
         raw[549] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[2].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[2].aircraft_state.geometric_altitude_uncertainty;
         raw[550] = __convert_uint16_to_bytes.b[0];
         raw[551] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[2].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.ground_speed_uncertainty;
         raw[552] = __convert_float32_to_bytes.b[0];
         raw[553] = __convert_float32_to_bytes.b[1];
         raw[554] = __convert_float32_to_bytes.b[2];
         raw[555] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.airspeed_uncertainty;
         raw[556] = __convert_float32_to_bytes.b[0];
         raw[557] = __convert_float32_to_bytes.b[1];
         raw[558] = __convert_float32_to_bytes.b[2];
         raw[559] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.heading_uncertainty;
         raw[560] = __convert_float32_to_bytes.b[0];
         raw[561] = __convert_float32_to_bytes.b[1];
         raw[562] = __convert_float32_to_bytes.b[2];
         raw[563] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.ground_track_angle_uncertainty;
         raw[564] = __convert_float32_to_bytes.b[0];
         raw[565] = __convert_float32_to_bytes.b[1];
         raw[566] = __convert_float32_to_bytes.b[2];
         raw[567] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.heading_rate_uncertainty;
         raw[568] = __convert_float32_to_bytes.b[0];
         raw[569] = __convert_float32_to_bytes.b[1];
         raw[570] = __convert_float32_to_bytes.b[2];
         raw[571] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[2].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[2].aircraft_state.ground_track_rate_uncertainty;
         raw[572] = __convert_float32_to_bytes.b[0];
         raw[573] = __convert_float32_to_bytes.b[1];
         raw[574] = __convert_float32_to_bytes.b[2];
         raw[575] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[3].track_id;
         raw[576] = __convert_uint32_to_bytes.b[0];
         raw[577] = __convert_uint32_to_bytes.b[1];
         raw[578] = __convert_uint32_to_bytes.b[2];
         raw[579] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[3].callsign[0];
         raw[580] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[3].callsign[1];
         raw[581] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[3].callsign[2];
         raw[582] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[3].callsign[3];
         raw[583] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[3].callsign[4];
         raw[584] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[3].callsign[5];
         raw[585] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[3].callsign[6];
         raw[586] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[3].callsign[7];
         raw[587] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[3].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[3].track_time;
         raw[588] = __convert_uint64_to_bytes.b[0];
         raw[589] = __convert_uint64_to_bytes.b[1];
         raw[590] = __convert_uint64_to_bytes.b[2];
         raw[591] = __convert_uint64_to_bytes.b[3];
         raw[592] = __convert_uint64_to_bytes.b[4];
         raw[593] = __convert_uint64_to_bytes.b[5];
         raw[594] = __convert_uint64_to_bytes.b[6];
         raw[595] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[3].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[596] = u.b[0];
         raw[597] = u.b[1];
         raw[598] = u.b[2];
         raw[599] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[3].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[600] = u.b[0];
         raw[601] = u.b[1];
         raw[602] = u.b[2];
         raw[603] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[3].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[3].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[3].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[3].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[3].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[3].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[3].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[3].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[3].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[604] = bits[0];
         raw[605] = bits[1];
      } 
      { //write data.tracks[3].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].track_state.relative_range;
         raw[606] = __convert_float32_to_bytes.b[0];
         raw[607] = __convert_float32_to_bytes.b[1];
         raw[608] = __convert_float32_to_bytes.b[2];
         raw[609] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].track_state.relative_bearing;
         raw[610] = __convert_float32_to_bytes.b[0];
         raw[611] = __convert_float32_to_bytes.b[1];
         raw[612] = __convert_float32_to_bytes.b[2];
         raw[613] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[3].track_state.bearing_invalid;
         raw[614] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[3].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].track_state.relative_ground_speed_north;
         raw[615] = __convert_float32_to_bytes.b[0];
         raw[616] = __convert_float32_to_bytes.b[1];
         raw[617] = __convert_float32_to_bytes.b[2];
         raw[618] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].track_state.relative_ground_speed_east;
         raw[619] = __convert_float32_to_bytes.b[0];
         raw[620] = __convert_float32_to_bytes.b[1];
         raw[621] = __convert_float32_to_bytes.b[2];
         raw[622] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[3].track_state.relative_altitude;
         raw[623] = __convert_int16_to_bytes.b[0];
         raw[624] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[3].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].track_state.relative_vertical_speed;
         raw[625] = __convert_float32_to_bytes.b[0];
         raw[626] = __convert_float32_to_bytes.b[1];
         raw[627] = __convert_float32_to_bytes.b[2];
         raw[628] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[3].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[629] = u.b[0];
         raw[630] = u.b[1];
         raw[631] = u.b[2];
         raw[632] = u.b[3];
      } 
      { //write data.tracks[3].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[3].track_state.relative_range_uncertainty;
         raw[633] = __convert_uint32_to_bytes.b[0];
         raw[634] = __convert_uint32_to_bytes.b[1];
         raw[635] = __convert_uint32_to_bytes.b[2];
         raw[636] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[3].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].track_state.relative_bearing_uncertainty;
         raw[637] = __convert_float32_to_bytes.b[0];
         raw[638] = __convert_float32_to_bytes.b[1];
         raw[639] = __convert_float32_to_bytes.b[2];
         raw[640] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].track_state.relative_ground_speed_north_uncertainty;
         raw[641] = __convert_float32_to_bytes.b[0];
         raw[642] = __convert_float32_to_bytes.b[1];
         raw[643] = __convert_float32_to_bytes.b[2];
         raw[644] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].track_state.relative_ground_speed_east_uncertainty;
         raw[645] = __convert_float32_to_bytes.b[0];
         raw[646] = __convert_float32_to_bytes.b[1];
         raw[647] = __convert_float32_to_bytes.b[2];
         raw[648] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[3].track_state.relative_altitude_uncertainty;
         raw[649] = __convert_int16_to_bytes.b[0];
         raw[650] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[3].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].track_state.relative_vertical_speed_uncertainty;
         raw[651] = __convert_float32_to_bytes.b[0];
         raw[652] = __convert_float32_to_bytes.b[1];
         raw[653] = __convert_float32_to_bytes.b[2];
         raw[654] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[3].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[3].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[3].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[3].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[3].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[3].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[3].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[3].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[3].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[3].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[3].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[3].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[3].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[3].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[3].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[3].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[3].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[655] = bits[0];
         raw[656] = bits[1];
         raw[657] = bits[2];
      } 
      { //write data.tracks[3].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[3].aircraft_state.toa;
         raw[658] = __convert_uint64_to_bytes.b[0];
         raw[659] = __convert_uint64_to_bytes.b[1];
         raw[660] = __convert_uint64_to_bytes.b[2];
         raw[661] = __convert_uint64_to_bytes.b[3];
         raw[662] = __convert_uint64_to_bytes.b[4];
         raw[663] = __convert_uint64_to_bytes.b[5];
         raw[664] = __convert_uint64_to_bytes.b[6];
         raw[665] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[3].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[3].aircraft_state.longitude;
         raw[666] = __convert_float64_to_bytes.b[0];
         raw[667] = __convert_float64_to_bytes.b[1];
         raw[668] = __convert_float64_to_bytes.b[2];
         raw[669] = __convert_float64_to_bytes.b[3];
         raw[670] = __convert_float64_to_bytes.b[4];
         raw[671] = __convert_float64_to_bytes.b[5];
         raw[672] = __convert_float64_to_bytes.b[6];
         raw[673] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[3].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[3].aircraft_state.latitude;
         raw[674] = __convert_float64_to_bytes.b[0];
         raw[675] = __convert_float64_to_bytes.b[1];
         raw[676] = __convert_float64_to_bytes.b[2];
         raw[677] = __convert_float64_to_bytes.b[3];
         raw[678] = __convert_float64_to_bytes.b[4];
         raw[679] = __convert_float64_to_bytes.b[5];
         raw[680] = __convert_float64_to_bytes.b[6];
         raw[681] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[3].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.geometric_altitude;
         raw[682] = __convert_float32_to_bytes.b[0];
         raw[683] = __convert_float32_to_bytes.b[1];
         raw[684] = __convert_float32_to_bytes.b[2];
         raw[685] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.barometric_altitude;
         raw[686] = __convert_float32_to_bytes.b[0];
         raw[687] = __convert_float32_to_bytes.b[1];
         raw[688] = __convert_float32_to_bytes.b[2];
         raw[689] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.true_heading;
         raw[690] = __convert_float32_to_bytes.b[0];
         raw[691] = __convert_float32_to_bytes.b[1];
         raw[692] = __convert_float32_to_bytes.b[2];
         raw[693] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.magnetic_heading;
         raw[694] = __convert_float32_to_bytes.b[0];
         raw[695] = __convert_float32_to_bytes.b[1];
         raw[696] = __convert_float32_to_bytes.b[2];
         raw[697] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.ground_track_angle;
         raw[698] = __convert_float32_to_bytes.b[0];
         raw[699] = __convert_float32_to_bytes.b[1];
         raw[700] = __convert_float32_to_bytes.b[2];
         raw[701] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.true_air_speed;
         raw[702] = __convert_float32_to_bytes.b[0];
         raw[703] = __convert_float32_to_bytes.b[1];
         raw[704] = __convert_float32_to_bytes.b[2];
         raw[705] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.indicated_air_speed;
         raw[706] = __convert_float32_to_bytes.b[0];
         raw[707] = __convert_float32_to_bytes.b[1];
         raw[708] = __convert_float32_to_bytes.b[2];
         raw[709] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.ground_speed_north;
         raw[710] = __convert_float32_to_bytes.b[0];
         raw[711] = __convert_float32_to_bytes.b[1];
         raw[712] = __convert_float32_to_bytes.b[2];
         raw[713] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.ground_speed_east;
         raw[714] = __convert_float32_to_bytes.b[0];
         raw[715] = __convert_float32_to_bytes.b[1];
         raw[716] = __convert_float32_to_bytes.b[2];
         raw[717] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.geometric_vertical_rate;
         raw[718] = __convert_float32_to_bytes.b[0];
         raw[719] = __convert_float32_to_bytes.b[1];
         raw[720] = __convert_float32_to_bytes.b[2];
         raw[721] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.pressure_vertical_rate;
         raw[722] = __convert_float32_to_bytes.b[0];
         raw[723] = __convert_float32_to_bytes.b[1];
         raw[724] = __convert_float32_to_bytes.b[2];
         raw[725] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.heading_rate;
         raw[726] = __convert_float32_to_bytes.b[0];
         raw[727] = __convert_float32_to_bytes.b[1];
         raw[728] = __convert_float32_to_bytes.b[2];
         raw[729] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.ground_track_rate;
         raw[730] = __convert_float32_to_bytes.b[0];
         raw[731] = __convert_float32_to_bytes.b[1];
         raw[732] = __convert_float32_to_bytes.b[2];
         raw[733] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[3].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[734] = u.b[0];
         raw[735] = u.b[1];
         raw[736] = u.b[2];
         raw[737] = u.b[3];
      } 
      { //write data.tracks[3].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[3].aircraft_state.horizonal_position_uncertainty;
         raw[738] = __convert_uint16_to_bytes.b[0];
         raw[739] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[3].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[3].aircraft_state.pressure_altitude_uncertainty;
         raw[740] = __convert_uint16_to_bytes.b[0];
         raw[741] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[3].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[3].aircraft_state.geometric_altitude_uncertainty;
         raw[742] = __convert_uint16_to_bytes.b[0];
         raw[743] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[3].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.ground_speed_uncertainty;
         raw[744] = __convert_float32_to_bytes.b[0];
         raw[745] = __convert_float32_to_bytes.b[1];
         raw[746] = __convert_float32_to_bytes.b[2];
         raw[747] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.airspeed_uncertainty;
         raw[748] = __convert_float32_to_bytes.b[0];
         raw[749] = __convert_float32_to_bytes.b[1];
         raw[750] = __convert_float32_to_bytes.b[2];
         raw[751] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.heading_uncertainty;
         raw[752] = __convert_float32_to_bytes.b[0];
         raw[753] = __convert_float32_to_bytes.b[1];
         raw[754] = __convert_float32_to_bytes.b[2];
         raw[755] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.ground_track_angle_uncertainty;
         raw[756] = __convert_float32_to_bytes.b[0];
         raw[757] = __convert_float32_to_bytes.b[1];
         raw[758] = __convert_float32_to_bytes.b[2];
         raw[759] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.heading_rate_uncertainty;
         raw[760] = __convert_float32_to_bytes.b[0];
         raw[761] = __convert_float32_to_bytes.b[1];
         raw[762] = __convert_float32_to_bytes.b[2];
         raw[763] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[3].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[3].aircraft_state.ground_track_rate_uncertainty;
         raw[764] = __convert_float32_to_bytes.b[0];
         raw[765] = __convert_float32_to_bytes.b[1];
         raw[766] = __convert_float32_to_bytes.b[2];
         raw[767] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[4].track_id;
         raw[768] = __convert_uint32_to_bytes.b[0];
         raw[769] = __convert_uint32_to_bytes.b[1];
         raw[770] = __convert_uint32_to_bytes.b[2];
         raw[771] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[4].callsign[0];
         raw[772] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[4].callsign[1];
         raw[773] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[4].callsign[2];
         raw[774] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[4].callsign[3];
         raw[775] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[4].callsign[4];
         raw[776] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[4].callsign[5];
         raw[777] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[4].callsign[6];
         raw[778] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[4].callsign[7];
         raw[779] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[4].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[4].track_time;
         raw[780] = __convert_uint64_to_bytes.b[0];
         raw[781] = __convert_uint64_to_bytes.b[1];
         raw[782] = __convert_uint64_to_bytes.b[2];
         raw[783] = __convert_uint64_to_bytes.b[3];
         raw[784] = __convert_uint64_to_bytes.b[4];
         raw[785] = __convert_uint64_to_bytes.b[5];
         raw[786] = __convert_uint64_to_bytes.b[6];
         raw[787] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[4].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[788] = u.b[0];
         raw[789] = u.b[1];
         raw[790] = u.b[2];
         raw[791] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[4].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[792] = u.b[0];
         raw[793] = u.b[1];
         raw[794] = u.b[2];
         raw[795] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[4].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[4].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[4].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[4].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[4].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[4].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[4].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[4].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[4].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[796] = bits[0];
         raw[797] = bits[1];
      } 
      { //write data.tracks[4].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].track_state.relative_range;
         raw[798] = __convert_float32_to_bytes.b[0];
         raw[799] = __convert_float32_to_bytes.b[1];
         raw[800] = __convert_float32_to_bytes.b[2];
         raw[801] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].track_state.relative_bearing;
         raw[802] = __convert_float32_to_bytes.b[0];
         raw[803] = __convert_float32_to_bytes.b[1];
         raw[804] = __convert_float32_to_bytes.b[2];
         raw[805] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[4].track_state.bearing_invalid;
         raw[806] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[4].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].track_state.relative_ground_speed_north;
         raw[807] = __convert_float32_to_bytes.b[0];
         raw[808] = __convert_float32_to_bytes.b[1];
         raw[809] = __convert_float32_to_bytes.b[2];
         raw[810] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].track_state.relative_ground_speed_east;
         raw[811] = __convert_float32_to_bytes.b[0];
         raw[812] = __convert_float32_to_bytes.b[1];
         raw[813] = __convert_float32_to_bytes.b[2];
         raw[814] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[4].track_state.relative_altitude;
         raw[815] = __convert_int16_to_bytes.b[0];
         raw[816] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[4].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].track_state.relative_vertical_speed;
         raw[817] = __convert_float32_to_bytes.b[0];
         raw[818] = __convert_float32_to_bytes.b[1];
         raw[819] = __convert_float32_to_bytes.b[2];
         raw[820] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[4].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[821] = u.b[0];
         raw[822] = u.b[1];
         raw[823] = u.b[2];
         raw[824] = u.b[3];
      } 
      { //write data.tracks[4].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[4].track_state.relative_range_uncertainty;
         raw[825] = __convert_uint32_to_bytes.b[0];
         raw[826] = __convert_uint32_to_bytes.b[1];
         raw[827] = __convert_uint32_to_bytes.b[2];
         raw[828] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[4].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].track_state.relative_bearing_uncertainty;
         raw[829] = __convert_float32_to_bytes.b[0];
         raw[830] = __convert_float32_to_bytes.b[1];
         raw[831] = __convert_float32_to_bytes.b[2];
         raw[832] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].track_state.relative_ground_speed_north_uncertainty;
         raw[833] = __convert_float32_to_bytes.b[0];
         raw[834] = __convert_float32_to_bytes.b[1];
         raw[835] = __convert_float32_to_bytes.b[2];
         raw[836] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].track_state.relative_ground_speed_east_uncertainty;
         raw[837] = __convert_float32_to_bytes.b[0];
         raw[838] = __convert_float32_to_bytes.b[1];
         raw[839] = __convert_float32_to_bytes.b[2];
         raw[840] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[4].track_state.relative_altitude_uncertainty;
         raw[841] = __convert_int16_to_bytes.b[0];
         raw[842] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[4].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].track_state.relative_vertical_speed_uncertainty;
         raw[843] = __convert_float32_to_bytes.b[0];
         raw[844] = __convert_float32_to_bytes.b[1];
         raw[845] = __convert_float32_to_bytes.b[2];
         raw[846] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[4].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[4].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[4].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[4].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[4].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[4].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[4].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[4].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[4].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[4].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[4].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[4].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[4].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[4].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[4].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[4].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[4].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[847] = bits[0];
         raw[848] = bits[1];
         raw[849] = bits[2];
      } 
      { //write data.tracks[4].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[4].aircraft_state.toa;
         raw[850] = __convert_uint64_to_bytes.b[0];
         raw[851] = __convert_uint64_to_bytes.b[1];
         raw[852] = __convert_uint64_to_bytes.b[2];
         raw[853] = __convert_uint64_to_bytes.b[3];
         raw[854] = __convert_uint64_to_bytes.b[4];
         raw[855] = __convert_uint64_to_bytes.b[5];
         raw[856] = __convert_uint64_to_bytes.b[6];
         raw[857] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[4].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[4].aircraft_state.longitude;
         raw[858] = __convert_float64_to_bytes.b[0];
         raw[859] = __convert_float64_to_bytes.b[1];
         raw[860] = __convert_float64_to_bytes.b[2];
         raw[861] = __convert_float64_to_bytes.b[3];
         raw[862] = __convert_float64_to_bytes.b[4];
         raw[863] = __convert_float64_to_bytes.b[5];
         raw[864] = __convert_float64_to_bytes.b[6];
         raw[865] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[4].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[4].aircraft_state.latitude;
         raw[866] = __convert_float64_to_bytes.b[0];
         raw[867] = __convert_float64_to_bytes.b[1];
         raw[868] = __convert_float64_to_bytes.b[2];
         raw[869] = __convert_float64_to_bytes.b[3];
         raw[870] = __convert_float64_to_bytes.b[4];
         raw[871] = __convert_float64_to_bytes.b[5];
         raw[872] = __convert_float64_to_bytes.b[6];
         raw[873] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[4].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.geometric_altitude;
         raw[874] = __convert_float32_to_bytes.b[0];
         raw[875] = __convert_float32_to_bytes.b[1];
         raw[876] = __convert_float32_to_bytes.b[2];
         raw[877] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.barometric_altitude;
         raw[878] = __convert_float32_to_bytes.b[0];
         raw[879] = __convert_float32_to_bytes.b[1];
         raw[880] = __convert_float32_to_bytes.b[2];
         raw[881] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.true_heading;
         raw[882] = __convert_float32_to_bytes.b[0];
         raw[883] = __convert_float32_to_bytes.b[1];
         raw[884] = __convert_float32_to_bytes.b[2];
         raw[885] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.magnetic_heading;
         raw[886] = __convert_float32_to_bytes.b[0];
         raw[887] = __convert_float32_to_bytes.b[1];
         raw[888] = __convert_float32_to_bytes.b[2];
         raw[889] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.ground_track_angle;
         raw[890] = __convert_float32_to_bytes.b[0];
         raw[891] = __convert_float32_to_bytes.b[1];
         raw[892] = __convert_float32_to_bytes.b[2];
         raw[893] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.true_air_speed;
         raw[894] = __convert_float32_to_bytes.b[0];
         raw[895] = __convert_float32_to_bytes.b[1];
         raw[896] = __convert_float32_to_bytes.b[2];
         raw[897] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.indicated_air_speed;
         raw[898] = __convert_float32_to_bytes.b[0];
         raw[899] = __convert_float32_to_bytes.b[1];
         raw[900] = __convert_float32_to_bytes.b[2];
         raw[901] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.ground_speed_north;
         raw[902] = __convert_float32_to_bytes.b[0];
         raw[903] = __convert_float32_to_bytes.b[1];
         raw[904] = __convert_float32_to_bytes.b[2];
         raw[905] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.ground_speed_east;
         raw[906] = __convert_float32_to_bytes.b[0];
         raw[907] = __convert_float32_to_bytes.b[1];
         raw[908] = __convert_float32_to_bytes.b[2];
         raw[909] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.geometric_vertical_rate;
         raw[910] = __convert_float32_to_bytes.b[0];
         raw[911] = __convert_float32_to_bytes.b[1];
         raw[912] = __convert_float32_to_bytes.b[2];
         raw[913] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.pressure_vertical_rate;
         raw[914] = __convert_float32_to_bytes.b[0];
         raw[915] = __convert_float32_to_bytes.b[1];
         raw[916] = __convert_float32_to_bytes.b[2];
         raw[917] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.heading_rate;
         raw[918] = __convert_float32_to_bytes.b[0];
         raw[919] = __convert_float32_to_bytes.b[1];
         raw[920] = __convert_float32_to_bytes.b[2];
         raw[921] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.ground_track_rate;
         raw[922] = __convert_float32_to_bytes.b[0];
         raw[923] = __convert_float32_to_bytes.b[1];
         raw[924] = __convert_float32_to_bytes.b[2];
         raw[925] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[4].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[926] = u.b[0];
         raw[927] = u.b[1];
         raw[928] = u.b[2];
         raw[929] = u.b[3];
      } 
      { //write data.tracks[4].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[4].aircraft_state.horizonal_position_uncertainty;
         raw[930] = __convert_uint16_to_bytes.b[0];
         raw[931] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[4].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[4].aircraft_state.pressure_altitude_uncertainty;
         raw[932] = __convert_uint16_to_bytes.b[0];
         raw[933] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[4].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[4].aircraft_state.geometric_altitude_uncertainty;
         raw[934] = __convert_uint16_to_bytes.b[0];
         raw[935] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[4].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.ground_speed_uncertainty;
         raw[936] = __convert_float32_to_bytes.b[0];
         raw[937] = __convert_float32_to_bytes.b[1];
         raw[938] = __convert_float32_to_bytes.b[2];
         raw[939] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.airspeed_uncertainty;
         raw[940] = __convert_float32_to_bytes.b[0];
         raw[941] = __convert_float32_to_bytes.b[1];
         raw[942] = __convert_float32_to_bytes.b[2];
         raw[943] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.heading_uncertainty;
         raw[944] = __convert_float32_to_bytes.b[0];
         raw[945] = __convert_float32_to_bytes.b[1];
         raw[946] = __convert_float32_to_bytes.b[2];
         raw[947] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.ground_track_angle_uncertainty;
         raw[948] = __convert_float32_to_bytes.b[0];
         raw[949] = __convert_float32_to_bytes.b[1];
         raw[950] = __convert_float32_to_bytes.b[2];
         raw[951] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.heading_rate_uncertainty;
         raw[952] = __convert_float32_to_bytes.b[0];
         raw[953] = __convert_float32_to_bytes.b[1];
         raw[954] = __convert_float32_to_bytes.b[2];
         raw[955] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[4].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[4].aircraft_state.ground_track_rate_uncertainty;
         raw[956] = __convert_float32_to_bytes.b[0];
         raw[957] = __convert_float32_to_bytes.b[1];
         raw[958] = __convert_float32_to_bytes.b[2];
         raw[959] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[5].track_id;
         raw[960] = __convert_uint32_to_bytes.b[0];
         raw[961] = __convert_uint32_to_bytes.b[1];
         raw[962] = __convert_uint32_to_bytes.b[2];
         raw[963] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[5].callsign[0];
         raw[964] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[5].callsign[1];
         raw[965] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[5].callsign[2];
         raw[966] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[5].callsign[3];
         raw[967] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[5].callsign[4];
         raw[968] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[5].callsign[5];
         raw[969] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[5].callsign[6];
         raw[970] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[5].callsign[7];
         raw[971] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[5].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[5].track_time;
         raw[972] = __convert_uint64_to_bytes.b[0];
         raw[973] = __convert_uint64_to_bytes.b[1];
         raw[974] = __convert_uint64_to_bytes.b[2];
         raw[975] = __convert_uint64_to_bytes.b[3];
         raw[976] = __convert_uint64_to_bytes.b[4];
         raw[977] = __convert_uint64_to_bytes.b[5];
         raw[978] = __convert_uint64_to_bytes.b[6];
         raw[979] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[5].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[980] = u.b[0];
         raw[981] = u.b[1];
         raw[982] = u.b[2];
         raw[983] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[5].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[984] = u.b[0];
         raw[985] = u.b[1];
         raw[986] = u.b[2];
         raw[987] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[5].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[5].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[5].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[5].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[5].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[5].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[5].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[5].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[5].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[988] = bits[0];
         raw[989] = bits[1];
      } 
      { //write data.tracks[5].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].track_state.relative_range;
         raw[990] = __convert_float32_to_bytes.b[0];
         raw[991] = __convert_float32_to_bytes.b[1];
         raw[992] = __convert_float32_to_bytes.b[2];
         raw[993] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].track_state.relative_bearing;
         raw[994] = __convert_float32_to_bytes.b[0];
         raw[995] = __convert_float32_to_bytes.b[1];
         raw[996] = __convert_float32_to_bytes.b[2];
         raw[997] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[5].track_state.bearing_invalid;
         raw[998] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[5].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].track_state.relative_ground_speed_north;
         raw[999] = __convert_float32_to_bytes.b[0];
         raw[1000] = __convert_float32_to_bytes.b[1];
         raw[1001] = __convert_float32_to_bytes.b[2];
         raw[1002] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].track_state.relative_ground_speed_east;
         raw[1003] = __convert_float32_to_bytes.b[0];
         raw[1004] = __convert_float32_to_bytes.b[1];
         raw[1005] = __convert_float32_to_bytes.b[2];
         raw[1006] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[5].track_state.relative_altitude;
         raw[1007] = __convert_int16_to_bytes.b[0];
         raw[1008] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[5].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].track_state.relative_vertical_speed;
         raw[1009] = __convert_float32_to_bytes.b[0];
         raw[1010] = __convert_float32_to_bytes.b[1];
         raw[1011] = __convert_float32_to_bytes.b[2];
         raw[1012] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[5].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1013] = u.b[0];
         raw[1014] = u.b[1];
         raw[1015] = u.b[2];
         raw[1016] = u.b[3];
      } 
      { //write data.tracks[5].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[5].track_state.relative_range_uncertainty;
         raw[1017] = __convert_uint32_to_bytes.b[0];
         raw[1018] = __convert_uint32_to_bytes.b[1];
         raw[1019] = __convert_uint32_to_bytes.b[2];
         raw[1020] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[5].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].track_state.relative_bearing_uncertainty;
         raw[1021] = __convert_float32_to_bytes.b[0];
         raw[1022] = __convert_float32_to_bytes.b[1];
         raw[1023] = __convert_float32_to_bytes.b[2];
         raw[1024] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].track_state.relative_ground_speed_north_uncertainty;
         raw[1025] = __convert_float32_to_bytes.b[0];
         raw[1026] = __convert_float32_to_bytes.b[1];
         raw[1027] = __convert_float32_to_bytes.b[2];
         raw[1028] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].track_state.relative_ground_speed_east_uncertainty;
         raw[1029] = __convert_float32_to_bytes.b[0];
         raw[1030] = __convert_float32_to_bytes.b[1];
         raw[1031] = __convert_float32_to_bytes.b[2];
         raw[1032] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[5].track_state.relative_altitude_uncertainty;
         raw[1033] = __convert_int16_to_bytes.b[0];
         raw[1034] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[5].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].track_state.relative_vertical_speed_uncertainty;
         raw[1035] = __convert_float32_to_bytes.b[0];
         raw[1036] = __convert_float32_to_bytes.b[1];
         raw[1037] = __convert_float32_to_bytes.b[2];
         raw[1038] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[5].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[5].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[5].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[5].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[5].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[5].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[5].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[5].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[5].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[5].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[5].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[5].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[5].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[5].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[5].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[5].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[5].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[1039] = bits[0];
         raw[1040] = bits[1];
         raw[1041] = bits[2];
      } 
      { //write data.tracks[5].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[5].aircraft_state.toa;
         raw[1042] = __convert_uint64_to_bytes.b[0];
         raw[1043] = __convert_uint64_to_bytes.b[1];
         raw[1044] = __convert_uint64_to_bytes.b[2];
         raw[1045] = __convert_uint64_to_bytes.b[3];
         raw[1046] = __convert_uint64_to_bytes.b[4];
         raw[1047] = __convert_uint64_to_bytes.b[5];
         raw[1048] = __convert_uint64_to_bytes.b[6];
         raw[1049] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[5].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[5].aircraft_state.longitude;
         raw[1050] = __convert_float64_to_bytes.b[0];
         raw[1051] = __convert_float64_to_bytes.b[1];
         raw[1052] = __convert_float64_to_bytes.b[2];
         raw[1053] = __convert_float64_to_bytes.b[3];
         raw[1054] = __convert_float64_to_bytes.b[4];
         raw[1055] = __convert_float64_to_bytes.b[5];
         raw[1056] = __convert_float64_to_bytes.b[6];
         raw[1057] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[5].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[5].aircraft_state.latitude;
         raw[1058] = __convert_float64_to_bytes.b[0];
         raw[1059] = __convert_float64_to_bytes.b[1];
         raw[1060] = __convert_float64_to_bytes.b[2];
         raw[1061] = __convert_float64_to_bytes.b[3];
         raw[1062] = __convert_float64_to_bytes.b[4];
         raw[1063] = __convert_float64_to_bytes.b[5];
         raw[1064] = __convert_float64_to_bytes.b[6];
         raw[1065] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[5].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.geometric_altitude;
         raw[1066] = __convert_float32_to_bytes.b[0];
         raw[1067] = __convert_float32_to_bytes.b[1];
         raw[1068] = __convert_float32_to_bytes.b[2];
         raw[1069] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.barometric_altitude;
         raw[1070] = __convert_float32_to_bytes.b[0];
         raw[1071] = __convert_float32_to_bytes.b[1];
         raw[1072] = __convert_float32_to_bytes.b[2];
         raw[1073] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.true_heading;
         raw[1074] = __convert_float32_to_bytes.b[0];
         raw[1075] = __convert_float32_to_bytes.b[1];
         raw[1076] = __convert_float32_to_bytes.b[2];
         raw[1077] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.magnetic_heading;
         raw[1078] = __convert_float32_to_bytes.b[0];
         raw[1079] = __convert_float32_to_bytes.b[1];
         raw[1080] = __convert_float32_to_bytes.b[2];
         raw[1081] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.ground_track_angle;
         raw[1082] = __convert_float32_to_bytes.b[0];
         raw[1083] = __convert_float32_to_bytes.b[1];
         raw[1084] = __convert_float32_to_bytes.b[2];
         raw[1085] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.true_air_speed;
         raw[1086] = __convert_float32_to_bytes.b[0];
         raw[1087] = __convert_float32_to_bytes.b[1];
         raw[1088] = __convert_float32_to_bytes.b[2];
         raw[1089] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.indicated_air_speed;
         raw[1090] = __convert_float32_to_bytes.b[0];
         raw[1091] = __convert_float32_to_bytes.b[1];
         raw[1092] = __convert_float32_to_bytes.b[2];
         raw[1093] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.ground_speed_north;
         raw[1094] = __convert_float32_to_bytes.b[0];
         raw[1095] = __convert_float32_to_bytes.b[1];
         raw[1096] = __convert_float32_to_bytes.b[2];
         raw[1097] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.ground_speed_east;
         raw[1098] = __convert_float32_to_bytes.b[0];
         raw[1099] = __convert_float32_to_bytes.b[1];
         raw[1100] = __convert_float32_to_bytes.b[2];
         raw[1101] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.geometric_vertical_rate;
         raw[1102] = __convert_float32_to_bytes.b[0];
         raw[1103] = __convert_float32_to_bytes.b[1];
         raw[1104] = __convert_float32_to_bytes.b[2];
         raw[1105] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.pressure_vertical_rate;
         raw[1106] = __convert_float32_to_bytes.b[0];
         raw[1107] = __convert_float32_to_bytes.b[1];
         raw[1108] = __convert_float32_to_bytes.b[2];
         raw[1109] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.heading_rate;
         raw[1110] = __convert_float32_to_bytes.b[0];
         raw[1111] = __convert_float32_to_bytes.b[1];
         raw[1112] = __convert_float32_to_bytes.b[2];
         raw[1113] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.ground_track_rate;
         raw[1114] = __convert_float32_to_bytes.b[0];
         raw[1115] = __convert_float32_to_bytes.b[1];
         raw[1116] = __convert_float32_to_bytes.b[2];
         raw[1117] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[5].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1118] = u.b[0];
         raw[1119] = u.b[1];
         raw[1120] = u.b[2];
         raw[1121] = u.b[3];
      } 
      { //write data.tracks[5].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[5].aircraft_state.horizonal_position_uncertainty;
         raw[1122] = __convert_uint16_to_bytes.b[0];
         raw[1123] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[5].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[5].aircraft_state.pressure_altitude_uncertainty;
         raw[1124] = __convert_uint16_to_bytes.b[0];
         raw[1125] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[5].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[5].aircraft_state.geometric_altitude_uncertainty;
         raw[1126] = __convert_uint16_to_bytes.b[0];
         raw[1127] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[5].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.ground_speed_uncertainty;
         raw[1128] = __convert_float32_to_bytes.b[0];
         raw[1129] = __convert_float32_to_bytes.b[1];
         raw[1130] = __convert_float32_to_bytes.b[2];
         raw[1131] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.airspeed_uncertainty;
         raw[1132] = __convert_float32_to_bytes.b[0];
         raw[1133] = __convert_float32_to_bytes.b[1];
         raw[1134] = __convert_float32_to_bytes.b[2];
         raw[1135] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.heading_uncertainty;
         raw[1136] = __convert_float32_to_bytes.b[0];
         raw[1137] = __convert_float32_to_bytes.b[1];
         raw[1138] = __convert_float32_to_bytes.b[2];
         raw[1139] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.ground_track_angle_uncertainty;
         raw[1140] = __convert_float32_to_bytes.b[0];
         raw[1141] = __convert_float32_to_bytes.b[1];
         raw[1142] = __convert_float32_to_bytes.b[2];
         raw[1143] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.heading_rate_uncertainty;
         raw[1144] = __convert_float32_to_bytes.b[0];
         raw[1145] = __convert_float32_to_bytes.b[1];
         raw[1146] = __convert_float32_to_bytes.b[2];
         raw[1147] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[5].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[5].aircraft_state.ground_track_rate_uncertainty;
         raw[1148] = __convert_float32_to_bytes.b[0];
         raw[1149] = __convert_float32_to_bytes.b[1];
         raw[1150] = __convert_float32_to_bytes.b[2];
         raw[1151] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[6].track_id;
         raw[1152] = __convert_uint32_to_bytes.b[0];
         raw[1153] = __convert_uint32_to_bytes.b[1];
         raw[1154] = __convert_uint32_to_bytes.b[2];
         raw[1155] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[6].callsign[0];
         raw[1156] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[6].callsign[1];
         raw[1157] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[6].callsign[2];
         raw[1158] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[6].callsign[3];
         raw[1159] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[6].callsign[4];
         raw[1160] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[6].callsign[5];
         raw[1161] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[6].callsign[6];
         raw[1162] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[6].callsign[7];
         raw[1163] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[6].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[6].track_time;
         raw[1164] = __convert_uint64_to_bytes.b[0];
         raw[1165] = __convert_uint64_to_bytes.b[1];
         raw[1166] = __convert_uint64_to_bytes.b[2];
         raw[1167] = __convert_uint64_to_bytes.b[3];
         raw[1168] = __convert_uint64_to_bytes.b[4];
         raw[1169] = __convert_uint64_to_bytes.b[5];
         raw[1170] = __convert_uint64_to_bytes.b[6];
         raw[1171] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[6].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1172] = u.b[0];
         raw[1173] = u.b[1];
         raw[1174] = u.b[2];
         raw[1175] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[6].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1176] = u.b[0];
         raw[1177] = u.b[1];
         raw[1178] = u.b[2];
         raw[1179] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[6].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[6].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[6].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[6].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[6].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[6].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[6].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[6].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[6].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[1180] = bits[0];
         raw[1181] = bits[1];
      } 
      { //write data.tracks[6].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].track_state.relative_range;
         raw[1182] = __convert_float32_to_bytes.b[0];
         raw[1183] = __convert_float32_to_bytes.b[1];
         raw[1184] = __convert_float32_to_bytes.b[2];
         raw[1185] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].track_state.relative_bearing;
         raw[1186] = __convert_float32_to_bytes.b[0];
         raw[1187] = __convert_float32_to_bytes.b[1];
         raw[1188] = __convert_float32_to_bytes.b[2];
         raw[1189] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[6].track_state.bearing_invalid;
         raw[1190] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[6].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].track_state.relative_ground_speed_north;
         raw[1191] = __convert_float32_to_bytes.b[0];
         raw[1192] = __convert_float32_to_bytes.b[1];
         raw[1193] = __convert_float32_to_bytes.b[2];
         raw[1194] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].track_state.relative_ground_speed_east;
         raw[1195] = __convert_float32_to_bytes.b[0];
         raw[1196] = __convert_float32_to_bytes.b[1];
         raw[1197] = __convert_float32_to_bytes.b[2];
         raw[1198] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[6].track_state.relative_altitude;
         raw[1199] = __convert_int16_to_bytes.b[0];
         raw[1200] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[6].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].track_state.relative_vertical_speed;
         raw[1201] = __convert_float32_to_bytes.b[0];
         raw[1202] = __convert_float32_to_bytes.b[1];
         raw[1203] = __convert_float32_to_bytes.b[2];
         raw[1204] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[6].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1205] = u.b[0];
         raw[1206] = u.b[1];
         raw[1207] = u.b[2];
         raw[1208] = u.b[3];
      } 
      { //write data.tracks[6].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[6].track_state.relative_range_uncertainty;
         raw[1209] = __convert_uint32_to_bytes.b[0];
         raw[1210] = __convert_uint32_to_bytes.b[1];
         raw[1211] = __convert_uint32_to_bytes.b[2];
         raw[1212] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[6].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].track_state.relative_bearing_uncertainty;
         raw[1213] = __convert_float32_to_bytes.b[0];
         raw[1214] = __convert_float32_to_bytes.b[1];
         raw[1215] = __convert_float32_to_bytes.b[2];
         raw[1216] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].track_state.relative_ground_speed_north_uncertainty;
         raw[1217] = __convert_float32_to_bytes.b[0];
         raw[1218] = __convert_float32_to_bytes.b[1];
         raw[1219] = __convert_float32_to_bytes.b[2];
         raw[1220] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].track_state.relative_ground_speed_east_uncertainty;
         raw[1221] = __convert_float32_to_bytes.b[0];
         raw[1222] = __convert_float32_to_bytes.b[1];
         raw[1223] = __convert_float32_to_bytes.b[2];
         raw[1224] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[6].track_state.relative_altitude_uncertainty;
         raw[1225] = __convert_int16_to_bytes.b[0];
         raw[1226] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[6].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].track_state.relative_vertical_speed_uncertainty;
         raw[1227] = __convert_float32_to_bytes.b[0];
         raw[1228] = __convert_float32_to_bytes.b[1];
         raw[1229] = __convert_float32_to_bytes.b[2];
         raw[1230] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[6].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[6].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[6].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[6].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[6].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[6].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[6].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[6].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[6].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[6].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[6].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[6].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[6].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[6].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[6].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[6].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[6].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[1231] = bits[0];
         raw[1232] = bits[1];
         raw[1233] = bits[2];
      } 
      { //write data.tracks[6].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[6].aircraft_state.toa;
         raw[1234] = __convert_uint64_to_bytes.b[0];
         raw[1235] = __convert_uint64_to_bytes.b[1];
         raw[1236] = __convert_uint64_to_bytes.b[2];
         raw[1237] = __convert_uint64_to_bytes.b[3];
         raw[1238] = __convert_uint64_to_bytes.b[4];
         raw[1239] = __convert_uint64_to_bytes.b[5];
         raw[1240] = __convert_uint64_to_bytes.b[6];
         raw[1241] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[6].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[6].aircraft_state.longitude;
         raw[1242] = __convert_float64_to_bytes.b[0];
         raw[1243] = __convert_float64_to_bytes.b[1];
         raw[1244] = __convert_float64_to_bytes.b[2];
         raw[1245] = __convert_float64_to_bytes.b[3];
         raw[1246] = __convert_float64_to_bytes.b[4];
         raw[1247] = __convert_float64_to_bytes.b[5];
         raw[1248] = __convert_float64_to_bytes.b[6];
         raw[1249] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[6].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[6].aircraft_state.latitude;
         raw[1250] = __convert_float64_to_bytes.b[0];
         raw[1251] = __convert_float64_to_bytes.b[1];
         raw[1252] = __convert_float64_to_bytes.b[2];
         raw[1253] = __convert_float64_to_bytes.b[3];
         raw[1254] = __convert_float64_to_bytes.b[4];
         raw[1255] = __convert_float64_to_bytes.b[5];
         raw[1256] = __convert_float64_to_bytes.b[6];
         raw[1257] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[6].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.geometric_altitude;
         raw[1258] = __convert_float32_to_bytes.b[0];
         raw[1259] = __convert_float32_to_bytes.b[1];
         raw[1260] = __convert_float32_to_bytes.b[2];
         raw[1261] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.barometric_altitude;
         raw[1262] = __convert_float32_to_bytes.b[0];
         raw[1263] = __convert_float32_to_bytes.b[1];
         raw[1264] = __convert_float32_to_bytes.b[2];
         raw[1265] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.true_heading;
         raw[1266] = __convert_float32_to_bytes.b[0];
         raw[1267] = __convert_float32_to_bytes.b[1];
         raw[1268] = __convert_float32_to_bytes.b[2];
         raw[1269] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.magnetic_heading;
         raw[1270] = __convert_float32_to_bytes.b[0];
         raw[1271] = __convert_float32_to_bytes.b[1];
         raw[1272] = __convert_float32_to_bytes.b[2];
         raw[1273] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.ground_track_angle;
         raw[1274] = __convert_float32_to_bytes.b[0];
         raw[1275] = __convert_float32_to_bytes.b[1];
         raw[1276] = __convert_float32_to_bytes.b[2];
         raw[1277] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.true_air_speed;
         raw[1278] = __convert_float32_to_bytes.b[0];
         raw[1279] = __convert_float32_to_bytes.b[1];
         raw[1280] = __convert_float32_to_bytes.b[2];
         raw[1281] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.indicated_air_speed;
         raw[1282] = __convert_float32_to_bytes.b[0];
         raw[1283] = __convert_float32_to_bytes.b[1];
         raw[1284] = __convert_float32_to_bytes.b[2];
         raw[1285] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.ground_speed_north;
         raw[1286] = __convert_float32_to_bytes.b[0];
         raw[1287] = __convert_float32_to_bytes.b[1];
         raw[1288] = __convert_float32_to_bytes.b[2];
         raw[1289] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.ground_speed_east;
         raw[1290] = __convert_float32_to_bytes.b[0];
         raw[1291] = __convert_float32_to_bytes.b[1];
         raw[1292] = __convert_float32_to_bytes.b[2];
         raw[1293] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.geometric_vertical_rate;
         raw[1294] = __convert_float32_to_bytes.b[0];
         raw[1295] = __convert_float32_to_bytes.b[1];
         raw[1296] = __convert_float32_to_bytes.b[2];
         raw[1297] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.pressure_vertical_rate;
         raw[1298] = __convert_float32_to_bytes.b[0];
         raw[1299] = __convert_float32_to_bytes.b[1];
         raw[1300] = __convert_float32_to_bytes.b[2];
         raw[1301] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.heading_rate;
         raw[1302] = __convert_float32_to_bytes.b[0];
         raw[1303] = __convert_float32_to_bytes.b[1];
         raw[1304] = __convert_float32_to_bytes.b[2];
         raw[1305] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.ground_track_rate;
         raw[1306] = __convert_float32_to_bytes.b[0];
         raw[1307] = __convert_float32_to_bytes.b[1];
         raw[1308] = __convert_float32_to_bytes.b[2];
         raw[1309] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[6].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1310] = u.b[0];
         raw[1311] = u.b[1];
         raw[1312] = u.b[2];
         raw[1313] = u.b[3];
      } 
      { //write data.tracks[6].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[6].aircraft_state.horizonal_position_uncertainty;
         raw[1314] = __convert_uint16_to_bytes.b[0];
         raw[1315] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[6].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[6].aircraft_state.pressure_altitude_uncertainty;
         raw[1316] = __convert_uint16_to_bytes.b[0];
         raw[1317] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[6].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[6].aircraft_state.geometric_altitude_uncertainty;
         raw[1318] = __convert_uint16_to_bytes.b[0];
         raw[1319] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[6].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.ground_speed_uncertainty;
         raw[1320] = __convert_float32_to_bytes.b[0];
         raw[1321] = __convert_float32_to_bytes.b[1];
         raw[1322] = __convert_float32_to_bytes.b[2];
         raw[1323] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.airspeed_uncertainty;
         raw[1324] = __convert_float32_to_bytes.b[0];
         raw[1325] = __convert_float32_to_bytes.b[1];
         raw[1326] = __convert_float32_to_bytes.b[2];
         raw[1327] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.heading_uncertainty;
         raw[1328] = __convert_float32_to_bytes.b[0];
         raw[1329] = __convert_float32_to_bytes.b[1];
         raw[1330] = __convert_float32_to_bytes.b[2];
         raw[1331] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.ground_track_angle_uncertainty;
         raw[1332] = __convert_float32_to_bytes.b[0];
         raw[1333] = __convert_float32_to_bytes.b[1];
         raw[1334] = __convert_float32_to_bytes.b[2];
         raw[1335] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.heading_rate_uncertainty;
         raw[1336] = __convert_float32_to_bytes.b[0];
         raw[1337] = __convert_float32_to_bytes.b[1];
         raw[1338] = __convert_float32_to_bytes.b[2];
         raw[1339] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[6].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[6].aircraft_state.ground_track_rate_uncertainty;
         raw[1340] = __convert_float32_to_bytes.b[0];
         raw[1341] = __convert_float32_to_bytes.b[1];
         raw[1342] = __convert_float32_to_bytes.b[2];
         raw[1343] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[7].track_id;
         raw[1344] = __convert_uint32_to_bytes.b[0];
         raw[1345] = __convert_uint32_to_bytes.b[1];
         raw[1346] = __convert_uint32_to_bytes.b[2];
         raw[1347] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[7].callsign[0];
         raw[1348] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[7].callsign[1];
         raw[1349] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[7].callsign[2];
         raw[1350] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[7].callsign[3];
         raw[1351] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[7].callsign[4];
         raw[1352] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[7].callsign[5];
         raw[1353] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[7].callsign[6];
         raw[1354] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[7].callsign[7];
         raw[1355] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[7].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[7].track_time;
         raw[1356] = __convert_uint64_to_bytes.b[0];
         raw[1357] = __convert_uint64_to_bytes.b[1];
         raw[1358] = __convert_uint64_to_bytes.b[2];
         raw[1359] = __convert_uint64_to_bytes.b[3];
         raw[1360] = __convert_uint64_to_bytes.b[4];
         raw[1361] = __convert_uint64_to_bytes.b[5];
         raw[1362] = __convert_uint64_to_bytes.b[6];
         raw[1363] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[7].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1364] = u.b[0];
         raw[1365] = u.b[1];
         raw[1366] = u.b[2];
         raw[1367] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[7].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1368] = u.b[0];
         raw[1369] = u.b[1];
         raw[1370] = u.b[2];
         raw[1371] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[7].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[7].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[7].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[7].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[7].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[7].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[7].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[7].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[7].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[1372] = bits[0];
         raw[1373] = bits[1];
      } 
      { //write data.tracks[7].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].track_state.relative_range;
         raw[1374] = __convert_float32_to_bytes.b[0];
         raw[1375] = __convert_float32_to_bytes.b[1];
         raw[1376] = __convert_float32_to_bytes.b[2];
         raw[1377] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].track_state.relative_bearing;
         raw[1378] = __convert_float32_to_bytes.b[0];
         raw[1379] = __convert_float32_to_bytes.b[1];
         raw[1380] = __convert_float32_to_bytes.b[2];
         raw[1381] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[7].track_state.bearing_invalid;
         raw[1382] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[7].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].track_state.relative_ground_speed_north;
         raw[1383] = __convert_float32_to_bytes.b[0];
         raw[1384] = __convert_float32_to_bytes.b[1];
         raw[1385] = __convert_float32_to_bytes.b[2];
         raw[1386] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].track_state.relative_ground_speed_east;
         raw[1387] = __convert_float32_to_bytes.b[0];
         raw[1388] = __convert_float32_to_bytes.b[1];
         raw[1389] = __convert_float32_to_bytes.b[2];
         raw[1390] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[7].track_state.relative_altitude;
         raw[1391] = __convert_int16_to_bytes.b[0];
         raw[1392] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[7].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].track_state.relative_vertical_speed;
         raw[1393] = __convert_float32_to_bytes.b[0];
         raw[1394] = __convert_float32_to_bytes.b[1];
         raw[1395] = __convert_float32_to_bytes.b[2];
         raw[1396] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[7].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1397] = u.b[0];
         raw[1398] = u.b[1];
         raw[1399] = u.b[2];
         raw[1400] = u.b[3];
      } 
      { //write data.tracks[7].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[7].track_state.relative_range_uncertainty;
         raw[1401] = __convert_uint32_to_bytes.b[0];
         raw[1402] = __convert_uint32_to_bytes.b[1];
         raw[1403] = __convert_uint32_to_bytes.b[2];
         raw[1404] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[7].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].track_state.relative_bearing_uncertainty;
         raw[1405] = __convert_float32_to_bytes.b[0];
         raw[1406] = __convert_float32_to_bytes.b[1];
         raw[1407] = __convert_float32_to_bytes.b[2];
         raw[1408] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].track_state.relative_ground_speed_north_uncertainty;
         raw[1409] = __convert_float32_to_bytes.b[0];
         raw[1410] = __convert_float32_to_bytes.b[1];
         raw[1411] = __convert_float32_to_bytes.b[2];
         raw[1412] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].track_state.relative_ground_speed_east_uncertainty;
         raw[1413] = __convert_float32_to_bytes.b[0];
         raw[1414] = __convert_float32_to_bytes.b[1];
         raw[1415] = __convert_float32_to_bytes.b[2];
         raw[1416] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[7].track_state.relative_altitude_uncertainty;
         raw[1417] = __convert_int16_to_bytes.b[0];
         raw[1418] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[7].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].track_state.relative_vertical_speed_uncertainty;
         raw[1419] = __convert_float32_to_bytes.b[0];
         raw[1420] = __convert_float32_to_bytes.b[1];
         raw[1421] = __convert_float32_to_bytes.b[2];
         raw[1422] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[7].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[7].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[7].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[7].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[7].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[7].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[7].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[7].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[7].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[7].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[7].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[7].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[7].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[7].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[7].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[7].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[7].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[1423] = bits[0];
         raw[1424] = bits[1];
         raw[1425] = bits[2];
      } 
      { //write data.tracks[7].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[7].aircraft_state.toa;
         raw[1426] = __convert_uint64_to_bytes.b[0];
         raw[1427] = __convert_uint64_to_bytes.b[1];
         raw[1428] = __convert_uint64_to_bytes.b[2];
         raw[1429] = __convert_uint64_to_bytes.b[3];
         raw[1430] = __convert_uint64_to_bytes.b[4];
         raw[1431] = __convert_uint64_to_bytes.b[5];
         raw[1432] = __convert_uint64_to_bytes.b[6];
         raw[1433] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[7].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[7].aircraft_state.longitude;
         raw[1434] = __convert_float64_to_bytes.b[0];
         raw[1435] = __convert_float64_to_bytes.b[1];
         raw[1436] = __convert_float64_to_bytes.b[2];
         raw[1437] = __convert_float64_to_bytes.b[3];
         raw[1438] = __convert_float64_to_bytes.b[4];
         raw[1439] = __convert_float64_to_bytes.b[5];
         raw[1440] = __convert_float64_to_bytes.b[6];
         raw[1441] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[7].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[7].aircraft_state.latitude;
         raw[1442] = __convert_float64_to_bytes.b[0];
         raw[1443] = __convert_float64_to_bytes.b[1];
         raw[1444] = __convert_float64_to_bytes.b[2];
         raw[1445] = __convert_float64_to_bytes.b[3];
         raw[1446] = __convert_float64_to_bytes.b[4];
         raw[1447] = __convert_float64_to_bytes.b[5];
         raw[1448] = __convert_float64_to_bytes.b[6];
         raw[1449] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[7].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.geometric_altitude;
         raw[1450] = __convert_float32_to_bytes.b[0];
         raw[1451] = __convert_float32_to_bytes.b[1];
         raw[1452] = __convert_float32_to_bytes.b[2];
         raw[1453] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.barometric_altitude;
         raw[1454] = __convert_float32_to_bytes.b[0];
         raw[1455] = __convert_float32_to_bytes.b[1];
         raw[1456] = __convert_float32_to_bytes.b[2];
         raw[1457] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.true_heading;
         raw[1458] = __convert_float32_to_bytes.b[0];
         raw[1459] = __convert_float32_to_bytes.b[1];
         raw[1460] = __convert_float32_to_bytes.b[2];
         raw[1461] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.magnetic_heading;
         raw[1462] = __convert_float32_to_bytes.b[0];
         raw[1463] = __convert_float32_to_bytes.b[1];
         raw[1464] = __convert_float32_to_bytes.b[2];
         raw[1465] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.ground_track_angle;
         raw[1466] = __convert_float32_to_bytes.b[0];
         raw[1467] = __convert_float32_to_bytes.b[1];
         raw[1468] = __convert_float32_to_bytes.b[2];
         raw[1469] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.true_air_speed;
         raw[1470] = __convert_float32_to_bytes.b[0];
         raw[1471] = __convert_float32_to_bytes.b[1];
         raw[1472] = __convert_float32_to_bytes.b[2];
         raw[1473] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.indicated_air_speed;
         raw[1474] = __convert_float32_to_bytes.b[0];
         raw[1475] = __convert_float32_to_bytes.b[1];
         raw[1476] = __convert_float32_to_bytes.b[2];
         raw[1477] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.ground_speed_north;
         raw[1478] = __convert_float32_to_bytes.b[0];
         raw[1479] = __convert_float32_to_bytes.b[1];
         raw[1480] = __convert_float32_to_bytes.b[2];
         raw[1481] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.ground_speed_east;
         raw[1482] = __convert_float32_to_bytes.b[0];
         raw[1483] = __convert_float32_to_bytes.b[1];
         raw[1484] = __convert_float32_to_bytes.b[2];
         raw[1485] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.geometric_vertical_rate;
         raw[1486] = __convert_float32_to_bytes.b[0];
         raw[1487] = __convert_float32_to_bytes.b[1];
         raw[1488] = __convert_float32_to_bytes.b[2];
         raw[1489] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.pressure_vertical_rate;
         raw[1490] = __convert_float32_to_bytes.b[0];
         raw[1491] = __convert_float32_to_bytes.b[1];
         raw[1492] = __convert_float32_to_bytes.b[2];
         raw[1493] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.heading_rate;
         raw[1494] = __convert_float32_to_bytes.b[0];
         raw[1495] = __convert_float32_to_bytes.b[1];
         raw[1496] = __convert_float32_to_bytes.b[2];
         raw[1497] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.ground_track_rate;
         raw[1498] = __convert_float32_to_bytes.b[0];
         raw[1499] = __convert_float32_to_bytes.b[1];
         raw[1500] = __convert_float32_to_bytes.b[2];
         raw[1501] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[7].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1502] = u.b[0];
         raw[1503] = u.b[1];
         raw[1504] = u.b[2];
         raw[1505] = u.b[3];
      } 
      { //write data.tracks[7].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[7].aircraft_state.horizonal_position_uncertainty;
         raw[1506] = __convert_uint16_to_bytes.b[0];
         raw[1507] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[7].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[7].aircraft_state.pressure_altitude_uncertainty;
         raw[1508] = __convert_uint16_to_bytes.b[0];
         raw[1509] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[7].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[7].aircraft_state.geometric_altitude_uncertainty;
         raw[1510] = __convert_uint16_to_bytes.b[0];
         raw[1511] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[7].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.ground_speed_uncertainty;
         raw[1512] = __convert_float32_to_bytes.b[0];
         raw[1513] = __convert_float32_to_bytes.b[1];
         raw[1514] = __convert_float32_to_bytes.b[2];
         raw[1515] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.airspeed_uncertainty;
         raw[1516] = __convert_float32_to_bytes.b[0];
         raw[1517] = __convert_float32_to_bytes.b[1];
         raw[1518] = __convert_float32_to_bytes.b[2];
         raw[1519] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.heading_uncertainty;
         raw[1520] = __convert_float32_to_bytes.b[0];
         raw[1521] = __convert_float32_to_bytes.b[1];
         raw[1522] = __convert_float32_to_bytes.b[2];
         raw[1523] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.ground_track_angle_uncertainty;
         raw[1524] = __convert_float32_to_bytes.b[0];
         raw[1525] = __convert_float32_to_bytes.b[1];
         raw[1526] = __convert_float32_to_bytes.b[2];
         raw[1527] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.heading_rate_uncertainty;
         raw[1528] = __convert_float32_to_bytes.b[0];
         raw[1529] = __convert_float32_to_bytes.b[1];
         raw[1530] = __convert_float32_to_bytes.b[2];
         raw[1531] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[7].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[7].aircraft_state.ground_track_rate_uncertainty;
         raw[1532] = __convert_float32_to_bytes.b[0];
         raw[1533] = __convert_float32_to_bytes.b[1];
         raw[1534] = __convert_float32_to_bytes.b[2];
         raw[1535] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[8].track_id;
         raw[1536] = __convert_uint32_to_bytes.b[0];
         raw[1537] = __convert_uint32_to_bytes.b[1];
         raw[1538] = __convert_uint32_to_bytes.b[2];
         raw[1539] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[8].callsign[0];
         raw[1540] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[8].callsign[1];
         raw[1541] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[8].callsign[2];
         raw[1542] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[8].callsign[3];
         raw[1543] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[8].callsign[4];
         raw[1544] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[8].callsign[5];
         raw[1545] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[8].callsign[6];
         raw[1546] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[8].callsign[7];
         raw[1547] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[8].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[8].track_time;
         raw[1548] = __convert_uint64_to_bytes.b[0];
         raw[1549] = __convert_uint64_to_bytes.b[1];
         raw[1550] = __convert_uint64_to_bytes.b[2];
         raw[1551] = __convert_uint64_to_bytes.b[3];
         raw[1552] = __convert_uint64_to_bytes.b[4];
         raw[1553] = __convert_uint64_to_bytes.b[5];
         raw[1554] = __convert_uint64_to_bytes.b[6];
         raw[1555] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[8].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1556] = u.b[0];
         raw[1557] = u.b[1];
         raw[1558] = u.b[2];
         raw[1559] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[8].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1560] = u.b[0];
         raw[1561] = u.b[1];
         raw[1562] = u.b[2];
         raw[1563] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[8].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[8].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[8].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[8].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[8].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[8].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[8].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[8].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[8].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[1564] = bits[0];
         raw[1565] = bits[1];
      } 
      { //write data.tracks[8].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].track_state.relative_range;
         raw[1566] = __convert_float32_to_bytes.b[0];
         raw[1567] = __convert_float32_to_bytes.b[1];
         raw[1568] = __convert_float32_to_bytes.b[2];
         raw[1569] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].track_state.relative_bearing;
         raw[1570] = __convert_float32_to_bytes.b[0];
         raw[1571] = __convert_float32_to_bytes.b[1];
         raw[1572] = __convert_float32_to_bytes.b[2];
         raw[1573] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[8].track_state.bearing_invalid;
         raw[1574] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[8].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].track_state.relative_ground_speed_north;
         raw[1575] = __convert_float32_to_bytes.b[0];
         raw[1576] = __convert_float32_to_bytes.b[1];
         raw[1577] = __convert_float32_to_bytes.b[2];
         raw[1578] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].track_state.relative_ground_speed_east;
         raw[1579] = __convert_float32_to_bytes.b[0];
         raw[1580] = __convert_float32_to_bytes.b[1];
         raw[1581] = __convert_float32_to_bytes.b[2];
         raw[1582] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[8].track_state.relative_altitude;
         raw[1583] = __convert_int16_to_bytes.b[0];
         raw[1584] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[8].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].track_state.relative_vertical_speed;
         raw[1585] = __convert_float32_to_bytes.b[0];
         raw[1586] = __convert_float32_to_bytes.b[1];
         raw[1587] = __convert_float32_to_bytes.b[2];
         raw[1588] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[8].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1589] = u.b[0];
         raw[1590] = u.b[1];
         raw[1591] = u.b[2];
         raw[1592] = u.b[3];
      } 
      { //write data.tracks[8].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[8].track_state.relative_range_uncertainty;
         raw[1593] = __convert_uint32_to_bytes.b[0];
         raw[1594] = __convert_uint32_to_bytes.b[1];
         raw[1595] = __convert_uint32_to_bytes.b[2];
         raw[1596] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[8].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].track_state.relative_bearing_uncertainty;
         raw[1597] = __convert_float32_to_bytes.b[0];
         raw[1598] = __convert_float32_to_bytes.b[1];
         raw[1599] = __convert_float32_to_bytes.b[2];
         raw[1600] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].track_state.relative_ground_speed_north_uncertainty;
         raw[1601] = __convert_float32_to_bytes.b[0];
         raw[1602] = __convert_float32_to_bytes.b[1];
         raw[1603] = __convert_float32_to_bytes.b[2];
         raw[1604] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].track_state.relative_ground_speed_east_uncertainty;
         raw[1605] = __convert_float32_to_bytes.b[0];
         raw[1606] = __convert_float32_to_bytes.b[1];
         raw[1607] = __convert_float32_to_bytes.b[2];
         raw[1608] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[8].track_state.relative_altitude_uncertainty;
         raw[1609] = __convert_int16_to_bytes.b[0];
         raw[1610] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[8].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].track_state.relative_vertical_speed_uncertainty;
         raw[1611] = __convert_float32_to_bytes.b[0];
         raw[1612] = __convert_float32_to_bytes.b[1];
         raw[1613] = __convert_float32_to_bytes.b[2];
         raw[1614] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[8].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[8].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[8].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[8].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[8].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[8].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[8].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[8].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[8].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[8].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[8].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[8].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[8].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[8].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[8].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[8].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[8].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[1615] = bits[0];
         raw[1616] = bits[1];
         raw[1617] = bits[2];
      } 
      { //write data.tracks[8].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[8].aircraft_state.toa;
         raw[1618] = __convert_uint64_to_bytes.b[0];
         raw[1619] = __convert_uint64_to_bytes.b[1];
         raw[1620] = __convert_uint64_to_bytes.b[2];
         raw[1621] = __convert_uint64_to_bytes.b[3];
         raw[1622] = __convert_uint64_to_bytes.b[4];
         raw[1623] = __convert_uint64_to_bytes.b[5];
         raw[1624] = __convert_uint64_to_bytes.b[6];
         raw[1625] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[8].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[8].aircraft_state.longitude;
         raw[1626] = __convert_float64_to_bytes.b[0];
         raw[1627] = __convert_float64_to_bytes.b[1];
         raw[1628] = __convert_float64_to_bytes.b[2];
         raw[1629] = __convert_float64_to_bytes.b[3];
         raw[1630] = __convert_float64_to_bytes.b[4];
         raw[1631] = __convert_float64_to_bytes.b[5];
         raw[1632] = __convert_float64_to_bytes.b[6];
         raw[1633] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[8].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[8].aircraft_state.latitude;
         raw[1634] = __convert_float64_to_bytes.b[0];
         raw[1635] = __convert_float64_to_bytes.b[1];
         raw[1636] = __convert_float64_to_bytes.b[2];
         raw[1637] = __convert_float64_to_bytes.b[3];
         raw[1638] = __convert_float64_to_bytes.b[4];
         raw[1639] = __convert_float64_to_bytes.b[5];
         raw[1640] = __convert_float64_to_bytes.b[6];
         raw[1641] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[8].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.geometric_altitude;
         raw[1642] = __convert_float32_to_bytes.b[0];
         raw[1643] = __convert_float32_to_bytes.b[1];
         raw[1644] = __convert_float32_to_bytes.b[2];
         raw[1645] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.barometric_altitude;
         raw[1646] = __convert_float32_to_bytes.b[0];
         raw[1647] = __convert_float32_to_bytes.b[1];
         raw[1648] = __convert_float32_to_bytes.b[2];
         raw[1649] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.true_heading;
         raw[1650] = __convert_float32_to_bytes.b[0];
         raw[1651] = __convert_float32_to_bytes.b[1];
         raw[1652] = __convert_float32_to_bytes.b[2];
         raw[1653] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.magnetic_heading;
         raw[1654] = __convert_float32_to_bytes.b[0];
         raw[1655] = __convert_float32_to_bytes.b[1];
         raw[1656] = __convert_float32_to_bytes.b[2];
         raw[1657] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.ground_track_angle;
         raw[1658] = __convert_float32_to_bytes.b[0];
         raw[1659] = __convert_float32_to_bytes.b[1];
         raw[1660] = __convert_float32_to_bytes.b[2];
         raw[1661] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.true_air_speed;
         raw[1662] = __convert_float32_to_bytes.b[0];
         raw[1663] = __convert_float32_to_bytes.b[1];
         raw[1664] = __convert_float32_to_bytes.b[2];
         raw[1665] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.indicated_air_speed;
         raw[1666] = __convert_float32_to_bytes.b[0];
         raw[1667] = __convert_float32_to_bytes.b[1];
         raw[1668] = __convert_float32_to_bytes.b[2];
         raw[1669] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.ground_speed_north;
         raw[1670] = __convert_float32_to_bytes.b[0];
         raw[1671] = __convert_float32_to_bytes.b[1];
         raw[1672] = __convert_float32_to_bytes.b[2];
         raw[1673] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.ground_speed_east;
         raw[1674] = __convert_float32_to_bytes.b[0];
         raw[1675] = __convert_float32_to_bytes.b[1];
         raw[1676] = __convert_float32_to_bytes.b[2];
         raw[1677] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.geometric_vertical_rate;
         raw[1678] = __convert_float32_to_bytes.b[0];
         raw[1679] = __convert_float32_to_bytes.b[1];
         raw[1680] = __convert_float32_to_bytes.b[2];
         raw[1681] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.pressure_vertical_rate;
         raw[1682] = __convert_float32_to_bytes.b[0];
         raw[1683] = __convert_float32_to_bytes.b[1];
         raw[1684] = __convert_float32_to_bytes.b[2];
         raw[1685] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.heading_rate;
         raw[1686] = __convert_float32_to_bytes.b[0];
         raw[1687] = __convert_float32_to_bytes.b[1];
         raw[1688] = __convert_float32_to_bytes.b[2];
         raw[1689] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.ground_track_rate;
         raw[1690] = __convert_float32_to_bytes.b[0];
         raw[1691] = __convert_float32_to_bytes.b[1];
         raw[1692] = __convert_float32_to_bytes.b[2];
         raw[1693] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[8].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1694] = u.b[0];
         raw[1695] = u.b[1];
         raw[1696] = u.b[2];
         raw[1697] = u.b[3];
      } 
      { //write data.tracks[8].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[8].aircraft_state.horizonal_position_uncertainty;
         raw[1698] = __convert_uint16_to_bytes.b[0];
         raw[1699] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[8].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[8].aircraft_state.pressure_altitude_uncertainty;
         raw[1700] = __convert_uint16_to_bytes.b[0];
         raw[1701] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[8].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[8].aircraft_state.geometric_altitude_uncertainty;
         raw[1702] = __convert_uint16_to_bytes.b[0];
         raw[1703] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[8].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.ground_speed_uncertainty;
         raw[1704] = __convert_float32_to_bytes.b[0];
         raw[1705] = __convert_float32_to_bytes.b[1];
         raw[1706] = __convert_float32_to_bytes.b[2];
         raw[1707] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.airspeed_uncertainty;
         raw[1708] = __convert_float32_to_bytes.b[0];
         raw[1709] = __convert_float32_to_bytes.b[1];
         raw[1710] = __convert_float32_to_bytes.b[2];
         raw[1711] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.heading_uncertainty;
         raw[1712] = __convert_float32_to_bytes.b[0];
         raw[1713] = __convert_float32_to_bytes.b[1];
         raw[1714] = __convert_float32_to_bytes.b[2];
         raw[1715] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.ground_track_angle_uncertainty;
         raw[1716] = __convert_float32_to_bytes.b[0];
         raw[1717] = __convert_float32_to_bytes.b[1];
         raw[1718] = __convert_float32_to_bytes.b[2];
         raw[1719] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.heading_rate_uncertainty;
         raw[1720] = __convert_float32_to_bytes.b[0];
         raw[1721] = __convert_float32_to_bytes.b[1];
         raw[1722] = __convert_float32_to_bytes.b[2];
         raw[1723] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[8].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[8].aircraft_state.ground_track_rate_uncertainty;
         raw[1724] = __convert_float32_to_bytes.b[0];
         raw[1725] = __convert_float32_to_bytes.b[1];
         raw[1726] = __convert_float32_to_bytes.b[2];
         raw[1727] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[9].track_id;
         raw[1728] = __convert_uint32_to_bytes.b[0];
         raw[1729] = __convert_uint32_to_bytes.b[1];
         raw[1730] = __convert_uint32_to_bytes.b[2];
         raw[1731] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[9].callsign[0];
         raw[1732] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[9].callsign[1];
         raw[1733] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[9].callsign[2];
         raw[1734] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[9].callsign[3];
         raw[1735] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[9].callsign[4];
         raw[1736] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[9].callsign[5];
         raw[1737] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[9].callsign[6];
         raw[1738] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.a = data.tracks[9].callsign[7];
         raw[1739] = __convert_text_to_bytes.b[0];
      } 
      { //write data.tracks[9].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[9].track_time;
         raw[1740] = __convert_uint64_to_bytes.b[0];
         raw[1741] = __convert_uint64_to_bytes.b[1];
         raw[1742] = __convert_uint64_to_bytes.b[2];
         raw[1743] = __convert_uint64_to_bytes.b[3];
         raw[1744] = __convert_uint64_to_bytes.b[4];
         raw[1745] = __convert_uint64_to_bytes.b[5];
         raw[1746] = __convert_uint64_to_bytes.b[6];
         raw[1747] = __convert_uint64_to_bytes.b[7];
      } 
      { //write enum aircraft_category_t = aircraft_category_t::aircraft_category_t_end
         uint32_t ev = aircraft_category_t::aircraft_category_t_end;
         switch(data.tracks[9].aircraft_category) {
         case aircraft_category_t::NA:
            ev = 0;
            break;
         case aircraft_category_t::LIGHT:
            ev = 1;
            break;
         case aircraft_category_t::SMALL:
            ev = 2;
            break;
         case aircraft_category_t::LARGE:
            ev = 3;
            break;
         case aircraft_category_t::HVORTEX:
            ev = 4;
            break;
         case aircraft_category_t::HEAVY:
            ev = 5;
            break;
         case aircraft_category_t::HPERFORMANCE:
            ev = 6;
            break;
         case aircraft_category_t::ROTORCRAFT:
            ev = 7;
            break;
         case aircraft_category_t::GLIDER:
            ev = 8;
            break;
         case aircraft_category_t::LTA:
            ev = 9;
            break;
         case aircraft_category_t::PARACHUTE:
            ev = 10;
            break;
         case aircraft_category_t::UAV:
            ev = 11;
            break;
         case aircraft_category_t::SPACE:
            ev = 12;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1748] = u.b[0];
         raw[1749] = u.b[1];
         raw[1750] = u.b[2];
         raw[1751] = u.b[3];
      } 
      { //write enum airframe = airframe::airframe_end
         uint32_t ev = airframe::airframe_end;
         switch(data.tracks[9].aircraft_model) {
         case airframe::B717:
            ev = 0;
            break;
         case airframe::B727:
            ev = 1;
            break;
         case airframe::B737:
            ev = 2;
            break;
         case airframe::B747:
            ev = 3;
            break;
         case airframe::B757:
            ev = 4;
            break;
         case airframe::B767:
            ev = 5;
            break;
         case airframe::B777:
            ev = 6;
            break;
         case airframe::B787:
            ev = 7;
            break;
         case airframe::A310:
            ev = 8;
            break;
         case airframe::A319:
            ev = 9;
            break;
         case airframe::A320:
            ev = 10;
            break;
         case airframe::A321:
            ev = 11;
            break;
         case airframe::A330:
            ev = 12;
            break;
         case airframe::A340:
            ev = 13;
            break;
         case airframe::C172:
            ev = 14;
            break;
         case airframe::SCE:
            ev = 15;
            break;
         case airframe::MUG:
            ev = 16;
            break;
         case airframe::UNKNOWN_AIRFRAME:
            ev = 17;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1752] = u.b[0];
         raw[1753] = u.b[1];
         raw[1754] = u.b[2];
         raw[1755] = u.b[3];
      } 
      { //write bitfield source
         uint8_t bits[2] = {0x00u, 0x00u};
         if(data.tracks[9].source_data.UNFUSED_TRACK == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[9].source_data.FUSED_TRACK == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[9].source_data.EO_IR_CAMERA_TRACK == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[9].source_data.ADSB_TRACK == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[9].source_data.ACAS_TRACK == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[9].source_data.RADAR_TRACK == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[9].source_data.LIDAR == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[9].source_data.ACOUSTIC == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[9].source_data.UNKNOWN_SOURCE == true) {
            bits[1] |= (0x01u << 0);
         }
         raw[1756] = bits[0];
         raw[1757] = bits[1];
      } 
      { //write data.tracks[9].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].track_state.relative_range;
         raw[1758] = __convert_float32_to_bytes.b[0];
         raw[1759] = __convert_float32_to_bytes.b[1];
         raw[1760] = __convert_float32_to_bytes.b[2];
         raw[1761] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].track_state.relative_bearing;
         raw[1762] = __convert_float32_to_bytes.b[0];
         raw[1763] = __convert_float32_to_bytes.b[1];
         raw[1764] = __convert_float32_to_bytes.b[2];
         raw[1765] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.tracks[9].track_state.bearing_invalid;
         raw[1766] = __convert_uint8_to_bytes.b[0];
      } 
      { //write data.tracks[9].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].track_state.relative_ground_speed_north;
         raw[1767] = __convert_float32_to_bytes.b[0];
         raw[1768] = __convert_float32_to_bytes.b[1];
         raw[1769] = __convert_float32_to_bytes.b[2];
         raw[1770] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].track_state.relative_ground_speed_east;
         raw[1771] = __convert_float32_to_bytes.b[0];
         raw[1772] = __convert_float32_to_bytes.b[1];
         raw[1773] = __convert_float32_to_bytes.b[2];
         raw[1774] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[9].track_state.relative_altitude;
         raw[1775] = __convert_int16_to_bytes.b[0];
         raw[1776] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[9].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].track_state.relative_vertical_speed;
         raw[1777] = __convert_float32_to_bytes.b[0];
         raw[1778] = __convert_float32_to_bytes.b[1];
         raw[1779] = __convert_float32_to_bytes.b[2];
         raw[1780] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum track_size_t = track_size_t::track_size_t_end
         uint32_t ev = track_size_t::track_size_t_end;
         switch(data.tracks[9].track_state.track_size) {
         case track_size_t::UNKNOWN_TRACK_SIZE:
            ev = 0;
            break;
         case track_size_t::SMALL_TRACK:
            ev = 1;
            break;
         case track_size_t::MEDIUM_TRACK:
            ev = 2;
            break;
         case track_size_t::LARGE_TRACK:
            ev = 3;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1781] = u.b[0];
         raw[1782] = u.b[1];
         raw[1783] = u.b[2];
         raw[1784] = u.b[3];
      } 
      { //write data.tracks[9].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.a = data.tracks[9].track_state.relative_range_uncertainty;
         raw[1785] = __convert_uint32_to_bytes.b[0];
         raw[1786] = __convert_uint32_to_bytes.b[1];
         raw[1787] = __convert_uint32_to_bytes.b[2];
         raw[1788] = __convert_uint32_to_bytes.b[3];
      } 
      { //write data.tracks[9].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].track_state.relative_bearing_uncertainty;
         raw[1789] = __convert_float32_to_bytes.b[0];
         raw[1790] = __convert_float32_to_bytes.b[1];
         raw[1791] = __convert_float32_to_bytes.b[2];
         raw[1792] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].track_state.relative_ground_speed_north_uncertainty;
         raw[1793] = __convert_float32_to_bytes.b[0];
         raw[1794] = __convert_float32_to_bytes.b[1];
         raw[1795] = __convert_float32_to_bytes.b[2];
         raw[1796] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].track_state.relative_ground_speed_east_uncertainty;
         raw[1797] = __convert_float32_to_bytes.b[0];
         raw[1798] = __convert_float32_to_bytes.b[1];
         raw[1799] = __convert_float32_to_bytes.b[2];
         raw[1800] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.a = data.tracks[9].track_state.relative_altitude_uncertainty;
         raw[1801] = __convert_int16_to_bytes.b[0];
         raw[1802] = __convert_int16_to_bytes.b[1];
      } 
      { //write data.tracks[9].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].track_state.relative_vertical_speed_uncertainty;
         raw[1803] = __convert_float32_to_bytes.b[0];
         raw[1804] = __convert_float32_to_bytes.b[1];
         raw[1805] = __convert_float32_to_bytes.b[2];
         raw[1806] = __convert_float32_to_bytes.b[3];
      } 
      { //write bitfield state_variable
         uint8_t bits[3] = {0x00u, 0x00u, 0x00u};
         if(data.tracks[9].aircraft_state.available_states.TOA == true) {
            bits[0] |= (0x01u << 0);
         }
         if(data.tracks[9].aircraft_state.available_states.LONGITUDE == true) {
            bits[0] |= (0x01u << 1);
         }
         if(data.tracks[9].aircraft_state.available_states.LATITUDE == true) {
            bits[0] |= (0x01u << 2);
         }
         if(data.tracks[9].aircraft_state.available_states.GEOMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 3);
         }
         if(data.tracks[9].aircraft_state.available_states.BAROMETRIC_ALTITUDE == true) {
            bits[0] |= (0x01u << 4);
         }
         if(data.tracks[9].aircraft_state.available_states.TRUE_HEADING == true) {
            bits[0] |= (0x01u << 5);
         }
         if(data.tracks[9].aircraft_state.available_states.MAGNETIC_HEADING == true) {
            bits[0] |= (0x01u << 6);
         }
         if(data.tracks[9].aircraft_state.available_states.GROUND_TRACK_ANGLE == true) {
            bits[0] |= (0x01u << 7);
         }
         if(data.tracks[9].aircraft_state.available_states.TRUE_AIRSPEED == true) {
            bits[1] |= (0x01u << 0);
         }
         if(data.tracks[9].aircraft_state.available_states.INDICATED_AISPEED == true) {
            bits[1] |= (0x01u << 1);
         }
         if(data.tracks[9].aircraft_state.available_states.GROUND_SPEED_NORTH == true) {
            bits[1] |= (0x01u << 2);
         }
         if(data.tracks[9].aircraft_state.available_states.GROUND_SPEED_EAST == true) {
            bits[1] |= (0x01u << 3);
         }
         if(data.tracks[9].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 4);
         }
         if(data.tracks[9].aircraft_state.available_states.PRESSURE_VERTICAL_RATE == true) {
            bits[1] |= (0x01u << 5);
         }
         if(data.tracks[9].aircraft_state.available_states.HEADING_RATE == true) {
            bits[1] |= (0x01u << 6);
         }
         if(data.tracks[9].aircraft_state.available_states.GROUND_TRACK_RATE == true) {
            bits[1] |= (0x01u << 7);
         }
         if(data.tracks[9].aircraft_state.available_states.MODE == true) {
            bits[2] |= (0x01u << 0);
         }
         raw[1807] = bits[0];
         raw[1808] = bits[1];
         raw[1809] = bits[2];
      } 
      { //write data.tracks[9].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.a = data.tracks[9].aircraft_state.toa;
         raw[1810] = __convert_uint64_to_bytes.b[0];
         raw[1811] = __convert_uint64_to_bytes.b[1];
         raw[1812] = __convert_uint64_to_bytes.b[2];
         raw[1813] = __convert_uint64_to_bytes.b[3];
         raw[1814] = __convert_uint64_to_bytes.b[4];
         raw[1815] = __convert_uint64_to_bytes.b[5];
         raw[1816] = __convert_uint64_to_bytes.b[6];
         raw[1817] = __convert_uint64_to_bytes.b[7];
      } 
      { //write data.tracks[9].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[9].aircraft_state.longitude;
         raw[1818] = __convert_float64_to_bytes.b[0];
         raw[1819] = __convert_float64_to_bytes.b[1];
         raw[1820] = __convert_float64_to_bytes.b[2];
         raw[1821] = __convert_float64_to_bytes.b[3];
         raw[1822] = __convert_float64_to_bytes.b[4];
         raw[1823] = __convert_float64_to_bytes.b[5];
         raw[1824] = __convert_float64_to_bytes.b[6];
         raw[1825] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[9].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.a = data.tracks[9].aircraft_state.latitude;
         raw[1826] = __convert_float64_to_bytes.b[0];
         raw[1827] = __convert_float64_to_bytes.b[1];
         raw[1828] = __convert_float64_to_bytes.b[2];
         raw[1829] = __convert_float64_to_bytes.b[3];
         raw[1830] = __convert_float64_to_bytes.b[4];
         raw[1831] = __convert_float64_to_bytes.b[5];
         raw[1832] = __convert_float64_to_bytes.b[6];
         raw[1833] = __convert_float64_to_bytes.b[7];
      } 
      { //write data.tracks[9].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.geometric_altitude;
         raw[1834] = __convert_float32_to_bytes.b[0];
         raw[1835] = __convert_float32_to_bytes.b[1];
         raw[1836] = __convert_float32_to_bytes.b[2];
         raw[1837] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.barometric_altitude;
         raw[1838] = __convert_float32_to_bytes.b[0];
         raw[1839] = __convert_float32_to_bytes.b[1];
         raw[1840] = __convert_float32_to_bytes.b[2];
         raw[1841] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.true_heading;
         raw[1842] = __convert_float32_to_bytes.b[0];
         raw[1843] = __convert_float32_to_bytes.b[1];
         raw[1844] = __convert_float32_to_bytes.b[2];
         raw[1845] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.magnetic_heading;
         raw[1846] = __convert_float32_to_bytes.b[0];
         raw[1847] = __convert_float32_to_bytes.b[1];
         raw[1848] = __convert_float32_to_bytes.b[2];
         raw[1849] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.ground_track_angle;
         raw[1850] = __convert_float32_to_bytes.b[0];
         raw[1851] = __convert_float32_to_bytes.b[1];
         raw[1852] = __convert_float32_to_bytes.b[2];
         raw[1853] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.true_air_speed;
         raw[1854] = __convert_float32_to_bytes.b[0];
         raw[1855] = __convert_float32_to_bytes.b[1];
         raw[1856] = __convert_float32_to_bytes.b[2];
         raw[1857] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.indicated_air_speed;
         raw[1858] = __convert_float32_to_bytes.b[0];
         raw[1859] = __convert_float32_to_bytes.b[1];
         raw[1860] = __convert_float32_to_bytes.b[2];
         raw[1861] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.ground_speed_north;
         raw[1862] = __convert_float32_to_bytes.b[0];
         raw[1863] = __convert_float32_to_bytes.b[1];
         raw[1864] = __convert_float32_to_bytes.b[2];
         raw[1865] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.ground_speed_east;
         raw[1866] = __convert_float32_to_bytes.b[0];
         raw[1867] = __convert_float32_to_bytes.b[1];
         raw[1868] = __convert_float32_to_bytes.b[2];
         raw[1869] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.geometric_vertical_rate;
         raw[1870] = __convert_float32_to_bytes.b[0];
         raw[1871] = __convert_float32_to_bytes.b[1];
         raw[1872] = __convert_float32_to_bytes.b[2];
         raw[1873] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.pressure_vertical_rate;
         raw[1874] = __convert_float32_to_bytes.b[0];
         raw[1875] = __convert_float32_to_bytes.b[1];
         raw[1876] = __convert_float32_to_bytes.b[2];
         raw[1877] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.heading_rate;
         raw[1878] = __convert_float32_to_bytes.b[0];
         raw[1879] = __convert_float32_to_bytes.b[1];
         raw[1880] = __convert_float32_to_bytes.b[2];
         raw[1881] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.ground_track_rate;
         raw[1882] = __convert_float32_to_bytes.b[0];
         raw[1883] = __convert_float32_to_bytes.b[1];
         raw[1884] = __convert_float32_to_bytes.b[2];
         raw[1885] = __convert_float32_to_bytes.b[3];
      } 
      { //write enum flight_mode = flight_mode::flight_mode_end
         uint32_t ev = flight_mode::flight_mode_end;
         switch(data.tracks[9].aircraft_state.mode) {
         case flight_mode::AT_GATE_HANGAR:
            ev = 0;
            break;
         case flight_mode::TAXI:
            ev = 1;
            break;
         case flight_mode::HOLD:
            ev = 2;
            break;
         case flight_mode::TAKE_OFF:
            ev = 3;
            break;
         case flight_mode::CLIMB:
            ev = 4;
            break;
         case flight_mode::CRUISE:
            ev = 5;
            break;
         case flight_mode::DESCENT:
            ev = 6;
            break;
         case flight_mode::LAND:
            ev = 7;
            break;
         }
         union {
            uint32_t a;
            char b[4];
         } u;
         u.a = ev;
         raw[1886] = u.b[0];
         raw[1887] = u.b[1];
         raw[1888] = u.b[2];
         raw[1889] = u.b[3];
      } 
      { //write data.tracks[9].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[9].aircraft_state.horizonal_position_uncertainty;
         raw[1890] = __convert_uint16_to_bytes.b[0];
         raw[1891] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[9].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[9].aircraft_state.pressure_altitude_uncertainty;
         raw[1892] = __convert_uint16_to_bytes.b[0];
         raw[1893] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[9].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.a = data.tracks[9].aircraft_state.geometric_altitude_uncertainty;
         raw[1894] = __convert_uint16_to_bytes.b[0];
         raw[1895] = __convert_uint16_to_bytes.b[1];
      } 
      { //write data.tracks[9].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.ground_speed_uncertainty;
         raw[1896] = __convert_float32_to_bytes.b[0];
         raw[1897] = __convert_float32_to_bytes.b[1];
         raw[1898] = __convert_float32_to_bytes.b[2];
         raw[1899] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.airspeed_uncertainty;
         raw[1900] = __convert_float32_to_bytes.b[0];
         raw[1901] = __convert_float32_to_bytes.b[1];
         raw[1902] = __convert_float32_to_bytes.b[2];
         raw[1903] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.heading_uncertainty;
         raw[1904] = __convert_float32_to_bytes.b[0];
         raw[1905] = __convert_float32_to_bytes.b[1];
         raw[1906] = __convert_float32_to_bytes.b[2];
         raw[1907] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.ground_track_angle_uncertainty;
         raw[1908] = __convert_float32_to_bytes.b[0];
         raw[1909] = __convert_float32_to_bytes.b[1];
         raw[1910] = __convert_float32_to_bytes.b[2];
         raw[1911] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.heading_rate_uncertainty;
         raw[1912] = __convert_float32_to_bytes.b[0];
         raw[1913] = __convert_float32_to_bytes.b[1];
         raw[1914] = __convert_float32_to_bytes.b[2];
         raw[1915] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.tracks[9].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.a = data.tracks[9].aircraft_state.ground_track_rate_uncertainty;
         raw[1916] = __convert_float32_to_bytes.b[0];
         raw[1917] = __convert_float32_to_bytes.b[1];
         raw[1918] = __convert_float32_to_bytes.b[2];
         raw[1919] = __convert_float32_to_bytes.b[3];
      } 
      { //write data.trackCount        uint8        1 bytes
         __convert_uint8_to_bytes.a = data.trackCount;
         raw[1920] = __convert_uint8_to_bytes.b[0];
      } 
   }

//NEW -- Reader functions
   void read_aircraft_state_t(aircraft_state_t& data, const aircraft_state_t_buffer& raw) { 
      read_aircraft_state_t_ptr(data, &(raw[0]));
   };
   void read_aircraft_state_t_ptr(aircraft_state_t& data, const char* raw) {
      { //READ bitfield state_variable
         data.available_states.TOA = ( ((0x01u << 0) & raw[0]) != 0x00u );
         data.available_states.LONGITUDE = ( ((0x01u << 1) & raw[0]) != 0x00u );
         data.available_states.LATITUDE = ( ((0x01u << 2) & raw[0]) != 0x00u );
         data.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[0]) != 0x00u );
         data.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[0]) != 0x00u );
         data.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[0]) != 0x00u );
         data.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[0]) != 0x00u );
         data.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[0]) != 0x00u );
         data.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[1]) != 0x00u );
         data.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[1]) != 0x00u );
         data.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[1]) != 0x00u );
         data.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[1]) != 0x00u );
         data.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[1]) != 0x00u );
         data.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[1]) != 0x00u );
         data.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[1]) != 0x00u );
         data.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[1]) != 0x00u );
         data.available_states.MODE = ( ((0x01u << 0) & raw[2]) != 0x00u );
      } 
      { //READ data.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[3];
         __convert_uint64_to_bytes.b[1] = raw[4];
         __convert_uint64_to_bytes.b[2] = raw[5];
         __convert_uint64_to_bytes.b[3] = raw[6];
         __convert_uint64_to_bytes.b[4] = raw[7];
         __convert_uint64_to_bytes.b[5] = raw[8];
         __convert_uint64_to_bytes.b[6] = raw[9];
         __convert_uint64_to_bytes.b[7] = raw[10];
         data.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[11];
         __convert_float64_to_bytes.b[1] = raw[12];
         __convert_float64_to_bytes.b[2] = raw[13];
         __convert_float64_to_bytes.b[3] = raw[14];
         __convert_float64_to_bytes.b[4] = raw[15];
         __convert_float64_to_bytes.b[5] = raw[16];
         __convert_float64_to_bytes.b[6] = raw[17];
         __convert_float64_to_bytes.b[7] = raw[18];
         data.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[19];
         __convert_float64_to_bytes.b[1] = raw[20];
         __convert_float64_to_bytes.b[2] = raw[21];
         __convert_float64_to_bytes.b[3] = raw[22];
         __convert_float64_to_bytes.b[4] = raw[23];
         __convert_float64_to_bytes.b[5] = raw[24];
         __convert_float64_to_bytes.b[6] = raw[25];
         __convert_float64_to_bytes.b[7] = raw[26];
         data.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[27];
         __convert_float32_to_bytes.b[1] = raw[28];
         __convert_float32_to_bytes.b[2] = raw[29];
         __convert_float32_to_bytes.b[3] = raw[30];
         data.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[31];
         __convert_float32_to_bytes.b[1] = raw[32];
         __convert_float32_to_bytes.b[2] = raw[33];
         __convert_float32_to_bytes.b[3] = raw[34];
         data.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[35];
         __convert_float32_to_bytes.b[1] = raw[36];
         __convert_float32_to_bytes.b[2] = raw[37];
         __convert_float32_to_bytes.b[3] = raw[38];
         data.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[39];
         __convert_float32_to_bytes.b[1] = raw[40];
         __convert_float32_to_bytes.b[2] = raw[41];
         __convert_float32_to_bytes.b[3] = raw[42];
         data.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[43];
         __convert_float32_to_bytes.b[1] = raw[44];
         __convert_float32_to_bytes.b[2] = raw[45];
         __convert_float32_to_bytes.b[3] = raw[46];
         data.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[47];
         __convert_float32_to_bytes.b[1] = raw[48];
         __convert_float32_to_bytes.b[2] = raw[49];
         __convert_float32_to_bytes.b[3] = raw[50];
         data.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[51];
         __convert_float32_to_bytes.b[1] = raw[52];
         __convert_float32_to_bytes.b[2] = raw[53];
         __convert_float32_to_bytes.b[3] = raw[54];
         data.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[55];
         __convert_float32_to_bytes.b[1] = raw[56];
         __convert_float32_to_bytes.b[2] = raw[57];
         __convert_float32_to_bytes.b[3] = raw[58];
         data.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[59];
         __convert_float32_to_bytes.b[1] = raw[60];
         __convert_float32_to_bytes.b[2] = raw[61];
         __convert_float32_to_bytes.b[3] = raw[62];
         data.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[63];
         __convert_float32_to_bytes.b[1] = raw[64];
         __convert_float32_to_bytes.b[2] = raw[65];
         __convert_float32_to_bytes.b[3] = raw[66];
         data.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[67];
         __convert_float32_to_bytes.b[1] = raw[68];
         __convert_float32_to_bytes.b[2] = raw[69];
         __convert_float32_to_bytes.b[3] = raw[70];
         data.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[71];
         __convert_float32_to_bytes.b[1] = raw[72];
         __convert_float32_to_bytes.b[2] = raw[73];
         __convert_float32_to_bytes.b[3] = raw[74];
         data.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[75];
         __convert_float32_to_bytes.b[1] = raw[76];
         __convert_float32_to_bytes.b[2] = raw[77];
         __convert_float32_to_bytes.b[3] = raw[78];
         data.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[79];
         u.b[1] = raw[80];
         u.b[2] = raw[81];
         u.b[3] = raw[82];
        switch(u.a) {
         case 0:
            data.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.mode = flight_mode::TAXI;
            break;
         case 2:
            data.mode = flight_mode::HOLD;
            break;
         case 3:
            data.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[83];
         __convert_uint16_to_bytes.b[1] = raw[84];
         data.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[85];
         __convert_uint16_to_bytes.b[1] = raw[86];
         data.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[87];
         __convert_uint16_to_bytes.b[1] = raw[88];
         data.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[89];
         __convert_float32_to_bytes.b[1] = raw[90];
         __convert_float32_to_bytes.b[2] = raw[91];
         __convert_float32_to_bytes.b[3] = raw[92];
         data.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[93];
         __convert_float32_to_bytes.b[1] = raw[94];
         __convert_float32_to_bytes.b[2] = raw[95];
         __convert_float32_to_bytes.b[3] = raw[96];
         data.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[97];
         __convert_float32_to_bytes.b[1] = raw[98];
         __convert_float32_to_bytes.b[2] = raw[99];
         __convert_float32_to_bytes.b[3] = raw[100];
         data.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[101];
         __convert_float32_to_bytes.b[1] = raw[102];
         __convert_float32_to_bytes.b[2] = raw[103];
         __convert_float32_to_bytes.b[3] = raw[104];
         data.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[105];
         __convert_float32_to_bytes.b[1] = raw[106];
         __convert_float32_to_bytes.b[2] = raw[107];
         __convert_float32_to_bytes.b[3] = raw[108];
         data.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[109];
         __convert_float32_to_bytes.b[1] = raw[110];
         __convert_float32_to_bytes.b[2] = raw[111];
         __convert_float32_to_bytes.b[3] = raw[112];
         data.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
   }
   void read_track(track& data, const track_buffer& raw) { 
      read_track_ptr(data, &(raw[0]));
   };
   void read_track_ptr(track& data, const char* raw) {
      { //READ data.track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[0];
         __convert_uint32_to_bytes.b[1] = raw[1];
         __convert_uint32_to_bytes.b[2] = raw[2];
         __convert_uint32_to_bytes.b[3] = raw[3];
         data.track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[4];
         data.callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[5];
         data.callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[6];
         data.callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[7];
         data.callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[8];
         data.callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[9];
         data.callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[10];
         data.callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[11];
         data.callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[12];
         __convert_uint64_to_bytes.b[1] = raw[13];
         __convert_uint64_to_bytes.b[2] = raw[14];
         __convert_uint64_to_bytes.b[3] = raw[15];
         __convert_uint64_to_bytes.b[4] = raw[16];
         __convert_uint64_to_bytes.b[5] = raw[17];
         __convert_uint64_to_bytes.b[6] = raw[18];
         __convert_uint64_to_bytes.b[7] = raw[19];
         data.track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[20];
         u.b[1] = raw[21];
         u.b[2] = raw[22];
         u.b[3] = raw[23];
        switch(u.a) {
         case 0:
            data.aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[24];
         u.b[1] = raw[25];
         u.b[2] = raw[26];
         u.b[3] = raw[27];
        switch(u.a) {
         case 0:
            data.aircraft_model = airframe::B717;
            break;
         case 1:
            data.aircraft_model = airframe::B727;
            break;
         case 2:
            data.aircraft_model = airframe::B737;
            break;
         case 3:
            data.aircraft_model = airframe::B747;
            break;
         case 4:
            data.aircraft_model = airframe::B757;
            break;
         case 5:
            data.aircraft_model = airframe::B767;
            break;
         case 6:
            data.aircraft_model = airframe::B777;
            break;
         case 7:
            data.aircraft_model = airframe::B787;
            break;
         case 8:
            data.aircraft_model = airframe::A310;
            break;
         case 9:
            data.aircraft_model = airframe::A319;
            break;
         case 10:
            data.aircraft_model = airframe::A320;
            break;
         case 11:
            data.aircraft_model = airframe::A321;
            break;
         case 12:
            data.aircraft_model = airframe::A330;
            break;
         case 13:
            data.aircraft_model = airframe::A340;
            break;
         case 14:
            data.aircraft_model = airframe::C172;
            break;
         case 15:
            data.aircraft_model = airframe::SCE;
            break;
         case 16:
            data.aircraft_model = airframe::MUG;
            break;
         case 17:
            data.aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[28]) != 0x00u );
         data.source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[28]) != 0x00u );
         data.source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[28]) != 0x00u );
         data.source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[28]) != 0x00u );
         data.source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[28]) != 0x00u );
         data.source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[28]) != 0x00u );
         data.source_data.LIDAR = ( ((0x01u << 6) & raw[28]) != 0x00u );
         data.source_data.ACOUSTIC = ( ((0x01u << 7) & raw[28]) != 0x00u );
         data.source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[29]) != 0x00u );
      } 
      { //READ data.track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[30];
         __convert_float32_to_bytes.b[1] = raw[31];
         __convert_float32_to_bytes.b[2] = raw[32];
         __convert_float32_to_bytes.b[3] = raw[33];
         data.track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[34];
         __convert_float32_to_bytes.b[1] = raw[35];
         __convert_float32_to_bytes.b[2] = raw[36];
         __convert_float32_to_bytes.b[3] = raw[37];
         data.track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[38];
         data.track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[39];
         __convert_float32_to_bytes.b[1] = raw[40];
         __convert_float32_to_bytes.b[2] = raw[41];
         __convert_float32_to_bytes.b[3] = raw[42];
         data.track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[43];
         __convert_float32_to_bytes.b[1] = raw[44];
         __convert_float32_to_bytes.b[2] = raw[45];
         __convert_float32_to_bytes.b[3] = raw[46];
         data.track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[47];
         __convert_int16_to_bytes.b[1] = raw[48];
         data.track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[49];
         __convert_float32_to_bytes.b[1] = raw[50];
         __convert_float32_to_bytes.b[2] = raw[51];
         __convert_float32_to_bytes.b[3] = raw[52];
         data.track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[53];
         u.b[1] = raw[54];
         u.b[2] = raw[55];
         u.b[3] = raw[56];
        switch(u.a) {
         case 0:
            data.track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[57];
         __convert_uint32_to_bytes.b[1] = raw[58];
         __convert_uint32_to_bytes.b[2] = raw[59];
         __convert_uint32_to_bytes.b[3] = raw[60];
         data.track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[61];
         __convert_float32_to_bytes.b[1] = raw[62];
         __convert_float32_to_bytes.b[2] = raw[63];
         __convert_float32_to_bytes.b[3] = raw[64];
         data.track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[65];
         __convert_float32_to_bytes.b[1] = raw[66];
         __convert_float32_to_bytes.b[2] = raw[67];
         __convert_float32_to_bytes.b[3] = raw[68];
         data.track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[69];
         __convert_float32_to_bytes.b[1] = raw[70];
         __convert_float32_to_bytes.b[2] = raw[71];
         __convert_float32_to_bytes.b[3] = raw[72];
         data.track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[73];
         __convert_int16_to_bytes.b[1] = raw[74];
         data.track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[75];
         __convert_float32_to_bytes.b[1] = raw[76];
         __convert_float32_to_bytes.b[2] = raw[77];
         __convert_float32_to_bytes.b[3] = raw[78];
         data.track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[79]) != 0x00u );
         data.aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[79]) != 0x00u );
         data.aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[79]) != 0x00u );
         data.aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[79]) != 0x00u );
         data.aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[79]) != 0x00u );
         data.aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[79]) != 0x00u );
         data.aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[79]) != 0x00u );
         data.aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[79]) != 0x00u );
         data.aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[80]) != 0x00u );
         data.aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[80]) != 0x00u );
         data.aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[80]) != 0x00u );
         data.aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[80]) != 0x00u );
         data.aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[80]) != 0x00u );
         data.aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[80]) != 0x00u );
         data.aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[80]) != 0x00u );
         data.aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[80]) != 0x00u );
         data.aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[81]) != 0x00u );
      } 
      { //READ data.aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[82];
         __convert_uint64_to_bytes.b[1] = raw[83];
         __convert_uint64_to_bytes.b[2] = raw[84];
         __convert_uint64_to_bytes.b[3] = raw[85];
         __convert_uint64_to_bytes.b[4] = raw[86];
         __convert_uint64_to_bytes.b[5] = raw[87];
         __convert_uint64_to_bytes.b[6] = raw[88];
         __convert_uint64_to_bytes.b[7] = raw[89];
         data.aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[90];
         __convert_float64_to_bytes.b[1] = raw[91];
         __convert_float64_to_bytes.b[2] = raw[92];
         __convert_float64_to_bytes.b[3] = raw[93];
         __convert_float64_to_bytes.b[4] = raw[94];
         __convert_float64_to_bytes.b[5] = raw[95];
         __convert_float64_to_bytes.b[6] = raw[96];
         __convert_float64_to_bytes.b[7] = raw[97];
         data.aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[98];
         __convert_float64_to_bytes.b[1] = raw[99];
         __convert_float64_to_bytes.b[2] = raw[100];
         __convert_float64_to_bytes.b[3] = raw[101];
         __convert_float64_to_bytes.b[4] = raw[102];
         __convert_float64_to_bytes.b[5] = raw[103];
         __convert_float64_to_bytes.b[6] = raw[104];
         __convert_float64_to_bytes.b[7] = raw[105];
         data.aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[106];
         __convert_float32_to_bytes.b[1] = raw[107];
         __convert_float32_to_bytes.b[2] = raw[108];
         __convert_float32_to_bytes.b[3] = raw[109];
         data.aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[110];
         __convert_float32_to_bytes.b[1] = raw[111];
         __convert_float32_to_bytes.b[2] = raw[112];
         __convert_float32_to_bytes.b[3] = raw[113];
         data.aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[114];
         __convert_float32_to_bytes.b[1] = raw[115];
         __convert_float32_to_bytes.b[2] = raw[116];
         __convert_float32_to_bytes.b[3] = raw[117];
         data.aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[118];
         __convert_float32_to_bytes.b[1] = raw[119];
         __convert_float32_to_bytes.b[2] = raw[120];
         __convert_float32_to_bytes.b[3] = raw[121];
         data.aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[122];
         __convert_float32_to_bytes.b[1] = raw[123];
         __convert_float32_to_bytes.b[2] = raw[124];
         __convert_float32_to_bytes.b[3] = raw[125];
         data.aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[126];
         __convert_float32_to_bytes.b[1] = raw[127];
         __convert_float32_to_bytes.b[2] = raw[128];
         __convert_float32_to_bytes.b[3] = raw[129];
         data.aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[130];
         __convert_float32_to_bytes.b[1] = raw[131];
         __convert_float32_to_bytes.b[2] = raw[132];
         __convert_float32_to_bytes.b[3] = raw[133];
         data.aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[134];
         __convert_float32_to_bytes.b[1] = raw[135];
         __convert_float32_to_bytes.b[2] = raw[136];
         __convert_float32_to_bytes.b[3] = raw[137];
         data.aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[138];
         __convert_float32_to_bytes.b[1] = raw[139];
         __convert_float32_to_bytes.b[2] = raw[140];
         __convert_float32_to_bytes.b[3] = raw[141];
         data.aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[142];
         __convert_float32_to_bytes.b[1] = raw[143];
         __convert_float32_to_bytes.b[2] = raw[144];
         __convert_float32_to_bytes.b[3] = raw[145];
         data.aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[146];
         __convert_float32_to_bytes.b[1] = raw[147];
         __convert_float32_to_bytes.b[2] = raw[148];
         __convert_float32_to_bytes.b[3] = raw[149];
         data.aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[150];
         __convert_float32_to_bytes.b[1] = raw[151];
         __convert_float32_to_bytes.b[2] = raw[152];
         __convert_float32_to_bytes.b[3] = raw[153];
         data.aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[154];
         __convert_float32_to_bytes.b[1] = raw[155];
         __convert_float32_to_bytes.b[2] = raw[156];
         __convert_float32_to_bytes.b[3] = raw[157];
         data.aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[158];
         u.b[1] = raw[159];
         u.b[2] = raw[160];
         u.b[3] = raw[161];
        switch(u.a) {
         case 0:
            data.aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[162];
         __convert_uint16_to_bytes.b[1] = raw[163];
         data.aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[164];
         __convert_uint16_to_bytes.b[1] = raw[165];
         data.aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[166];
         __convert_uint16_to_bytes.b[1] = raw[167];
         data.aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[168];
         __convert_float32_to_bytes.b[1] = raw[169];
         __convert_float32_to_bytes.b[2] = raw[170];
         __convert_float32_to_bytes.b[3] = raw[171];
         data.aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[172];
         __convert_float32_to_bytes.b[1] = raw[173];
         __convert_float32_to_bytes.b[2] = raw[174];
         __convert_float32_to_bytes.b[3] = raw[175];
         data.aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[176];
         __convert_float32_to_bytes.b[1] = raw[177];
         __convert_float32_to_bytes.b[2] = raw[178];
         __convert_float32_to_bytes.b[3] = raw[179];
         data.aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[180];
         __convert_float32_to_bytes.b[1] = raw[181];
         __convert_float32_to_bytes.b[2] = raw[182];
         __convert_float32_to_bytes.b[3] = raw[183];
         data.aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[184];
         __convert_float32_to_bytes.b[1] = raw[185];
         __convert_float32_to_bytes.b[2] = raw[186];
         __convert_float32_to_bytes.b[3] = raw[187];
         data.aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[188];
         __convert_float32_to_bytes.b[1] = raw[189];
         __convert_float32_to_bytes.b[2] = raw[190];
         __convert_float32_to_bytes.b[3] = raw[191];
         data.aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
   }
   void read_track_list(track_list& data, const track_list_buffer& raw) { 
      read_track_list_ptr(data, &(raw[0]));
   };
   void read_track_list_ptr(track_list& data, const char* raw) {
      { //READ data.tracks[0].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[0];
         __convert_uint32_to_bytes.b[1] = raw[1];
         __convert_uint32_to_bytes.b[2] = raw[2];
         __convert_uint32_to_bytes.b[3] = raw[3];
         data.tracks[0].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[4];
         data.tracks[0].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[5];
         data.tracks[0].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[6];
         data.tracks[0].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[7];
         data.tracks[0].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[8];
         data.tracks[0].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[9];
         data.tracks[0].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[10];
         data.tracks[0].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[0].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[11];
         data.tracks[0].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[0].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[12];
         __convert_uint64_to_bytes.b[1] = raw[13];
         __convert_uint64_to_bytes.b[2] = raw[14];
         __convert_uint64_to_bytes.b[3] = raw[15];
         __convert_uint64_to_bytes.b[4] = raw[16];
         __convert_uint64_to_bytes.b[5] = raw[17];
         __convert_uint64_to_bytes.b[6] = raw[18];
         __convert_uint64_to_bytes.b[7] = raw[19];
         data.tracks[0].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[0].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[20];
         u.b[1] = raw[21];
         u.b[2] = raw[22];
         u.b[3] = raw[23];
        switch(u.a) {
         case 0:
            data.tracks[0].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[0].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[0].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[0].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[0].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[0].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[0].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[0].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[0].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[0].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[0].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[0].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[0].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[0].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[0].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[24];
         u.b[1] = raw[25];
         u.b[2] = raw[26];
         u.b[3] = raw[27];
        switch(u.a) {
         case 0:
            data.tracks[0].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[0].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[0].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[0].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[0].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[0].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[0].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[0].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[0].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[0].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[0].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[0].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[0].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[0].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[0].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[0].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[0].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[0].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[0].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[0].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[28]) != 0x00u );
         data.tracks[0].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[28]) != 0x00u );
         data.tracks[0].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[28]) != 0x00u );
         data.tracks[0].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[28]) != 0x00u );
         data.tracks[0].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[28]) != 0x00u );
         data.tracks[0].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[28]) != 0x00u );
         data.tracks[0].source_data.LIDAR = ( ((0x01u << 6) & raw[28]) != 0x00u );
         data.tracks[0].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[28]) != 0x00u );
         data.tracks[0].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[29]) != 0x00u );
      } 
      { //READ data.tracks[0].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[30];
         __convert_float32_to_bytes.b[1] = raw[31];
         __convert_float32_to_bytes.b[2] = raw[32];
         __convert_float32_to_bytes.b[3] = raw[33];
         data.tracks[0].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[34];
         __convert_float32_to_bytes.b[1] = raw[35];
         __convert_float32_to_bytes.b[2] = raw[36];
         __convert_float32_to_bytes.b[3] = raw[37];
         data.tracks[0].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[38];
         data.tracks[0].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[39];
         __convert_float32_to_bytes.b[1] = raw[40];
         __convert_float32_to_bytes.b[2] = raw[41];
         __convert_float32_to_bytes.b[3] = raw[42];
         data.tracks[0].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[43];
         __convert_float32_to_bytes.b[1] = raw[44];
         __convert_float32_to_bytes.b[2] = raw[45];
         __convert_float32_to_bytes.b[3] = raw[46];
         data.tracks[0].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[47];
         __convert_int16_to_bytes.b[1] = raw[48];
         data.tracks[0].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[49];
         __convert_float32_to_bytes.b[1] = raw[50];
         __convert_float32_to_bytes.b[2] = raw[51];
         __convert_float32_to_bytes.b[3] = raw[52];
         data.tracks[0].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[0].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[53];
         u.b[1] = raw[54];
         u.b[2] = raw[55];
         u.b[3] = raw[56];
        switch(u.a) {
         case 0:
            data.tracks[0].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[0].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[0].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[0].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[0].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[0].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[57];
         __convert_uint32_to_bytes.b[1] = raw[58];
         __convert_uint32_to_bytes.b[2] = raw[59];
         __convert_uint32_to_bytes.b[3] = raw[60];
         data.tracks[0].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[61];
         __convert_float32_to_bytes.b[1] = raw[62];
         __convert_float32_to_bytes.b[2] = raw[63];
         __convert_float32_to_bytes.b[3] = raw[64];
         data.tracks[0].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[65];
         __convert_float32_to_bytes.b[1] = raw[66];
         __convert_float32_to_bytes.b[2] = raw[67];
         __convert_float32_to_bytes.b[3] = raw[68];
         data.tracks[0].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[69];
         __convert_float32_to_bytes.b[1] = raw[70];
         __convert_float32_to_bytes.b[2] = raw[71];
         __convert_float32_to_bytes.b[3] = raw[72];
         data.tracks[0].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[73];
         __convert_int16_to_bytes.b[1] = raw[74];
         data.tracks[0].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[0].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[75];
         __convert_float32_to_bytes.b[1] = raw[76];
         __convert_float32_to_bytes.b[2] = raw[77];
         __convert_float32_to_bytes.b[3] = raw[78];
         data.tracks[0].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[0].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[79]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[79]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[79]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[79]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[79]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[79]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[79]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[79]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[80]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[80]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[80]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[80]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[80]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[80]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[80]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[80]) != 0x00u );
         data.tracks[0].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[81]) != 0x00u );
      } 
      { //READ data.tracks[0].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[82];
         __convert_uint64_to_bytes.b[1] = raw[83];
         __convert_uint64_to_bytes.b[2] = raw[84];
         __convert_uint64_to_bytes.b[3] = raw[85];
         __convert_uint64_to_bytes.b[4] = raw[86];
         __convert_uint64_to_bytes.b[5] = raw[87];
         __convert_uint64_to_bytes.b[6] = raw[88];
         __convert_uint64_to_bytes.b[7] = raw[89];
         data.tracks[0].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[90];
         __convert_float64_to_bytes.b[1] = raw[91];
         __convert_float64_to_bytes.b[2] = raw[92];
         __convert_float64_to_bytes.b[3] = raw[93];
         __convert_float64_to_bytes.b[4] = raw[94];
         __convert_float64_to_bytes.b[5] = raw[95];
         __convert_float64_to_bytes.b[6] = raw[96];
         __convert_float64_to_bytes.b[7] = raw[97];
         data.tracks[0].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[98];
         __convert_float64_to_bytes.b[1] = raw[99];
         __convert_float64_to_bytes.b[2] = raw[100];
         __convert_float64_to_bytes.b[3] = raw[101];
         __convert_float64_to_bytes.b[4] = raw[102];
         __convert_float64_to_bytes.b[5] = raw[103];
         __convert_float64_to_bytes.b[6] = raw[104];
         __convert_float64_to_bytes.b[7] = raw[105];
         data.tracks[0].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[106];
         __convert_float32_to_bytes.b[1] = raw[107];
         __convert_float32_to_bytes.b[2] = raw[108];
         __convert_float32_to_bytes.b[3] = raw[109];
         data.tracks[0].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[110];
         __convert_float32_to_bytes.b[1] = raw[111];
         __convert_float32_to_bytes.b[2] = raw[112];
         __convert_float32_to_bytes.b[3] = raw[113];
         data.tracks[0].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[114];
         __convert_float32_to_bytes.b[1] = raw[115];
         __convert_float32_to_bytes.b[2] = raw[116];
         __convert_float32_to_bytes.b[3] = raw[117];
         data.tracks[0].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[118];
         __convert_float32_to_bytes.b[1] = raw[119];
         __convert_float32_to_bytes.b[2] = raw[120];
         __convert_float32_to_bytes.b[3] = raw[121];
         data.tracks[0].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[122];
         __convert_float32_to_bytes.b[1] = raw[123];
         __convert_float32_to_bytes.b[2] = raw[124];
         __convert_float32_to_bytes.b[3] = raw[125];
         data.tracks[0].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[126];
         __convert_float32_to_bytes.b[1] = raw[127];
         __convert_float32_to_bytes.b[2] = raw[128];
         __convert_float32_to_bytes.b[3] = raw[129];
         data.tracks[0].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[130];
         __convert_float32_to_bytes.b[1] = raw[131];
         __convert_float32_to_bytes.b[2] = raw[132];
         __convert_float32_to_bytes.b[3] = raw[133];
         data.tracks[0].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[134];
         __convert_float32_to_bytes.b[1] = raw[135];
         __convert_float32_to_bytes.b[2] = raw[136];
         __convert_float32_to_bytes.b[3] = raw[137];
         data.tracks[0].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[138];
         __convert_float32_to_bytes.b[1] = raw[139];
         __convert_float32_to_bytes.b[2] = raw[140];
         __convert_float32_to_bytes.b[3] = raw[141];
         data.tracks[0].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[142];
         __convert_float32_to_bytes.b[1] = raw[143];
         __convert_float32_to_bytes.b[2] = raw[144];
         __convert_float32_to_bytes.b[3] = raw[145];
         data.tracks[0].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[146];
         __convert_float32_to_bytes.b[1] = raw[147];
         __convert_float32_to_bytes.b[2] = raw[148];
         __convert_float32_to_bytes.b[3] = raw[149];
         data.tracks[0].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[150];
         __convert_float32_to_bytes.b[1] = raw[151];
         __convert_float32_to_bytes.b[2] = raw[152];
         __convert_float32_to_bytes.b[3] = raw[153];
         data.tracks[0].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[154];
         __convert_float32_to_bytes.b[1] = raw[155];
         __convert_float32_to_bytes.b[2] = raw[156];
         __convert_float32_to_bytes.b[3] = raw[157];
         data.tracks[0].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[0].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[158];
         u.b[1] = raw[159];
         u.b[2] = raw[160];
         u.b[3] = raw[161];
        switch(u.a) {
         case 0:
            data.tracks[0].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[0].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[0].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[0].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[0].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[0].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[0].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[0].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[0].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[0].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[162];
         __convert_uint16_to_bytes.b[1] = raw[163];
         data.tracks[0].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[164];
         __convert_uint16_to_bytes.b[1] = raw[165];
         data.tracks[0].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[166];
         __convert_uint16_to_bytes.b[1] = raw[167];
         data.tracks[0].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[168];
         __convert_float32_to_bytes.b[1] = raw[169];
         __convert_float32_to_bytes.b[2] = raw[170];
         __convert_float32_to_bytes.b[3] = raw[171];
         data.tracks[0].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[172];
         __convert_float32_to_bytes.b[1] = raw[173];
         __convert_float32_to_bytes.b[2] = raw[174];
         __convert_float32_to_bytes.b[3] = raw[175];
         data.tracks[0].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[176];
         __convert_float32_to_bytes.b[1] = raw[177];
         __convert_float32_to_bytes.b[2] = raw[178];
         __convert_float32_to_bytes.b[3] = raw[179];
         data.tracks[0].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[180];
         __convert_float32_to_bytes.b[1] = raw[181];
         __convert_float32_to_bytes.b[2] = raw[182];
         __convert_float32_to_bytes.b[3] = raw[183];
         data.tracks[0].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[184];
         __convert_float32_to_bytes.b[1] = raw[185];
         __convert_float32_to_bytes.b[2] = raw[186];
         __convert_float32_to_bytes.b[3] = raw[187];
         data.tracks[0].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[0].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[188];
         __convert_float32_to_bytes.b[1] = raw[189];
         __convert_float32_to_bytes.b[2] = raw[190];
         __convert_float32_to_bytes.b[3] = raw[191];
         data.tracks[0].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[192];
         __convert_uint32_to_bytes.b[1] = raw[193];
         __convert_uint32_to_bytes.b[2] = raw[194];
         __convert_uint32_to_bytes.b[3] = raw[195];
         data.tracks[1].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[196];
         data.tracks[1].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[197];
         data.tracks[1].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[198];
         data.tracks[1].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[199];
         data.tracks[1].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[200];
         data.tracks[1].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[201];
         data.tracks[1].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[202];
         data.tracks[1].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[1].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[203];
         data.tracks[1].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[1].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[204];
         __convert_uint64_to_bytes.b[1] = raw[205];
         __convert_uint64_to_bytes.b[2] = raw[206];
         __convert_uint64_to_bytes.b[3] = raw[207];
         __convert_uint64_to_bytes.b[4] = raw[208];
         __convert_uint64_to_bytes.b[5] = raw[209];
         __convert_uint64_to_bytes.b[6] = raw[210];
         __convert_uint64_to_bytes.b[7] = raw[211];
         data.tracks[1].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[1].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[212];
         u.b[1] = raw[213];
         u.b[2] = raw[214];
         u.b[3] = raw[215];
        switch(u.a) {
         case 0:
            data.tracks[1].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[1].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[1].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[1].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[1].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[1].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[1].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[1].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[1].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[1].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[1].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[1].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[1].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[1].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[1].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[216];
         u.b[1] = raw[217];
         u.b[2] = raw[218];
         u.b[3] = raw[219];
        switch(u.a) {
         case 0:
            data.tracks[1].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[1].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[1].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[1].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[1].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[1].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[1].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[1].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[1].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[1].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[1].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[1].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[1].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[1].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[1].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[1].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[1].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[1].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[1].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[1].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[220]) != 0x00u );
         data.tracks[1].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[220]) != 0x00u );
         data.tracks[1].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[220]) != 0x00u );
         data.tracks[1].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[220]) != 0x00u );
         data.tracks[1].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[220]) != 0x00u );
         data.tracks[1].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[220]) != 0x00u );
         data.tracks[1].source_data.LIDAR = ( ((0x01u << 6) & raw[220]) != 0x00u );
         data.tracks[1].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[220]) != 0x00u );
         data.tracks[1].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[221]) != 0x00u );
      } 
      { //READ data.tracks[1].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[222];
         __convert_float32_to_bytes.b[1] = raw[223];
         __convert_float32_to_bytes.b[2] = raw[224];
         __convert_float32_to_bytes.b[3] = raw[225];
         data.tracks[1].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[226];
         __convert_float32_to_bytes.b[1] = raw[227];
         __convert_float32_to_bytes.b[2] = raw[228];
         __convert_float32_to_bytes.b[3] = raw[229];
         data.tracks[1].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[230];
         data.tracks[1].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[231];
         __convert_float32_to_bytes.b[1] = raw[232];
         __convert_float32_to_bytes.b[2] = raw[233];
         __convert_float32_to_bytes.b[3] = raw[234];
         data.tracks[1].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[235];
         __convert_float32_to_bytes.b[1] = raw[236];
         __convert_float32_to_bytes.b[2] = raw[237];
         __convert_float32_to_bytes.b[3] = raw[238];
         data.tracks[1].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[239];
         __convert_int16_to_bytes.b[1] = raw[240];
         data.tracks[1].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[241];
         __convert_float32_to_bytes.b[1] = raw[242];
         __convert_float32_to_bytes.b[2] = raw[243];
         __convert_float32_to_bytes.b[3] = raw[244];
         data.tracks[1].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[1].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[245];
         u.b[1] = raw[246];
         u.b[2] = raw[247];
         u.b[3] = raw[248];
        switch(u.a) {
         case 0:
            data.tracks[1].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[1].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[1].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[1].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[1].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[1].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[249];
         __convert_uint32_to_bytes.b[1] = raw[250];
         __convert_uint32_to_bytes.b[2] = raw[251];
         __convert_uint32_to_bytes.b[3] = raw[252];
         data.tracks[1].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[253];
         __convert_float32_to_bytes.b[1] = raw[254];
         __convert_float32_to_bytes.b[2] = raw[255];
         __convert_float32_to_bytes.b[3] = raw[256];
         data.tracks[1].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[257];
         __convert_float32_to_bytes.b[1] = raw[258];
         __convert_float32_to_bytes.b[2] = raw[259];
         __convert_float32_to_bytes.b[3] = raw[260];
         data.tracks[1].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[261];
         __convert_float32_to_bytes.b[1] = raw[262];
         __convert_float32_to_bytes.b[2] = raw[263];
         __convert_float32_to_bytes.b[3] = raw[264];
         data.tracks[1].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[265];
         __convert_int16_to_bytes.b[1] = raw[266];
         data.tracks[1].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[1].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[267];
         __convert_float32_to_bytes.b[1] = raw[268];
         __convert_float32_to_bytes.b[2] = raw[269];
         __convert_float32_to_bytes.b[3] = raw[270];
         data.tracks[1].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[1].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[271]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[271]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[271]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[271]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[271]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[271]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[271]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[271]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[272]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[272]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[272]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[272]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[272]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[272]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[272]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[272]) != 0x00u );
         data.tracks[1].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[273]) != 0x00u );
      } 
      { //READ data.tracks[1].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[274];
         __convert_uint64_to_bytes.b[1] = raw[275];
         __convert_uint64_to_bytes.b[2] = raw[276];
         __convert_uint64_to_bytes.b[3] = raw[277];
         __convert_uint64_to_bytes.b[4] = raw[278];
         __convert_uint64_to_bytes.b[5] = raw[279];
         __convert_uint64_to_bytes.b[6] = raw[280];
         __convert_uint64_to_bytes.b[7] = raw[281];
         data.tracks[1].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[282];
         __convert_float64_to_bytes.b[1] = raw[283];
         __convert_float64_to_bytes.b[2] = raw[284];
         __convert_float64_to_bytes.b[3] = raw[285];
         __convert_float64_to_bytes.b[4] = raw[286];
         __convert_float64_to_bytes.b[5] = raw[287];
         __convert_float64_to_bytes.b[6] = raw[288];
         __convert_float64_to_bytes.b[7] = raw[289];
         data.tracks[1].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[290];
         __convert_float64_to_bytes.b[1] = raw[291];
         __convert_float64_to_bytes.b[2] = raw[292];
         __convert_float64_to_bytes.b[3] = raw[293];
         __convert_float64_to_bytes.b[4] = raw[294];
         __convert_float64_to_bytes.b[5] = raw[295];
         __convert_float64_to_bytes.b[6] = raw[296];
         __convert_float64_to_bytes.b[7] = raw[297];
         data.tracks[1].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[298];
         __convert_float32_to_bytes.b[1] = raw[299];
         __convert_float32_to_bytes.b[2] = raw[300];
         __convert_float32_to_bytes.b[3] = raw[301];
         data.tracks[1].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[302];
         __convert_float32_to_bytes.b[1] = raw[303];
         __convert_float32_to_bytes.b[2] = raw[304];
         __convert_float32_to_bytes.b[3] = raw[305];
         data.tracks[1].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[306];
         __convert_float32_to_bytes.b[1] = raw[307];
         __convert_float32_to_bytes.b[2] = raw[308];
         __convert_float32_to_bytes.b[3] = raw[309];
         data.tracks[1].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[310];
         __convert_float32_to_bytes.b[1] = raw[311];
         __convert_float32_to_bytes.b[2] = raw[312];
         __convert_float32_to_bytes.b[3] = raw[313];
         data.tracks[1].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[314];
         __convert_float32_to_bytes.b[1] = raw[315];
         __convert_float32_to_bytes.b[2] = raw[316];
         __convert_float32_to_bytes.b[3] = raw[317];
         data.tracks[1].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[318];
         __convert_float32_to_bytes.b[1] = raw[319];
         __convert_float32_to_bytes.b[2] = raw[320];
         __convert_float32_to_bytes.b[3] = raw[321];
         data.tracks[1].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[322];
         __convert_float32_to_bytes.b[1] = raw[323];
         __convert_float32_to_bytes.b[2] = raw[324];
         __convert_float32_to_bytes.b[3] = raw[325];
         data.tracks[1].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[326];
         __convert_float32_to_bytes.b[1] = raw[327];
         __convert_float32_to_bytes.b[2] = raw[328];
         __convert_float32_to_bytes.b[3] = raw[329];
         data.tracks[1].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[330];
         __convert_float32_to_bytes.b[1] = raw[331];
         __convert_float32_to_bytes.b[2] = raw[332];
         __convert_float32_to_bytes.b[3] = raw[333];
         data.tracks[1].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[334];
         __convert_float32_to_bytes.b[1] = raw[335];
         __convert_float32_to_bytes.b[2] = raw[336];
         __convert_float32_to_bytes.b[3] = raw[337];
         data.tracks[1].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[338];
         __convert_float32_to_bytes.b[1] = raw[339];
         __convert_float32_to_bytes.b[2] = raw[340];
         __convert_float32_to_bytes.b[3] = raw[341];
         data.tracks[1].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[342];
         __convert_float32_to_bytes.b[1] = raw[343];
         __convert_float32_to_bytes.b[2] = raw[344];
         __convert_float32_to_bytes.b[3] = raw[345];
         data.tracks[1].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[346];
         __convert_float32_to_bytes.b[1] = raw[347];
         __convert_float32_to_bytes.b[2] = raw[348];
         __convert_float32_to_bytes.b[3] = raw[349];
         data.tracks[1].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[1].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[350];
         u.b[1] = raw[351];
         u.b[2] = raw[352];
         u.b[3] = raw[353];
        switch(u.a) {
         case 0:
            data.tracks[1].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[1].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[1].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[1].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[1].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[1].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[1].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[1].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[1].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[1].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[354];
         __convert_uint16_to_bytes.b[1] = raw[355];
         data.tracks[1].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[356];
         __convert_uint16_to_bytes.b[1] = raw[357];
         data.tracks[1].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[358];
         __convert_uint16_to_bytes.b[1] = raw[359];
         data.tracks[1].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[360];
         __convert_float32_to_bytes.b[1] = raw[361];
         __convert_float32_to_bytes.b[2] = raw[362];
         __convert_float32_to_bytes.b[3] = raw[363];
         data.tracks[1].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[364];
         __convert_float32_to_bytes.b[1] = raw[365];
         __convert_float32_to_bytes.b[2] = raw[366];
         __convert_float32_to_bytes.b[3] = raw[367];
         data.tracks[1].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[368];
         __convert_float32_to_bytes.b[1] = raw[369];
         __convert_float32_to_bytes.b[2] = raw[370];
         __convert_float32_to_bytes.b[3] = raw[371];
         data.tracks[1].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[372];
         __convert_float32_to_bytes.b[1] = raw[373];
         __convert_float32_to_bytes.b[2] = raw[374];
         __convert_float32_to_bytes.b[3] = raw[375];
         data.tracks[1].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[376];
         __convert_float32_to_bytes.b[1] = raw[377];
         __convert_float32_to_bytes.b[2] = raw[378];
         __convert_float32_to_bytes.b[3] = raw[379];
         data.tracks[1].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[1].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[380];
         __convert_float32_to_bytes.b[1] = raw[381];
         __convert_float32_to_bytes.b[2] = raw[382];
         __convert_float32_to_bytes.b[3] = raw[383];
         data.tracks[1].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[384];
         __convert_uint32_to_bytes.b[1] = raw[385];
         __convert_uint32_to_bytes.b[2] = raw[386];
         __convert_uint32_to_bytes.b[3] = raw[387];
         data.tracks[2].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[388];
         data.tracks[2].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[389];
         data.tracks[2].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[390];
         data.tracks[2].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[391];
         data.tracks[2].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[392];
         data.tracks[2].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[393];
         data.tracks[2].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[394];
         data.tracks[2].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[2].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[395];
         data.tracks[2].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[2].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[396];
         __convert_uint64_to_bytes.b[1] = raw[397];
         __convert_uint64_to_bytes.b[2] = raw[398];
         __convert_uint64_to_bytes.b[3] = raw[399];
         __convert_uint64_to_bytes.b[4] = raw[400];
         __convert_uint64_to_bytes.b[5] = raw[401];
         __convert_uint64_to_bytes.b[6] = raw[402];
         __convert_uint64_to_bytes.b[7] = raw[403];
         data.tracks[2].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[2].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[404];
         u.b[1] = raw[405];
         u.b[2] = raw[406];
         u.b[3] = raw[407];
        switch(u.a) {
         case 0:
            data.tracks[2].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[2].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[2].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[2].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[2].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[2].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[2].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[2].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[2].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[2].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[2].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[2].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[2].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[2].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[2].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[408];
         u.b[1] = raw[409];
         u.b[2] = raw[410];
         u.b[3] = raw[411];
        switch(u.a) {
         case 0:
            data.tracks[2].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[2].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[2].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[2].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[2].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[2].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[2].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[2].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[2].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[2].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[2].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[2].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[2].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[2].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[2].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[2].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[2].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[2].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[2].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[2].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[412]) != 0x00u );
         data.tracks[2].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[412]) != 0x00u );
         data.tracks[2].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[412]) != 0x00u );
         data.tracks[2].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[412]) != 0x00u );
         data.tracks[2].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[412]) != 0x00u );
         data.tracks[2].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[412]) != 0x00u );
         data.tracks[2].source_data.LIDAR = ( ((0x01u << 6) & raw[412]) != 0x00u );
         data.tracks[2].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[412]) != 0x00u );
         data.tracks[2].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[413]) != 0x00u );
      } 
      { //READ data.tracks[2].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[414];
         __convert_float32_to_bytes.b[1] = raw[415];
         __convert_float32_to_bytes.b[2] = raw[416];
         __convert_float32_to_bytes.b[3] = raw[417];
         data.tracks[2].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[418];
         __convert_float32_to_bytes.b[1] = raw[419];
         __convert_float32_to_bytes.b[2] = raw[420];
         __convert_float32_to_bytes.b[3] = raw[421];
         data.tracks[2].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[422];
         data.tracks[2].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[423];
         __convert_float32_to_bytes.b[1] = raw[424];
         __convert_float32_to_bytes.b[2] = raw[425];
         __convert_float32_to_bytes.b[3] = raw[426];
         data.tracks[2].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[427];
         __convert_float32_to_bytes.b[1] = raw[428];
         __convert_float32_to_bytes.b[2] = raw[429];
         __convert_float32_to_bytes.b[3] = raw[430];
         data.tracks[2].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[431];
         __convert_int16_to_bytes.b[1] = raw[432];
         data.tracks[2].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[433];
         __convert_float32_to_bytes.b[1] = raw[434];
         __convert_float32_to_bytes.b[2] = raw[435];
         __convert_float32_to_bytes.b[3] = raw[436];
         data.tracks[2].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[2].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[437];
         u.b[1] = raw[438];
         u.b[2] = raw[439];
         u.b[3] = raw[440];
        switch(u.a) {
         case 0:
            data.tracks[2].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[2].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[2].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[2].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[2].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[2].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[441];
         __convert_uint32_to_bytes.b[1] = raw[442];
         __convert_uint32_to_bytes.b[2] = raw[443];
         __convert_uint32_to_bytes.b[3] = raw[444];
         data.tracks[2].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[445];
         __convert_float32_to_bytes.b[1] = raw[446];
         __convert_float32_to_bytes.b[2] = raw[447];
         __convert_float32_to_bytes.b[3] = raw[448];
         data.tracks[2].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[449];
         __convert_float32_to_bytes.b[1] = raw[450];
         __convert_float32_to_bytes.b[2] = raw[451];
         __convert_float32_to_bytes.b[3] = raw[452];
         data.tracks[2].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[453];
         __convert_float32_to_bytes.b[1] = raw[454];
         __convert_float32_to_bytes.b[2] = raw[455];
         __convert_float32_to_bytes.b[3] = raw[456];
         data.tracks[2].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[457];
         __convert_int16_to_bytes.b[1] = raw[458];
         data.tracks[2].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[2].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[459];
         __convert_float32_to_bytes.b[1] = raw[460];
         __convert_float32_to_bytes.b[2] = raw[461];
         __convert_float32_to_bytes.b[3] = raw[462];
         data.tracks[2].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[2].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[463]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[463]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[463]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[463]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[463]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[463]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[463]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[463]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[464]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[464]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[464]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[464]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[464]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[464]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[464]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[464]) != 0x00u );
         data.tracks[2].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[465]) != 0x00u );
      } 
      { //READ data.tracks[2].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[466];
         __convert_uint64_to_bytes.b[1] = raw[467];
         __convert_uint64_to_bytes.b[2] = raw[468];
         __convert_uint64_to_bytes.b[3] = raw[469];
         __convert_uint64_to_bytes.b[4] = raw[470];
         __convert_uint64_to_bytes.b[5] = raw[471];
         __convert_uint64_to_bytes.b[6] = raw[472];
         __convert_uint64_to_bytes.b[7] = raw[473];
         data.tracks[2].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[474];
         __convert_float64_to_bytes.b[1] = raw[475];
         __convert_float64_to_bytes.b[2] = raw[476];
         __convert_float64_to_bytes.b[3] = raw[477];
         __convert_float64_to_bytes.b[4] = raw[478];
         __convert_float64_to_bytes.b[5] = raw[479];
         __convert_float64_to_bytes.b[6] = raw[480];
         __convert_float64_to_bytes.b[7] = raw[481];
         data.tracks[2].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[482];
         __convert_float64_to_bytes.b[1] = raw[483];
         __convert_float64_to_bytes.b[2] = raw[484];
         __convert_float64_to_bytes.b[3] = raw[485];
         __convert_float64_to_bytes.b[4] = raw[486];
         __convert_float64_to_bytes.b[5] = raw[487];
         __convert_float64_to_bytes.b[6] = raw[488];
         __convert_float64_to_bytes.b[7] = raw[489];
         data.tracks[2].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[490];
         __convert_float32_to_bytes.b[1] = raw[491];
         __convert_float32_to_bytes.b[2] = raw[492];
         __convert_float32_to_bytes.b[3] = raw[493];
         data.tracks[2].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[494];
         __convert_float32_to_bytes.b[1] = raw[495];
         __convert_float32_to_bytes.b[2] = raw[496];
         __convert_float32_to_bytes.b[3] = raw[497];
         data.tracks[2].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[498];
         __convert_float32_to_bytes.b[1] = raw[499];
         __convert_float32_to_bytes.b[2] = raw[500];
         __convert_float32_to_bytes.b[3] = raw[501];
         data.tracks[2].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[502];
         __convert_float32_to_bytes.b[1] = raw[503];
         __convert_float32_to_bytes.b[2] = raw[504];
         __convert_float32_to_bytes.b[3] = raw[505];
         data.tracks[2].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[506];
         __convert_float32_to_bytes.b[1] = raw[507];
         __convert_float32_to_bytes.b[2] = raw[508];
         __convert_float32_to_bytes.b[3] = raw[509];
         data.tracks[2].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[510];
         __convert_float32_to_bytes.b[1] = raw[511];
         __convert_float32_to_bytes.b[2] = raw[512];
         __convert_float32_to_bytes.b[3] = raw[513];
         data.tracks[2].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[514];
         __convert_float32_to_bytes.b[1] = raw[515];
         __convert_float32_to_bytes.b[2] = raw[516];
         __convert_float32_to_bytes.b[3] = raw[517];
         data.tracks[2].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[518];
         __convert_float32_to_bytes.b[1] = raw[519];
         __convert_float32_to_bytes.b[2] = raw[520];
         __convert_float32_to_bytes.b[3] = raw[521];
         data.tracks[2].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[522];
         __convert_float32_to_bytes.b[1] = raw[523];
         __convert_float32_to_bytes.b[2] = raw[524];
         __convert_float32_to_bytes.b[3] = raw[525];
         data.tracks[2].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[526];
         __convert_float32_to_bytes.b[1] = raw[527];
         __convert_float32_to_bytes.b[2] = raw[528];
         __convert_float32_to_bytes.b[3] = raw[529];
         data.tracks[2].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[530];
         __convert_float32_to_bytes.b[1] = raw[531];
         __convert_float32_to_bytes.b[2] = raw[532];
         __convert_float32_to_bytes.b[3] = raw[533];
         data.tracks[2].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[534];
         __convert_float32_to_bytes.b[1] = raw[535];
         __convert_float32_to_bytes.b[2] = raw[536];
         __convert_float32_to_bytes.b[3] = raw[537];
         data.tracks[2].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[538];
         __convert_float32_to_bytes.b[1] = raw[539];
         __convert_float32_to_bytes.b[2] = raw[540];
         __convert_float32_to_bytes.b[3] = raw[541];
         data.tracks[2].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[2].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[542];
         u.b[1] = raw[543];
         u.b[2] = raw[544];
         u.b[3] = raw[545];
        switch(u.a) {
         case 0:
            data.tracks[2].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[2].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[2].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[2].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[2].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[2].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[2].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[2].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[2].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[2].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[546];
         __convert_uint16_to_bytes.b[1] = raw[547];
         data.tracks[2].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[548];
         __convert_uint16_to_bytes.b[1] = raw[549];
         data.tracks[2].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[550];
         __convert_uint16_to_bytes.b[1] = raw[551];
         data.tracks[2].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[552];
         __convert_float32_to_bytes.b[1] = raw[553];
         __convert_float32_to_bytes.b[2] = raw[554];
         __convert_float32_to_bytes.b[3] = raw[555];
         data.tracks[2].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[556];
         __convert_float32_to_bytes.b[1] = raw[557];
         __convert_float32_to_bytes.b[2] = raw[558];
         __convert_float32_to_bytes.b[3] = raw[559];
         data.tracks[2].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[560];
         __convert_float32_to_bytes.b[1] = raw[561];
         __convert_float32_to_bytes.b[2] = raw[562];
         __convert_float32_to_bytes.b[3] = raw[563];
         data.tracks[2].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[564];
         __convert_float32_to_bytes.b[1] = raw[565];
         __convert_float32_to_bytes.b[2] = raw[566];
         __convert_float32_to_bytes.b[3] = raw[567];
         data.tracks[2].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[568];
         __convert_float32_to_bytes.b[1] = raw[569];
         __convert_float32_to_bytes.b[2] = raw[570];
         __convert_float32_to_bytes.b[3] = raw[571];
         data.tracks[2].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[2].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[572];
         __convert_float32_to_bytes.b[1] = raw[573];
         __convert_float32_to_bytes.b[2] = raw[574];
         __convert_float32_to_bytes.b[3] = raw[575];
         data.tracks[2].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[576];
         __convert_uint32_to_bytes.b[1] = raw[577];
         __convert_uint32_to_bytes.b[2] = raw[578];
         __convert_uint32_to_bytes.b[3] = raw[579];
         data.tracks[3].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[580];
         data.tracks[3].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[581];
         data.tracks[3].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[582];
         data.tracks[3].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[583];
         data.tracks[3].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[584];
         data.tracks[3].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[585];
         data.tracks[3].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[586];
         data.tracks[3].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[3].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[587];
         data.tracks[3].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[3].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[588];
         __convert_uint64_to_bytes.b[1] = raw[589];
         __convert_uint64_to_bytes.b[2] = raw[590];
         __convert_uint64_to_bytes.b[3] = raw[591];
         __convert_uint64_to_bytes.b[4] = raw[592];
         __convert_uint64_to_bytes.b[5] = raw[593];
         __convert_uint64_to_bytes.b[6] = raw[594];
         __convert_uint64_to_bytes.b[7] = raw[595];
         data.tracks[3].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[3].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[596];
         u.b[1] = raw[597];
         u.b[2] = raw[598];
         u.b[3] = raw[599];
        switch(u.a) {
         case 0:
            data.tracks[3].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[3].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[3].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[3].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[3].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[3].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[3].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[3].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[3].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[3].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[3].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[3].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[3].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[3].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[3].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[600];
         u.b[1] = raw[601];
         u.b[2] = raw[602];
         u.b[3] = raw[603];
        switch(u.a) {
         case 0:
            data.tracks[3].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[3].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[3].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[3].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[3].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[3].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[3].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[3].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[3].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[3].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[3].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[3].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[3].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[3].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[3].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[3].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[3].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[3].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[3].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[3].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[604]) != 0x00u );
         data.tracks[3].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[604]) != 0x00u );
         data.tracks[3].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[604]) != 0x00u );
         data.tracks[3].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[604]) != 0x00u );
         data.tracks[3].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[604]) != 0x00u );
         data.tracks[3].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[604]) != 0x00u );
         data.tracks[3].source_data.LIDAR = ( ((0x01u << 6) & raw[604]) != 0x00u );
         data.tracks[3].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[604]) != 0x00u );
         data.tracks[3].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[605]) != 0x00u );
      } 
      { //READ data.tracks[3].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[606];
         __convert_float32_to_bytes.b[1] = raw[607];
         __convert_float32_to_bytes.b[2] = raw[608];
         __convert_float32_to_bytes.b[3] = raw[609];
         data.tracks[3].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[610];
         __convert_float32_to_bytes.b[1] = raw[611];
         __convert_float32_to_bytes.b[2] = raw[612];
         __convert_float32_to_bytes.b[3] = raw[613];
         data.tracks[3].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[614];
         data.tracks[3].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[615];
         __convert_float32_to_bytes.b[1] = raw[616];
         __convert_float32_to_bytes.b[2] = raw[617];
         __convert_float32_to_bytes.b[3] = raw[618];
         data.tracks[3].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[619];
         __convert_float32_to_bytes.b[1] = raw[620];
         __convert_float32_to_bytes.b[2] = raw[621];
         __convert_float32_to_bytes.b[3] = raw[622];
         data.tracks[3].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[623];
         __convert_int16_to_bytes.b[1] = raw[624];
         data.tracks[3].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[625];
         __convert_float32_to_bytes.b[1] = raw[626];
         __convert_float32_to_bytes.b[2] = raw[627];
         __convert_float32_to_bytes.b[3] = raw[628];
         data.tracks[3].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[3].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[629];
         u.b[1] = raw[630];
         u.b[2] = raw[631];
         u.b[3] = raw[632];
        switch(u.a) {
         case 0:
            data.tracks[3].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[3].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[3].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[3].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[3].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[3].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[633];
         __convert_uint32_to_bytes.b[1] = raw[634];
         __convert_uint32_to_bytes.b[2] = raw[635];
         __convert_uint32_to_bytes.b[3] = raw[636];
         data.tracks[3].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[637];
         __convert_float32_to_bytes.b[1] = raw[638];
         __convert_float32_to_bytes.b[2] = raw[639];
         __convert_float32_to_bytes.b[3] = raw[640];
         data.tracks[3].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[641];
         __convert_float32_to_bytes.b[1] = raw[642];
         __convert_float32_to_bytes.b[2] = raw[643];
         __convert_float32_to_bytes.b[3] = raw[644];
         data.tracks[3].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[645];
         __convert_float32_to_bytes.b[1] = raw[646];
         __convert_float32_to_bytes.b[2] = raw[647];
         __convert_float32_to_bytes.b[3] = raw[648];
         data.tracks[3].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[649];
         __convert_int16_to_bytes.b[1] = raw[650];
         data.tracks[3].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[3].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[651];
         __convert_float32_to_bytes.b[1] = raw[652];
         __convert_float32_to_bytes.b[2] = raw[653];
         __convert_float32_to_bytes.b[3] = raw[654];
         data.tracks[3].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[3].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[655]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[655]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[655]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[655]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[655]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[655]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[655]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[655]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[656]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[656]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[656]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[656]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[656]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[656]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[656]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[656]) != 0x00u );
         data.tracks[3].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[657]) != 0x00u );
      } 
      { //READ data.tracks[3].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[658];
         __convert_uint64_to_bytes.b[1] = raw[659];
         __convert_uint64_to_bytes.b[2] = raw[660];
         __convert_uint64_to_bytes.b[3] = raw[661];
         __convert_uint64_to_bytes.b[4] = raw[662];
         __convert_uint64_to_bytes.b[5] = raw[663];
         __convert_uint64_to_bytes.b[6] = raw[664];
         __convert_uint64_to_bytes.b[7] = raw[665];
         data.tracks[3].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[666];
         __convert_float64_to_bytes.b[1] = raw[667];
         __convert_float64_to_bytes.b[2] = raw[668];
         __convert_float64_to_bytes.b[3] = raw[669];
         __convert_float64_to_bytes.b[4] = raw[670];
         __convert_float64_to_bytes.b[5] = raw[671];
         __convert_float64_to_bytes.b[6] = raw[672];
         __convert_float64_to_bytes.b[7] = raw[673];
         data.tracks[3].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[674];
         __convert_float64_to_bytes.b[1] = raw[675];
         __convert_float64_to_bytes.b[2] = raw[676];
         __convert_float64_to_bytes.b[3] = raw[677];
         __convert_float64_to_bytes.b[4] = raw[678];
         __convert_float64_to_bytes.b[5] = raw[679];
         __convert_float64_to_bytes.b[6] = raw[680];
         __convert_float64_to_bytes.b[7] = raw[681];
         data.tracks[3].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[682];
         __convert_float32_to_bytes.b[1] = raw[683];
         __convert_float32_to_bytes.b[2] = raw[684];
         __convert_float32_to_bytes.b[3] = raw[685];
         data.tracks[3].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[686];
         __convert_float32_to_bytes.b[1] = raw[687];
         __convert_float32_to_bytes.b[2] = raw[688];
         __convert_float32_to_bytes.b[3] = raw[689];
         data.tracks[3].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[690];
         __convert_float32_to_bytes.b[1] = raw[691];
         __convert_float32_to_bytes.b[2] = raw[692];
         __convert_float32_to_bytes.b[3] = raw[693];
         data.tracks[3].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[694];
         __convert_float32_to_bytes.b[1] = raw[695];
         __convert_float32_to_bytes.b[2] = raw[696];
         __convert_float32_to_bytes.b[3] = raw[697];
         data.tracks[3].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[698];
         __convert_float32_to_bytes.b[1] = raw[699];
         __convert_float32_to_bytes.b[2] = raw[700];
         __convert_float32_to_bytes.b[3] = raw[701];
         data.tracks[3].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[702];
         __convert_float32_to_bytes.b[1] = raw[703];
         __convert_float32_to_bytes.b[2] = raw[704];
         __convert_float32_to_bytes.b[3] = raw[705];
         data.tracks[3].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[706];
         __convert_float32_to_bytes.b[1] = raw[707];
         __convert_float32_to_bytes.b[2] = raw[708];
         __convert_float32_to_bytes.b[3] = raw[709];
         data.tracks[3].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[710];
         __convert_float32_to_bytes.b[1] = raw[711];
         __convert_float32_to_bytes.b[2] = raw[712];
         __convert_float32_to_bytes.b[3] = raw[713];
         data.tracks[3].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[714];
         __convert_float32_to_bytes.b[1] = raw[715];
         __convert_float32_to_bytes.b[2] = raw[716];
         __convert_float32_to_bytes.b[3] = raw[717];
         data.tracks[3].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[718];
         __convert_float32_to_bytes.b[1] = raw[719];
         __convert_float32_to_bytes.b[2] = raw[720];
         __convert_float32_to_bytes.b[3] = raw[721];
         data.tracks[3].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[722];
         __convert_float32_to_bytes.b[1] = raw[723];
         __convert_float32_to_bytes.b[2] = raw[724];
         __convert_float32_to_bytes.b[3] = raw[725];
         data.tracks[3].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[726];
         __convert_float32_to_bytes.b[1] = raw[727];
         __convert_float32_to_bytes.b[2] = raw[728];
         __convert_float32_to_bytes.b[3] = raw[729];
         data.tracks[3].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[730];
         __convert_float32_to_bytes.b[1] = raw[731];
         __convert_float32_to_bytes.b[2] = raw[732];
         __convert_float32_to_bytes.b[3] = raw[733];
         data.tracks[3].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[3].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[734];
         u.b[1] = raw[735];
         u.b[2] = raw[736];
         u.b[3] = raw[737];
        switch(u.a) {
         case 0:
            data.tracks[3].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[3].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[3].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[3].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[3].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[3].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[3].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[3].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[3].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[3].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[738];
         __convert_uint16_to_bytes.b[1] = raw[739];
         data.tracks[3].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[740];
         __convert_uint16_to_bytes.b[1] = raw[741];
         data.tracks[3].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[742];
         __convert_uint16_to_bytes.b[1] = raw[743];
         data.tracks[3].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[744];
         __convert_float32_to_bytes.b[1] = raw[745];
         __convert_float32_to_bytes.b[2] = raw[746];
         __convert_float32_to_bytes.b[3] = raw[747];
         data.tracks[3].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[748];
         __convert_float32_to_bytes.b[1] = raw[749];
         __convert_float32_to_bytes.b[2] = raw[750];
         __convert_float32_to_bytes.b[3] = raw[751];
         data.tracks[3].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[752];
         __convert_float32_to_bytes.b[1] = raw[753];
         __convert_float32_to_bytes.b[2] = raw[754];
         __convert_float32_to_bytes.b[3] = raw[755];
         data.tracks[3].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[756];
         __convert_float32_to_bytes.b[1] = raw[757];
         __convert_float32_to_bytes.b[2] = raw[758];
         __convert_float32_to_bytes.b[3] = raw[759];
         data.tracks[3].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[760];
         __convert_float32_to_bytes.b[1] = raw[761];
         __convert_float32_to_bytes.b[2] = raw[762];
         __convert_float32_to_bytes.b[3] = raw[763];
         data.tracks[3].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[3].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[764];
         __convert_float32_to_bytes.b[1] = raw[765];
         __convert_float32_to_bytes.b[2] = raw[766];
         __convert_float32_to_bytes.b[3] = raw[767];
         data.tracks[3].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[768];
         __convert_uint32_to_bytes.b[1] = raw[769];
         __convert_uint32_to_bytes.b[2] = raw[770];
         __convert_uint32_to_bytes.b[3] = raw[771];
         data.tracks[4].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[772];
         data.tracks[4].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[773];
         data.tracks[4].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[774];
         data.tracks[4].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[775];
         data.tracks[4].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[776];
         data.tracks[4].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[777];
         data.tracks[4].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[778];
         data.tracks[4].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[4].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[779];
         data.tracks[4].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[4].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[780];
         __convert_uint64_to_bytes.b[1] = raw[781];
         __convert_uint64_to_bytes.b[2] = raw[782];
         __convert_uint64_to_bytes.b[3] = raw[783];
         __convert_uint64_to_bytes.b[4] = raw[784];
         __convert_uint64_to_bytes.b[5] = raw[785];
         __convert_uint64_to_bytes.b[6] = raw[786];
         __convert_uint64_to_bytes.b[7] = raw[787];
         data.tracks[4].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[4].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[788];
         u.b[1] = raw[789];
         u.b[2] = raw[790];
         u.b[3] = raw[791];
        switch(u.a) {
         case 0:
            data.tracks[4].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[4].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[4].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[4].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[4].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[4].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[4].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[4].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[4].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[4].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[4].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[4].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[4].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[4].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[4].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[792];
         u.b[1] = raw[793];
         u.b[2] = raw[794];
         u.b[3] = raw[795];
        switch(u.a) {
         case 0:
            data.tracks[4].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[4].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[4].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[4].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[4].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[4].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[4].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[4].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[4].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[4].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[4].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[4].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[4].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[4].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[4].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[4].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[4].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[4].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[4].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[4].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[796]) != 0x00u );
         data.tracks[4].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[796]) != 0x00u );
         data.tracks[4].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[796]) != 0x00u );
         data.tracks[4].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[796]) != 0x00u );
         data.tracks[4].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[796]) != 0x00u );
         data.tracks[4].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[796]) != 0x00u );
         data.tracks[4].source_data.LIDAR = ( ((0x01u << 6) & raw[796]) != 0x00u );
         data.tracks[4].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[796]) != 0x00u );
         data.tracks[4].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[797]) != 0x00u );
      } 
      { //READ data.tracks[4].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[798];
         __convert_float32_to_bytes.b[1] = raw[799];
         __convert_float32_to_bytes.b[2] = raw[800];
         __convert_float32_to_bytes.b[3] = raw[801];
         data.tracks[4].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[802];
         __convert_float32_to_bytes.b[1] = raw[803];
         __convert_float32_to_bytes.b[2] = raw[804];
         __convert_float32_to_bytes.b[3] = raw[805];
         data.tracks[4].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[806];
         data.tracks[4].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[807];
         __convert_float32_to_bytes.b[1] = raw[808];
         __convert_float32_to_bytes.b[2] = raw[809];
         __convert_float32_to_bytes.b[3] = raw[810];
         data.tracks[4].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[811];
         __convert_float32_to_bytes.b[1] = raw[812];
         __convert_float32_to_bytes.b[2] = raw[813];
         __convert_float32_to_bytes.b[3] = raw[814];
         data.tracks[4].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[815];
         __convert_int16_to_bytes.b[1] = raw[816];
         data.tracks[4].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[817];
         __convert_float32_to_bytes.b[1] = raw[818];
         __convert_float32_to_bytes.b[2] = raw[819];
         __convert_float32_to_bytes.b[3] = raw[820];
         data.tracks[4].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[4].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[821];
         u.b[1] = raw[822];
         u.b[2] = raw[823];
         u.b[3] = raw[824];
        switch(u.a) {
         case 0:
            data.tracks[4].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[4].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[4].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[4].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[4].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[4].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[825];
         __convert_uint32_to_bytes.b[1] = raw[826];
         __convert_uint32_to_bytes.b[2] = raw[827];
         __convert_uint32_to_bytes.b[3] = raw[828];
         data.tracks[4].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[829];
         __convert_float32_to_bytes.b[1] = raw[830];
         __convert_float32_to_bytes.b[2] = raw[831];
         __convert_float32_to_bytes.b[3] = raw[832];
         data.tracks[4].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[833];
         __convert_float32_to_bytes.b[1] = raw[834];
         __convert_float32_to_bytes.b[2] = raw[835];
         __convert_float32_to_bytes.b[3] = raw[836];
         data.tracks[4].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[837];
         __convert_float32_to_bytes.b[1] = raw[838];
         __convert_float32_to_bytes.b[2] = raw[839];
         __convert_float32_to_bytes.b[3] = raw[840];
         data.tracks[4].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[841];
         __convert_int16_to_bytes.b[1] = raw[842];
         data.tracks[4].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[4].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[843];
         __convert_float32_to_bytes.b[1] = raw[844];
         __convert_float32_to_bytes.b[2] = raw[845];
         __convert_float32_to_bytes.b[3] = raw[846];
         data.tracks[4].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[4].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[847]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[847]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[847]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[847]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[847]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[847]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[847]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[847]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[848]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[848]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[848]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[848]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[848]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[848]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[848]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[848]) != 0x00u );
         data.tracks[4].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[849]) != 0x00u );
      } 
      { //READ data.tracks[4].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[850];
         __convert_uint64_to_bytes.b[1] = raw[851];
         __convert_uint64_to_bytes.b[2] = raw[852];
         __convert_uint64_to_bytes.b[3] = raw[853];
         __convert_uint64_to_bytes.b[4] = raw[854];
         __convert_uint64_to_bytes.b[5] = raw[855];
         __convert_uint64_to_bytes.b[6] = raw[856];
         __convert_uint64_to_bytes.b[7] = raw[857];
         data.tracks[4].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[858];
         __convert_float64_to_bytes.b[1] = raw[859];
         __convert_float64_to_bytes.b[2] = raw[860];
         __convert_float64_to_bytes.b[3] = raw[861];
         __convert_float64_to_bytes.b[4] = raw[862];
         __convert_float64_to_bytes.b[5] = raw[863];
         __convert_float64_to_bytes.b[6] = raw[864];
         __convert_float64_to_bytes.b[7] = raw[865];
         data.tracks[4].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[866];
         __convert_float64_to_bytes.b[1] = raw[867];
         __convert_float64_to_bytes.b[2] = raw[868];
         __convert_float64_to_bytes.b[3] = raw[869];
         __convert_float64_to_bytes.b[4] = raw[870];
         __convert_float64_to_bytes.b[5] = raw[871];
         __convert_float64_to_bytes.b[6] = raw[872];
         __convert_float64_to_bytes.b[7] = raw[873];
         data.tracks[4].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[874];
         __convert_float32_to_bytes.b[1] = raw[875];
         __convert_float32_to_bytes.b[2] = raw[876];
         __convert_float32_to_bytes.b[3] = raw[877];
         data.tracks[4].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[878];
         __convert_float32_to_bytes.b[1] = raw[879];
         __convert_float32_to_bytes.b[2] = raw[880];
         __convert_float32_to_bytes.b[3] = raw[881];
         data.tracks[4].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[882];
         __convert_float32_to_bytes.b[1] = raw[883];
         __convert_float32_to_bytes.b[2] = raw[884];
         __convert_float32_to_bytes.b[3] = raw[885];
         data.tracks[4].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[886];
         __convert_float32_to_bytes.b[1] = raw[887];
         __convert_float32_to_bytes.b[2] = raw[888];
         __convert_float32_to_bytes.b[3] = raw[889];
         data.tracks[4].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[890];
         __convert_float32_to_bytes.b[1] = raw[891];
         __convert_float32_to_bytes.b[2] = raw[892];
         __convert_float32_to_bytes.b[3] = raw[893];
         data.tracks[4].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[894];
         __convert_float32_to_bytes.b[1] = raw[895];
         __convert_float32_to_bytes.b[2] = raw[896];
         __convert_float32_to_bytes.b[3] = raw[897];
         data.tracks[4].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[898];
         __convert_float32_to_bytes.b[1] = raw[899];
         __convert_float32_to_bytes.b[2] = raw[900];
         __convert_float32_to_bytes.b[3] = raw[901];
         data.tracks[4].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[902];
         __convert_float32_to_bytes.b[1] = raw[903];
         __convert_float32_to_bytes.b[2] = raw[904];
         __convert_float32_to_bytes.b[3] = raw[905];
         data.tracks[4].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[906];
         __convert_float32_to_bytes.b[1] = raw[907];
         __convert_float32_to_bytes.b[2] = raw[908];
         __convert_float32_to_bytes.b[3] = raw[909];
         data.tracks[4].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[910];
         __convert_float32_to_bytes.b[1] = raw[911];
         __convert_float32_to_bytes.b[2] = raw[912];
         __convert_float32_to_bytes.b[3] = raw[913];
         data.tracks[4].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[914];
         __convert_float32_to_bytes.b[1] = raw[915];
         __convert_float32_to_bytes.b[2] = raw[916];
         __convert_float32_to_bytes.b[3] = raw[917];
         data.tracks[4].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[918];
         __convert_float32_to_bytes.b[1] = raw[919];
         __convert_float32_to_bytes.b[2] = raw[920];
         __convert_float32_to_bytes.b[3] = raw[921];
         data.tracks[4].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[922];
         __convert_float32_to_bytes.b[1] = raw[923];
         __convert_float32_to_bytes.b[2] = raw[924];
         __convert_float32_to_bytes.b[3] = raw[925];
         data.tracks[4].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[4].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[926];
         u.b[1] = raw[927];
         u.b[2] = raw[928];
         u.b[3] = raw[929];
        switch(u.a) {
         case 0:
            data.tracks[4].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[4].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[4].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[4].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[4].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[4].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[4].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[4].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[4].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[4].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[930];
         __convert_uint16_to_bytes.b[1] = raw[931];
         data.tracks[4].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[932];
         __convert_uint16_to_bytes.b[1] = raw[933];
         data.tracks[4].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[934];
         __convert_uint16_to_bytes.b[1] = raw[935];
         data.tracks[4].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[936];
         __convert_float32_to_bytes.b[1] = raw[937];
         __convert_float32_to_bytes.b[2] = raw[938];
         __convert_float32_to_bytes.b[3] = raw[939];
         data.tracks[4].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[940];
         __convert_float32_to_bytes.b[1] = raw[941];
         __convert_float32_to_bytes.b[2] = raw[942];
         __convert_float32_to_bytes.b[3] = raw[943];
         data.tracks[4].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[944];
         __convert_float32_to_bytes.b[1] = raw[945];
         __convert_float32_to_bytes.b[2] = raw[946];
         __convert_float32_to_bytes.b[3] = raw[947];
         data.tracks[4].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[948];
         __convert_float32_to_bytes.b[1] = raw[949];
         __convert_float32_to_bytes.b[2] = raw[950];
         __convert_float32_to_bytes.b[3] = raw[951];
         data.tracks[4].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[952];
         __convert_float32_to_bytes.b[1] = raw[953];
         __convert_float32_to_bytes.b[2] = raw[954];
         __convert_float32_to_bytes.b[3] = raw[955];
         data.tracks[4].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[4].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[956];
         __convert_float32_to_bytes.b[1] = raw[957];
         __convert_float32_to_bytes.b[2] = raw[958];
         __convert_float32_to_bytes.b[3] = raw[959];
         data.tracks[4].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[960];
         __convert_uint32_to_bytes.b[1] = raw[961];
         __convert_uint32_to_bytes.b[2] = raw[962];
         __convert_uint32_to_bytes.b[3] = raw[963];
         data.tracks[5].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[964];
         data.tracks[5].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[965];
         data.tracks[5].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[966];
         data.tracks[5].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[967];
         data.tracks[5].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[968];
         data.tracks[5].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[969];
         data.tracks[5].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[970];
         data.tracks[5].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[5].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[971];
         data.tracks[5].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[5].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[972];
         __convert_uint64_to_bytes.b[1] = raw[973];
         __convert_uint64_to_bytes.b[2] = raw[974];
         __convert_uint64_to_bytes.b[3] = raw[975];
         __convert_uint64_to_bytes.b[4] = raw[976];
         __convert_uint64_to_bytes.b[5] = raw[977];
         __convert_uint64_to_bytes.b[6] = raw[978];
         __convert_uint64_to_bytes.b[7] = raw[979];
         data.tracks[5].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[5].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[980];
         u.b[1] = raw[981];
         u.b[2] = raw[982];
         u.b[3] = raw[983];
        switch(u.a) {
         case 0:
            data.tracks[5].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[5].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[5].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[5].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[5].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[5].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[5].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[5].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[5].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[5].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[5].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[5].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[5].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[5].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[5].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[984];
         u.b[1] = raw[985];
         u.b[2] = raw[986];
         u.b[3] = raw[987];
        switch(u.a) {
         case 0:
            data.tracks[5].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[5].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[5].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[5].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[5].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[5].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[5].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[5].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[5].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[5].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[5].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[5].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[5].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[5].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[5].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[5].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[5].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[5].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[5].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[5].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[988]) != 0x00u );
         data.tracks[5].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[988]) != 0x00u );
         data.tracks[5].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[988]) != 0x00u );
         data.tracks[5].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[988]) != 0x00u );
         data.tracks[5].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[988]) != 0x00u );
         data.tracks[5].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[988]) != 0x00u );
         data.tracks[5].source_data.LIDAR = ( ((0x01u << 6) & raw[988]) != 0x00u );
         data.tracks[5].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[988]) != 0x00u );
         data.tracks[5].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[989]) != 0x00u );
      } 
      { //READ data.tracks[5].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[990];
         __convert_float32_to_bytes.b[1] = raw[991];
         __convert_float32_to_bytes.b[2] = raw[992];
         __convert_float32_to_bytes.b[3] = raw[993];
         data.tracks[5].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[994];
         __convert_float32_to_bytes.b[1] = raw[995];
         __convert_float32_to_bytes.b[2] = raw[996];
         __convert_float32_to_bytes.b[3] = raw[997];
         data.tracks[5].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[998];
         data.tracks[5].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[999];
         __convert_float32_to_bytes.b[1] = raw[1000];
         __convert_float32_to_bytes.b[2] = raw[1001];
         __convert_float32_to_bytes.b[3] = raw[1002];
         data.tracks[5].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1003];
         __convert_float32_to_bytes.b[1] = raw[1004];
         __convert_float32_to_bytes.b[2] = raw[1005];
         __convert_float32_to_bytes.b[3] = raw[1006];
         data.tracks[5].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1007];
         __convert_int16_to_bytes.b[1] = raw[1008];
         data.tracks[5].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1009];
         __convert_float32_to_bytes.b[1] = raw[1010];
         __convert_float32_to_bytes.b[2] = raw[1011];
         __convert_float32_to_bytes.b[3] = raw[1012];
         data.tracks[5].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[5].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1013];
         u.b[1] = raw[1014];
         u.b[2] = raw[1015];
         u.b[3] = raw[1016];
        switch(u.a) {
         case 0:
            data.tracks[5].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[5].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[5].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[5].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[5].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[5].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[1017];
         __convert_uint32_to_bytes.b[1] = raw[1018];
         __convert_uint32_to_bytes.b[2] = raw[1019];
         __convert_uint32_to_bytes.b[3] = raw[1020];
         data.tracks[5].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1021];
         __convert_float32_to_bytes.b[1] = raw[1022];
         __convert_float32_to_bytes.b[2] = raw[1023];
         __convert_float32_to_bytes.b[3] = raw[1024];
         data.tracks[5].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1025];
         __convert_float32_to_bytes.b[1] = raw[1026];
         __convert_float32_to_bytes.b[2] = raw[1027];
         __convert_float32_to_bytes.b[3] = raw[1028];
         data.tracks[5].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1029];
         __convert_float32_to_bytes.b[1] = raw[1030];
         __convert_float32_to_bytes.b[2] = raw[1031];
         __convert_float32_to_bytes.b[3] = raw[1032];
         data.tracks[5].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1033];
         __convert_int16_to_bytes.b[1] = raw[1034];
         data.tracks[5].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[5].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1035];
         __convert_float32_to_bytes.b[1] = raw[1036];
         __convert_float32_to_bytes.b[2] = raw[1037];
         __convert_float32_to_bytes.b[3] = raw[1038];
         data.tracks[5].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[5].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[1039]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[1039]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[1039]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[1039]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[1039]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[1039]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[1039]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[1039]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[1040]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[1040]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[1040]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[1040]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[1040]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[1040]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[1040]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[1040]) != 0x00u );
         data.tracks[5].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[1041]) != 0x00u );
      } 
      { //READ data.tracks[5].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[1042];
         __convert_uint64_to_bytes.b[1] = raw[1043];
         __convert_uint64_to_bytes.b[2] = raw[1044];
         __convert_uint64_to_bytes.b[3] = raw[1045];
         __convert_uint64_to_bytes.b[4] = raw[1046];
         __convert_uint64_to_bytes.b[5] = raw[1047];
         __convert_uint64_to_bytes.b[6] = raw[1048];
         __convert_uint64_to_bytes.b[7] = raw[1049];
         data.tracks[5].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1050];
         __convert_float64_to_bytes.b[1] = raw[1051];
         __convert_float64_to_bytes.b[2] = raw[1052];
         __convert_float64_to_bytes.b[3] = raw[1053];
         __convert_float64_to_bytes.b[4] = raw[1054];
         __convert_float64_to_bytes.b[5] = raw[1055];
         __convert_float64_to_bytes.b[6] = raw[1056];
         __convert_float64_to_bytes.b[7] = raw[1057];
         data.tracks[5].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1058];
         __convert_float64_to_bytes.b[1] = raw[1059];
         __convert_float64_to_bytes.b[2] = raw[1060];
         __convert_float64_to_bytes.b[3] = raw[1061];
         __convert_float64_to_bytes.b[4] = raw[1062];
         __convert_float64_to_bytes.b[5] = raw[1063];
         __convert_float64_to_bytes.b[6] = raw[1064];
         __convert_float64_to_bytes.b[7] = raw[1065];
         data.tracks[5].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1066];
         __convert_float32_to_bytes.b[1] = raw[1067];
         __convert_float32_to_bytes.b[2] = raw[1068];
         __convert_float32_to_bytes.b[3] = raw[1069];
         data.tracks[5].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1070];
         __convert_float32_to_bytes.b[1] = raw[1071];
         __convert_float32_to_bytes.b[2] = raw[1072];
         __convert_float32_to_bytes.b[3] = raw[1073];
         data.tracks[5].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1074];
         __convert_float32_to_bytes.b[1] = raw[1075];
         __convert_float32_to_bytes.b[2] = raw[1076];
         __convert_float32_to_bytes.b[3] = raw[1077];
         data.tracks[5].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1078];
         __convert_float32_to_bytes.b[1] = raw[1079];
         __convert_float32_to_bytes.b[2] = raw[1080];
         __convert_float32_to_bytes.b[3] = raw[1081];
         data.tracks[5].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1082];
         __convert_float32_to_bytes.b[1] = raw[1083];
         __convert_float32_to_bytes.b[2] = raw[1084];
         __convert_float32_to_bytes.b[3] = raw[1085];
         data.tracks[5].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1086];
         __convert_float32_to_bytes.b[1] = raw[1087];
         __convert_float32_to_bytes.b[2] = raw[1088];
         __convert_float32_to_bytes.b[3] = raw[1089];
         data.tracks[5].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1090];
         __convert_float32_to_bytes.b[1] = raw[1091];
         __convert_float32_to_bytes.b[2] = raw[1092];
         __convert_float32_to_bytes.b[3] = raw[1093];
         data.tracks[5].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1094];
         __convert_float32_to_bytes.b[1] = raw[1095];
         __convert_float32_to_bytes.b[2] = raw[1096];
         __convert_float32_to_bytes.b[3] = raw[1097];
         data.tracks[5].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1098];
         __convert_float32_to_bytes.b[1] = raw[1099];
         __convert_float32_to_bytes.b[2] = raw[1100];
         __convert_float32_to_bytes.b[3] = raw[1101];
         data.tracks[5].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1102];
         __convert_float32_to_bytes.b[1] = raw[1103];
         __convert_float32_to_bytes.b[2] = raw[1104];
         __convert_float32_to_bytes.b[3] = raw[1105];
         data.tracks[5].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1106];
         __convert_float32_to_bytes.b[1] = raw[1107];
         __convert_float32_to_bytes.b[2] = raw[1108];
         __convert_float32_to_bytes.b[3] = raw[1109];
         data.tracks[5].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1110];
         __convert_float32_to_bytes.b[1] = raw[1111];
         __convert_float32_to_bytes.b[2] = raw[1112];
         __convert_float32_to_bytes.b[3] = raw[1113];
         data.tracks[5].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1114];
         __convert_float32_to_bytes.b[1] = raw[1115];
         __convert_float32_to_bytes.b[2] = raw[1116];
         __convert_float32_to_bytes.b[3] = raw[1117];
         data.tracks[5].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[5].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1118];
         u.b[1] = raw[1119];
         u.b[2] = raw[1120];
         u.b[3] = raw[1121];
        switch(u.a) {
         case 0:
            data.tracks[5].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[5].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[5].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[5].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[5].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[5].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[5].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[5].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[5].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[5].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1122];
         __convert_uint16_to_bytes.b[1] = raw[1123];
         data.tracks[5].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1124];
         __convert_uint16_to_bytes.b[1] = raw[1125];
         data.tracks[5].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1126];
         __convert_uint16_to_bytes.b[1] = raw[1127];
         data.tracks[5].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1128];
         __convert_float32_to_bytes.b[1] = raw[1129];
         __convert_float32_to_bytes.b[2] = raw[1130];
         __convert_float32_to_bytes.b[3] = raw[1131];
         data.tracks[5].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1132];
         __convert_float32_to_bytes.b[1] = raw[1133];
         __convert_float32_to_bytes.b[2] = raw[1134];
         __convert_float32_to_bytes.b[3] = raw[1135];
         data.tracks[5].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1136];
         __convert_float32_to_bytes.b[1] = raw[1137];
         __convert_float32_to_bytes.b[2] = raw[1138];
         __convert_float32_to_bytes.b[3] = raw[1139];
         data.tracks[5].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1140];
         __convert_float32_to_bytes.b[1] = raw[1141];
         __convert_float32_to_bytes.b[2] = raw[1142];
         __convert_float32_to_bytes.b[3] = raw[1143];
         data.tracks[5].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1144];
         __convert_float32_to_bytes.b[1] = raw[1145];
         __convert_float32_to_bytes.b[2] = raw[1146];
         __convert_float32_to_bytes.b[3] = raw[1147];
         data.tracks[5].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[5].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1148];
         __convert_float32_to_bytes.b[1] = raw[1149];
         __convert_float32_to_bytes.b[2] = raw[1150];
         __convert_float32_to_bytes.b[3] = raw[1151];
         data.tracks[5].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[1152];
         __convert_uint32_to_bytes.b[1] = raw[1153];
         __convert_uint32_to_bytes.b[2] = raw[1154];
         __convert_uint32_to_bytes.b[3] = raw[1155];
         data.tracks[6].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1156];
         data.tracks[6].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1157];
         data.tracks[6].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1158];
         data.tracks[6].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1159];
         data.tracks[6].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1160];
         data.tracks[6].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1161];
         data.tracks[6].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1162];
         data.tracks[6].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[6].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1163];
         data.tracks[6].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[6].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[1164];
         __convert_uint64_to_bytes.b[1] = raw[1165];
         __convert_uint64_to_bytes.b[2] = raw[1166];
         __convert_uint64_to_bytes.b[3] = raw[1167];
         __convert_uint64_to_bytes.b[4] = raw[1168];
         __convert_uint64_to_bytes.b[5] = raw[1169];
         __convert_uint64_to_bytes.b[6] = raw[1170];
         __convert_uint64_to_bytes.b[7] = raw[1171];
         data.tracks[6].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[6].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1172];
         u.b[1] = raw[1173];
         u.b[2] = raw[1174];
         u.b[3] = raw[1175];
        switch(u.a) {
         case 0:
            data.tracks[6].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[6].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[6].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[6].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[6].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[6].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[6].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[6].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[6].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[6].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[6].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[6].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[6].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[6].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[6].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1176];
         u.b[1] = raw[1177];
         u.b[2] = raw[1178];
         u.b[3] = raw[1179];
        switch(u.a) {
         case 0:
            data.tracks[6].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[6].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[6].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[6].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[6].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[6].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[6].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[6].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[6].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[6].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[6].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[6].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[6].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[6].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[6].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[6].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[6].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[6].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[6].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[6].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[1180]) != 0x00u );
         data.tracks[6].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[1180]) != 0x00u );
         data.tracks[6].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[1180]) != 0x00u );
         data.tracks[6].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[1180]) != 0x00u );
         data.tracks[6].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[1180]) != 0x00u );
         data.tracks[6].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[1180]) != 0x00u );
         data.tracks[6].source_data.LIDAR = ( ((0x01u << 6) & raw[1180]) != 0x00u );
         data.tracks[6].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[1180]) != 0x00u );
         data.tracks[6].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[1181]) != 0x00u );
      } 
      { //READ data.tracks[6].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1182];
         __convert_float32_to_bytes.b[1] = raw[1183];
         __convert_float32_to_bytes.b[2] = raw[1184];
         __convert_float32_to_bytes.b[3] = raw[1185];
         data.tracks[6].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1186];
         __convert_float32_to_bytes.b[1] = raw[1187];
         __convert_float32_to_bytes.b[2] = raw[1188];
         __convert_float32_to_bytes.b[3] = raw[1189];
         data.tracks[6].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[1190];
         data.tracks[6].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1191];
         __convert_float32_to_bytes.b[1] = raw[1192];
         __convert_float32_to_bytes.b[2] = raw[1193];
         __convert_float32_to_bytes.b[3] = raw[1194];
         data.tracks[6].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1195];
         __convert_float32_to_bytes.b[1] = raw[1196];
         __convert_float32_to_bytes.b[2] = raw[1197];
         __convert_float32_to_bytes.b[3] = raw[1198];
         data.tracks[6].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1199];
         __convert_int16_to_bytes.b[1] = raw[1200];
         data.tracks[6].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1201];
         __convert_float32_to_bytes.b[1] = raw[1202];
         __convert_float32_to_bytes.b[2] = raw[1203];
         __convert_float32_to_bytes.b[3] = raw[1204];
         data.tracks[6].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[6].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1205];
         u.b[1] = raw[1206];
         u.b[2] = raw[1207];
         u.b[3] = raw[1208];
        switch(u.a) {
         case 0:
            data.tracks[6].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[6].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[6].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[6].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[6].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[6].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[1209];
         __convert_uint32_to_bytes.b[1] = raw[1210];
         __convert_uint32_to_bytes.b[2] = raw[1211];
         __convert_uint32_to_bytes.b[3] = raw[1212];
         data.tracks[6].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1213];
         __convert_float32_to_bytes.b[1] = raw[1214];
         __convert_float32_to_bytes.b[2] = raw[1215];
         __convert_float32_to_bytes.b[3] = raw[1216];
         data.tracks[6].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1217];
         __convert_float32_to_bytes.b[1] = raw[1218];
         __convert_float32_to_bytes.b[2] = raw[1219];
         __convert_float32_to_bytes.b[3] = raw[1220];
         data.tracks[6].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1221];
         __convert_float32_to_bytes.b[1] = raw[1222];
         __convert_float32_to_bytes.b[2] = raw[1223];
         __convert_float32_to_bytes.b[3] = raw[1224];
         data.tracks[6].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1225];
         __convert_int16_to_bytes.b[1] = raw[1226];
         data.tracks[6].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[6].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1227];
         __convert_float32_to_bytes.b[1] = raw[1228];
         __convert_float32_to_bytes.b[2] = raw[1229];
         __convert_float32_to_bytes.b[3] = raw[1230];
         data.tracks[6].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[6].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[1231]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[1231]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[1231]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[1231]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[1231]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[1231]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[1231]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[1231]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[1232]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[1232]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[1232]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[1232]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[1232]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[1232]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[1232]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[1232]) != 0x00u );
         data.tracks[6].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[1233]) != 0x00u );
      } 
      { //READ data.tracks[6].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[1234];
         __convert_uint64_to_bytes.b[1] = raw[1235];
         __convert_uint64_to_bytes.b[2] = raw[1236];
         __convert_uint64_to_bytes.b[3] = raw[1237];
         __convert_uint64_to_bytes.b[4] = raw[1238];
         __convert_uint64_to_bytes.b[5] = raw[1239];
         __convert_uint64_to_bytes.b[6] = raw[1240];
         __convert_uint64_to_bytes.b[7] = raw[1241];
         data.tracks[6].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1242];
         __convert_float64_to_bytes.b[1] = raw[1243];
         __convert_float64_to_bytes.b[2] = raw[1244];
         __convert_float64_to_bytes.b[3] = raw[1245];
         __convert_float64_to_bytes.b[4] = raw[1246];
         __convert_float64_to_bytes.b[5] = raw[1247];
         __convert_float64_to_bytes.b[6] = raw[1248];
         __convert_float64_to_bytes.b[7] = raw[1249];
         data.tracks[6].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1250];
         __convert_float64_to_bytes.b[1] = raw[1251];
         __convert_float64_to_bytes.b[2] = raw[1252];
         __convert_float64_to_bytes.b[3] = raw[1253];
         __convert_float64_to_bytes.b[4] = raw[1254];
         __convert_float64_to_bytes.b[5] = raw[1255];
         __convert_float64_to_bytes.b[6] = raw[1256];
         __convert_float64_to_bytes.b[7] = raw[1257];
         data.tracks[6].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1258];
         __convert_float32_to_bytes.b[1] = raw[1259];
         __convert_float32_to_bytes.b[2] = raw[1260];
         __convert_float32_to_bytes.b[3] = raw[1261];
         data.tracks[6].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1262];
         __convert_float32_to_bytes.b[1] = raw[1263];
         __convert_float32_to_bytes.b[2] = raw[1264];
         __convert_float32_to_bytes.b[3] = raw[1265];
         data.tracks[6].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1266];
         __convert_float32_to_bytes.b[1] = raw[1267];
         __convert_float32_to_bytes.b[2] = raw[1268];
         __convert_float32_to_bytes.b[3] = raw[1269];
         data.tracks[6].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1270];
         __convert_float32_to_bytes.b[1] = raw[1271];
         __convert_float32_to_bytes.b[2] = raw[1272];
         __convert_float32_to_bytes.b[3] = raw[1273];
         data.tracks[6].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1274];
         __convert_float32_to_bytes.b[1] = raw[1275];
         __convert_float32_to_bytes.b[2] = raw[1276];
         __convert_float32_to_bytes.b[3] = raw[1277];
         data.tracks[6].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1278];
         __convert_float32_to_bytes.b[1] = raw[1279];
         __convert_float32_to_bytes.b[2] = raw[1280];
         __convert_float32_to_bytes.b[3] = raw[1281];
         data.tracks[6].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1282];
         __convert_float32_to_bytes.b[1] = raw[1283];
         __convert_float32_to_bytes.b[2] = raw[1284];
         __convert_float32_to_bytes.b[3] = raw[1285];
         data.tracks[6].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1286];
         __convert_float32_to_bytes.b[1] = raw[1287];
         __convert_float32_to_bytes.b[2] = raw[1288];
         __convert_float32_to_bytes.b[3] = raw[1289];
         data.tracks[6].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1290];
         __convert_float32_to_bytes.b[1] = raw[1291];
         __convert_float32_to_bytes.b[2] = raw[1292];
         __convert_float32_to_bytes.b[3] = raw[1293];
         data.tracks[6].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1294];
         __convert_float32_to_bytes.b[1] = raw[1295];
         __convert_float32_to_bytes.b[2] = raw[1296];
         __convert_float32_to_bytes.b[3] = raw[1297];
         data.tracks[6].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1298];
         __convert_float32_to_bytes.b[1] = raw[1299];
         __convert_float32_to_bytes.b[2] = raw[1300];
         __convert_float32_to_bytes.b[3] = raw[1301];
         data.tracks[6].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1302];
         __convert_float32_to_bytes.b[1] = raw[1303];
         __convert_float32_to_bytes.b[2] = raw[1304];
         __convert_float32_to_bytes.b[3] = raw[1305];
         data.tracks[6].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1306];
         __convert_float32_to_bytes.b[1] = raw[1307];
         __convert_float32_to_bytes.b[2] = raw[1308];
         __convert_float32_to_bytes.b[3] = raw[1309];
         data.tracks[6].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[6].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1310];
         u.b[1] = raw[1311];
         u.b[2] = raw[1312];
         u.b[3] = raw[1313];
        switch(u.a) {
         case 0:
            data.tracks[6].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[6].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[6].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[6].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[6].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[6].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[6].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[6].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[6].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[6].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1314];
         __convert_uint16_to_bytes.b[1] = raw[1315];
         data.tracks[6].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1316];
         __convert_uint16_to_bytes.b[1] = raw[1317];
         data.tracks[6].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1318];
         __convert_uint16_to_bytes.b[1] = raw[1319];
         data.tracks[6].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1320];
         __convert_float32_to_bytes.b[1] = raw[1321];
         __convert_float32_to_bytes.b[2] = raw[1322];
         __convert_float32_to_bytes.b[3] = raw[1323];
         data.tracks[6].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1324];
         __convert_float32_to_bytes.b[1] = raw[1325];
         __convert_float32_to_bytes.b[2] = raw[1326];
         __convert_float32_to_bytes.b[3] = raw[1327];
         data.tracks[6].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1328];
         __convert_float32_to_bytes.b[1] = raw[1329];
         __convert_float32_to_bytes.b[2] = raw[1330];
         __convert_float32_to_bytes.b[3] = raw[1331];
         data.tracks[6].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1332];
         __convert_float32_to_bytes.b[1] = raw[1333];
         __convert_float32_to_bytes.b[2] = raw[1334];
         __convert_float32_to_bytes.b[3] = raw[1335];
         data.tracks[6].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1336];
         __convert_float32_to_bytes.b[1] = raw[1337];
         __convert_float32_to_bytes.b[2] = raw[1338];
         __convert_float32_to_bytes.b[3] = raw[1339];
         data.tracks[6].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[6].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1340];
         __convert_float32_to_bytes.b[1] = raw[1341];
         __convert_float32_to_bytes.b[2] = raw[1342];
         __convert_float32_to_bytes.b[3] = raw[1343];
         data.tracks[6].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[1344];
         __convert_uint32_to_bytes.b[1] = raw[1345];
         __convert_uint32_to_bytes.b[2] = raw[1346];
         __convert_uint32_to_bytes.b[3] = raw[1347];
         data.tracks[7].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1348];
         data.tracks[7].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1349];
         data.tracks[7].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1350];
         data.tracks[7].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1351];
         data.tracks[7].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1352];
         data.tracks[7].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1353];
         data.tracks[7].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1354];
         data.tracks[7].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[7].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1355];
         data.tracks[7].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[7].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[1356];
         __convert_uint64_to_bytes.b[1] = raw[1357];
         __convert_uint64_to_bytes.b[2] = raw[1358];
         __convert_uint64_to_bytes.b[3] = raw[1359];
         __convert_uint64_to_bytes.b[4] = raw[1360];
         __convert_uint64_to_bytes.b[5] = raw[1361];
         __convert_uint64_to_bytes.b[6] = raw[1362];
         __convert_uint64_to_bytes.b[7] = raw[1363];
         data.tracks[7].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[7].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1364];
         u.b[1] = raw[1365];
         u.b[2] = raw[1366];
         u.b[3] = raw[1367];
        switch(u.a) {
         case 0:
            data.tracks[7].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[7].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[7].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[7].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[7].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[7].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[7].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[7].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[7].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[7].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[7].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[7].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[7].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[7].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[7].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1368];
         u.b[1] = raw[1369];
         u.b[2] = raw[1370];
         u.b[3] = raw[1371];
        switch(u.a) {
         case 0:
            data.tracks[7].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[7].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[7].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[7].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[7].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[7].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[7].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[7].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[7].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[7].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[7].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[7].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[7].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[7].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[7].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[7].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[7].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[7].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[7].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[7].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[1372]) != 0x00u );
         data.tracks[7].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[1372]) != 0x00u );
         data.tracks[7].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[1372]) != 0x00u );
         data.tracks[7].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[1372]) != 0x00u );
         data.tracks[7].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[1372]) != 0x00u );
         data.tracks[7].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[1372]) != 0x00u );
         data.tracks[7].source_data.LIDAR = ( ((0x01u << 6) & raw[1372]) != 0x00u );
         data.tracks[7].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[1372]) != 0x00u );
         data.tracks[7].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[1373]) != 0x00u );
      } 
      { //READ data.tracks[7].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1374];
         __convert_float32_to_bytes.b[1] = raw[1375];
         __convert_float32_to_bytes.b[2] = raw[1376];
         __convert_float32_to_bytes.b[3] = raw[1377];
         data.tracks[7].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1378];
         __convert_float32_to_bytes.b[1] = raw[1379];
         __convert_float32_to_bytes.b[2] = raw[1380];
         __convert_float32_to_bytes.b[3] = raw[1381];
         data.tracks[7].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[1382];
         data.tracks[7].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1383];
         __convert_float32_to_bytes.b[1] = raw[1384];
         __convert_float32_to_bytes.b[2] = raw[1385];
         __convert_float32_to_bytes.b[3] = raw[1386];
         data.tracks[7].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1387];
         __convert_float32_to_bytes.b[1] = raw[1388];
         __convert_float32_to_bytes.b[2] = raw[1389];
         __convert_float32_to_bytes.b[3] = raw[1390];
         data.tracks[7].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1391];
         __convert_int16_to_bytes.b[1] = raw[1392];
         data.tracks[7].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1393];
         __convert_float32_to_bytes.b[1] = raw[1394];
         __convert_float32_to_bytes.b[2] = raw[1395];
         __convert_float32_to_bytes.b[3] = raw[1396];
         data.tracks[7].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[7].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1397];
         u.b[1] = raw[1398];
         u.b[2] = raw[1399];
         u.b[3] = raw[1400];
        switch(u.a) {
         case 0:
            data.tracks[7].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[7].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[7].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[7].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[7].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[7].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[1401];
         __convert_uint32_to_bytes.b[1] = raw[1402];
         __convert_uint32_to_bytes.b[2] = raw[1403];
         __convert_uint32_to_bytes.b[3] = raw[1404];
         data.tracks[7].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1405];
         __convert_float32_to_bytes.b[1] = raw[1406];
         __convert_float32_to_bytes.b[2] = raw[1407];
         __convert_float32_to_bytes.b[3] = raw[1408];
         data.tracks[7].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1409];
         __convert_float32_to_bytes.b[1] = raw[1410];
         __convert_float32_to_bytes.b[2] = raw[1411];
         __convert_float32_to_bytes.b[3] = raw[1412];
         data.tracks[7].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1413];
         __convert_float32_to_bytes.b[1] = raw[1414];
         __convert_float32_to_bytes.b[2] = raw[1415];
         __convert_float32_to_bytes.b[3] = raw[1416];
         data.tracks[7].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1417];
         __convert_int16_to_bytes.b[1] = raw[1418];
         data.tracks[7].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[7].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1419];
         __convert_float32_to_bytes.b[1] = raw[1420];
         __convert_float32_to_bytes.b[2] = raw[1421];
         __convert_float32_to_bytes.b[3] = raw[1422];
         data.tracks[7].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[7].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[1423]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[1423]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[1423]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[1423]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[1423]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[1423]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[1423]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[1423]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[1424]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[1424]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[1424]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[1424]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[1424]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[1424]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[1424]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[1424]) != 0x00u );
         data.tracks[7].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[1425]) != 0x00u );
      } 
      { //READ data.tracks[7].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[1426];
         __convert_uint64_to_bytes.b[1] = raw[1427];
         __convert_uint64_to_bytes.b[2] = raw[1428];
         __convert_uint64_to_bytes.b[3] = raw[1429];
         __convert_uint64_to_bytes.b[4] = raw[1430];
         __convert_uint64_to_bytes.b[5] = raw[1431];
         __convert_uint64_to_bytes.b[6] = raw[1432];
         __convert_uint64_to_bytes.b[7] = raw[1433];
         data.tracks[7].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1434];
         __convert_float64_to_bytes.b[1] = raw[1435];
         __convert_float64_to_bytes.b[2] = raw[1436];
         __convert_float64_to_bytes.b[3] = raw[1437];
         __convert_float64_to_bytes.b[4] = raw[1438];
         __convert_float64_to_bytes.b[5] = raw[1439];
         __convert_float64_to_bytes.b[6] = raw[1440];
         __convert_float64_to_bytes.b[7] = raw[1441];
         data.tracks[7].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1442];
         __convert_float64_to_bytes.b[1] = raw[1443];
         __convert_float64_to_bytes.b[2] = raw[1444];
         __convert_float64_to_bytes.b[3] = raw[1445];
         __convert_float64_to_bytes.b[4] = raw[1446];
         __convert_float64_to_bytes.b[5] = raw[1447];
         __convert_float64_to_bytes.b[6] = raw[1448];
         __convert_float64_to_bytes.b[7] = raw[1449];
         data.tracks[7].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1450];
         __convert_float32_to_bytes.b[1] = raw[1451];
         __convert_float32_to_bytes.b[2] = raw[1452];
         __convert_float32_to_bytes.b[3] = raw[1453];
         data.tracks[7].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1454];
         __convert_float32_to_bytes.b[1] = raw[1455];
         __convert_float32_to_bytes.b[2] = raw[1456];
         __convert_float32_to_bytes.b[3] = raw[1457];
         data.tracks[7].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1458];
         __convert_float32_to_bytes.b[1] = raw[1459];
         __convert_float32_to_bytes.b[2] = raw[1460];
         __convert_float32_to_bytes.b[3] = raw[1461];
         data.tracks[7].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1462];
         __convert_float32_to_bytes.b[1] = raw[1463];
         __convert_float32_to_bytes.b[2] = raw[1464];
         __convert_float32_to_bytes.b[3] = raw[1465];
         data.tracks[7].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1466];
         __convert_float32_to_bytes.b[1] = raw[1467];
         __convert_float32_to_bytes.b[2] = raw[1468];
         __convert_float32_to_bytes.b[3] = raw[1469];
         data.tracks[7].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1470];
         __convert_float32_to_bytes.b[1] = raw[1471];
         __convert_float32_to_bytes.b[2] = raw[1472];
         __convert_float32_to_bytes.b[3] = raw[1473];
         data.tracks[7].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1474];
         __convert_float32_to_bytes.b[1] = raw[1475];
         __convert_float32_to_bytes.b[2] = raw[1476];
         __convert_float32_to_bytes.b[3] = raw[1477];
         data.tracks[7].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1478];
         __convert_float32_to_bytes.b[1] = raw[1479];
         __convert_float32_to_bytes.b[2] = raw[1480];
         __convert_float32_to_bytes.b[3] = raw[1481];
         data.tracks[7].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1482];
         __convert_float32_to_bytes.b[1] = raw[1483];
         __convert_float32_to_bytes.b[2] = raw[1484];
         __convert_float32_to_bytes.b[3] = raw[1485];
         data.tracks[7].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1486];
         __convert_float32_to_bytes.b[1] = raw[1487];
         __convert_float32_to_bytes.b[2] = raw[1488];
         __convert_float32_to_bytes.b[3] = raw[1489];
         data.tracks[7].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1490];
         __convert_float32_to_bytes.b[1] = raw[1491];
         __convert_float32_to_bytes.b[2] = raw[1492];
         __convert_float32_to_bytes.b[3] = raw[1493];
         data.tracks[7].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1494];
         __convert_float32_to_bytes.b[1] = raw[1495];
         __convert_float32_to_bytes.b[2] = raw[1496];
         __convert_float32_to_bytes.b[3] = raw[1497];
         data.tracks[7].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1498];
         __convert_float32_to_bytes.b[1] = raw[1499];
         __convert_float32_to_bytes.b[2] = raw[1500];
         __convert_float32_to_bytes.b[3] = raw[1501];
         data.tracks[7].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[7].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1502];
         u.b[1] = raw[1503];
         u.b[2] = raw[1504];
         u.b[3] = raw[1505];
        switch(u.a) {
         case 0:
            data.tracks[7].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[7].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[7].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[7].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[7].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[7].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[7].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[7].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[7].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[7].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1506];
         __convert_uint16_to_bytes.b[1] = raw[1507];
         data.tracks[7].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1508];
         __convert_uint16_to_bytes.b[1] = raw[1509];
         data.tracks[7].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1510];
         __convert_uint16_to_bytes.b[1] = raw[1511];
         data.tracks[7].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1512];
         __convert_float32_to_bytes.b[1] = raw[1513];
         __convert_float32_to_bytes.b[2] = raw[1514];
         __convert_float32_to_bytes.b[3] = raw[1515];
         data.tracks[7].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1516];
         __convert_float32_to_bytes.b[1] = raw[1517];
         __convert_float32_to_bytes.b[2] = raw[1518];
         __convert_float32_to_bytes.b[3] = raw[1519];
         data.tracks[7].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1520];
         __convert_float32_to_bytes.b[1] = raw[1521];
         __convert_float32_to_bytes.b[2] = raw[1522];
         __convert_float32_to_bytes.b[3] = raw[1523];
         data.tracks[7].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1524];
         __convert_float32_to_bytes.b[1] = raw[1525];
         __convert_float32_to_bytes.b[2] = raw[1526];
         __convert_float32_to_bytes.b[3] = raw[1527];
         data.tracks[7].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1528];
         __convert_float32_to_bytes.b[1] = raw[1529];
         __convert_float32_to_bytes.b[2] = raw[1530];
         __convert_float32_to_bytes.b[3] = raw[1531];
         data.tracks[7].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[7].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1532];
         __convert_float32_to_bytes.b[1] = raw[1533];
         __convert_float32_to_bytes.b[2] = raw[1534];
         __convert_float32_to_bytes.b[3] = raw[1535];
         data.tracks[7].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[1536];
         __convert_uint32_to_bytes.b[1] = raw[1537];
         __convert_uint32_to_bytes.b[2] = raw[1538];
         __convert_uint32_to_bytes.b[3] = raw[1539];
         data.tracks[8].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1540];
         data.tracks[8].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1541];
         data.tracks[8].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1542];
         data.tracks[8].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1543];
         data.tracks[8].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1544];
         data.tracks[8].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1545];
         data.tracks[8].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1546];
         data.tracks[8].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[8].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1547];
         data.tracks[8].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[8].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[1548];
         __convert_uint64_to_bytes.b[1] = raw[1549];
         __convert_uint64_to_bytes.b[2] = raw[1550];
         __convert_uint64_to_bytes.b[3] = raw[1551];
         __convert_uint64_to_bytes.b[4] = raw[1552];
         __convert_uint64_to_bytes.b[5] = raw[1553];
         __convert_uint64_to_bytes.b[6] = raw[1554];
         __convert_uint64_to_bytes.b[7] = raw[1555];
         data.tracks[8].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[8].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1556];
         u.b[1] = raw[1557];
         u.b[2] = raw[1558];
         u.b[3] = raw[1559];
        switch(u.a) {
         case 0:
            data.tracks[8].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[8].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[8].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[8].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[8].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[8].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[8].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[8].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[8].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[8].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[8].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[8].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[8].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[8].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[8].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1560];
         u.b[1] = raw[1561];
         u.b[2] = raw[1562];
         u.b[3] = raw[1563];
        switch(u.a) {
         case 0:
            data.tracks[8].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[8].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[8].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[8].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[8].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[8].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[8].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[8].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[8].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[8].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[8].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[8].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[8].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[8].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[8].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[8].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[8].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[8].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[8].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[8].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[1564]) != 0x00u );
         data.tracks[8].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[1564]) != 0x00u );
         data.tracks[8].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[1564]) != 0x00u );
         data.tracks[8].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[1564]) != 0x00u );
         data.tracks[8].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[1564]) != 0x00u );
         data.tracks[8].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[1564]) != 0x00u );
         data.tracks[8].source_data.LIDAR = ( ((0x01u << 6) & raw[1564]) != 0x00u );
         data.tracks[8].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[1564]) != 0x00u );
         data.tracks[8].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[1565]) != 0x00u );
      } 
      { //READ data.tracks[8].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1566];
         __convert_float32_to_bytes.b[1] = raw[1567];
         __convert_float32_to_bytes.b[2] = raw[1568];
         __convert_float32_to_bytes.b[3] = raw[1569];
         data.tracks[8].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1570];
         __convert_float32_to_bytes.b[1] = raw[1571];
         __convert_float32_to_bytes.b[2] = raw[1572];
         __convert_float32_to_bytes.b[3] = raw[1573];
         data.tracks[8].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[1574];
         data.tracks[8].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1575];
         __convert_float32_to_bytes.b[1] = raw[1576];
         __convert_float32_to_bytes.b[2] = raw[1577];
         __convert_float32_to_bytes.b[3] = raw[1578];
         data.tracks[8].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1579];
         __convert_float32_to_bytes.b[1] = raw[1580];
         __convert_float32_to_bytes.b[2] = raw[1581];
         __convert_float32_to_bytes.b[3] = raw[1582];
         data.tracks[8].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1583];
         __convert_int16_to_bytes.b[1] = raw[1584];
         data.tracks[8].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1585];
         __convert_float32_to_bytes.b[1] = raw[1586];
         __convert_float32_to_bytes.b[2] = raw[1587];
         __convert_float32_to_bytes.b[3] = raw[1588];
         data.tracks[8].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[8].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1589];
         u.b[1] = raw[1590];
         u.b[2] = raw[1591];
         u.b[3] = raw[1592];
        switch(u.a) {
         case 0:
            data.tracks[8].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[8].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[8].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[8].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[8].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[8].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[1593];
         __convert_uint32_to_bytes.b[1] = raw[1594];
         __convert_uint32_to_bytes.b[2] = raw[1595];
         __convert_uint32_to_bytes.b[3] = raw[1596];
         data.tracks[8].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1597];
         __convert_float32_to_bytes.b[1] = raw[1598];
         __convert_float32_to_bytes.b[2] = raw[1599];
         __convert_float32_to_bytes.b[3] = raw[1600];
         data.tracks[8].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1601];
         __convert_float32_to_bytes.b[1] = raw[1602];
         __convert_float32_to_bytes.b[2] = raw[1603];
         __convert_float32_to_bytes.b[3] = raw[1604];
         data.tracks[8].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1605];
         __convert_float32_to_bytes.b[1] = raw[1606];
         __convert_float32_to_bytes.b[2] = raw[1607];
         __convert_float32_to_bytes.b[3] = raw[1608];
         data.tracks[8].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1609];
         __convert_int16_to_bytes.b[1] = raw[1610];
         data.tracks[8].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[8].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1611];
         __convert_float32_to_bytes.b[1] = raw[1612];
         __convert_float32_to_bytes.b[2] = raw[1613];
         __convert_float32_to_bytes.b[3] = raw[1614];
         data.tracks[8].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[8].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[1615]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[1615]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[1615]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[1615]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[1615]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[1615]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[1615]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[1615]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[1616]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[1616]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[1616]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[1616]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[1616]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[1616]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[1616]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[1616]) != 0x00u );
         data.tracks[8].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[1617]) != 0x00u );
      } 
      { //READ data.tracks[8].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[1618];
         __convert_uint64_to_bytes.b[1] = raw[1619];
         __convert_uint64_to_bytes.b[2] = raw[1620];
         __convert_uint64_to_bytes.b[3] = raw[1621];
         __convert_uint64_to_bytes.b[4] = raw[1622];
         __convert_uint64_to_bytes.b[5] = raw[1623];
         __convert_uint64_to_bytes.b[6] = raw[1624];
         __convert_uint64_to_bytes.b[7] = raw[1625];
         data.tracks[8].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1626];
         __convert_float64_to_bytes.b[1] = raw[1627];
         __convert_float64_to_bytes.b[2] = raw[1628];
         __convert_float64_to_bytes.b[3] = raw[1629];
         __convert_float64_to_bytes.b[4] = raw[1630];
         __convert_float64_to_bytes.b[5] = raw[1631];
         __convert_float64_to_bytes.b[6] = raw[1632];
         __convert_float64_to_bytes.b[7] = raw[1633];
         data.tracks[8].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1634];
         __convert_float64_to_bytes.b[1] = raw[1635];
         __convert_float64_to_bytes.b[2] = raw[1636];
         __convert_float64_to_bytes.b[3] = raw[1637];
         __convert_float64_to_bytes.b[4] = raw[1638];
         __convert_float64_to_bytes.b[5] = raw[1639];
         __convert_float64_to_bytes.b[6] = raw[1640];
         __convert_float64_to_bytes.b[7] = raw[1641];
         data.tracks[8].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1642];
         __convert_float32_to_bytes.b[1] = raw[1643];
         __convert_float32_to_bytes.b[2] = raw[1644];
         __convert_float32_to_bytes.b[3] = raw[1645];
         data.tracks[8].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1646];
         __convert_float32_to_bytes.b[1] = raw[1647];
         __convert_float32_to_bytes.b[2] = raw[1648];
         __convert_float32_to_bytes.b[3] = raw[1649];
         data.tracks[8].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1650];
         __convert_float32_to_bytes.b[1] = raw[1651];
         __convert_float32_to_bytes.b[2] = raw[1652];
         __convert_float32_to_bytes.b[3] = raw[1653];
         data.tracks[8].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1654];
         __convert_float32_to_bytes.b[1] = raw[1655];
         __convert_float32_to_bytes.b[2] = raw[1656];
         __convert_float32_to_bytes.b[3] = raw[1657];
         data.tracks[8].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1658];
         __convert_float32_to_bytes.b[1] = raw[1659];
         __convert_float32_to_bytes.b[2] = raw[1660];
         __convert_float32_to_bytes.b[3] = raw[1661];
         data.tracks[8].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1662];
         __convert_float32_to_bytes.b[1] = raw[1663];
         __convert_float32_to_bytes.b[2] = raw[1664];
         __convert_float32_to_bytes.b[3] = raw[1665];
         data.tracks[8].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1666];
         __convert_float32_to_bytes.b[1] = raw[1667];
         __convert_float32_to_bytes.b[2] = raw[1668];
         __convert_float32_to_bytes.b[3] = raw[1669];
         data.tracks[8].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1670];
         __convert_float32_to_bytes.b[1] = raw[1671];
         __convert_float32_to_bytes.b[2] = raw[1672];
         __convert_float32_to_bytes.b[3] = raw[1673];
         data.tracks[8].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1674];
         __convert_float32_to_bytes.b[1] = raw[1675];
         __convert_float32_to_bytes.b[2] = raw[1676];
         __convert_float32_to_bytes.b[3] = raw[1677];
         data.tracks[8].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1678];
         __convert_float32_to_bytes.b[1] = raw[1679];
         __convert_float32_to_bytes.b[2] = raw[1680];
         __convert_float32_to_bytes.b[3] = raw[1681];
         data.tracks[8].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1682];
         __convert_float32_to_bytes.b[1] = raw[1683];
         __convert_float32_to_bytes.b[2] = raw[1684];
         __convert_float32_to_bytes.b[3] = raw[1685];
         data.tracks[8].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1686];
         __convert_float32_to_bytes.b[1] = raw[1687];
         __convert_float32_to_bytes.b[2] = raw[1688];
         __convert_float32_to_bytes.b[3] = raw[1689];
         data.tracks[8].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1690];
         __convert_float32_to_bytes.b[1] = raw[1691];
         __convert_float32_to_bytes.b[2] = raw[1692];
         __convert_float32_to_bytes.b[3] = raw[1693];
         data.tracks[8].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[8].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1694];
         u.b[1] = raw[1695];
         u.b[2] = raw[1696];
         u.b[3] = raw[1697];
        switch(u.a) {
         case 0:
            data.tracks[8].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[8].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[8].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[8].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[8].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[8].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[8].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[8].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[8].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[8].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1698];
         __convert_uint16_to_bytes.b[1] = raw[1699];
         data.tracks[8].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1700];
         __convert_uint16_to_bytes.b[1] = raw[1701];
         data.tracks[8].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1702];
         __convert_uint16_to_bytes.b[1] = raw[1703];
         data.tracks[8].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1704];
         __convert_float32_to_bytes.b[1] = raw[1705];
         __convert_float32_to_bytes.b[2] = raw[1706];
         __convert_float32_to_bytes.b[3] = raw[1707];
         data.tracks[8].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1708];
         __convert_float32_to_bytes.b[1] = raw[1709];
         __convert_float32_to_bytes.b[2] = raw[1710];
         __convert_float32_to_bytes.b[3] = raw[1711];
         data.tracks[8].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1712];
         __convert_float32_to_bytes.b[1] = raw[1713];
         __convert_float32_to_bytes.b[2] = raw[1714];
         __convert_float32_to_bytes.b[3] = raw[1715];
         data.tracks[8].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1716];
         __convert_float32_to_bytes.b[1] = raw[1717];
         __convert_float32_to_bytes.b[2] = raw[1718];
         __convert_float32_to_bytes.b[3] = raw[1719];
         data.tracks[8].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1720];
         __convert_float32_to_bytes.b[1] = raw[1721];
         __convert_float32_to_bytes.b[2] = raw[1722];
         __convert_float32_to_bytes.b[3] = raw[1723];
         data.tracks[8].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[8].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1724];
         __convert_float32_to_bytes.b[1] = raw[1725];
         __convert_float32_to_bytes.b[2] = raw[1726];
         __convert_float32_to_bytes.b[3] = raw[1727];
         data.tracks[8].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].track_id        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[1728];
         __convert_uint32_to_bytes.b[1] = raw[1729];
         __convert_uint32_to_bytes.b[2] = raw[1730];
         __convert_uint32_to_bytes.b[3] = raw[1731];
         data.tracks[9].track_id = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1732];
         data.tracks[9].callsign[0] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1733];
         data.tracks[9].callsign[1] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1734];
         data.tracks[9].callsign[2] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1735];
         data.tracks[9].callsign[3] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1736];
         data.tracks[9].callsign[4] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1737];
         data.tracks[9].callsign[5] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1738];
         data.tracks[9].callsign[6] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[9].callsign        text        1 bytes
         __convert_text_to_bytes.b[0] = raw[1739];
         data.tracks[9].callsign[7] = __convert_text_to_bytes.a;
      } 
      { //READ data.tracks[9].track_time        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[1740];
         __convert_uint64_to_bytes.b[1] = raw[1741];
         __convert_uint64_to_bytes.b[2] = raw[1742];
         __convert_uint64_to_bytes.b[3] = raw[1743];
         __convert_uint64_to_bytes.b[4] = raw[1744];
         __convert_uint64_to_bytes.b[5] = raw[1745];
         __convert_uint64_to_bytes.b[6] = raw[1746];
         __convert_uint64_to_bytes.b[7] = raw[1747];
         data.tracks[9].track_time = __convert_uint64_to_bytes.a;
      } 
      { //READ enum data.tracks[9].aircraft_category
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1748];
         u.b[1] = raw[1749];
         u.b[2] = raw[1750];
         u.b[3] = raw[1751];
        switch(u.a) {
         case 0:
            data.tracks[9].aircraft_category = aircraft_category_t::NA;
            break;
         case 1:
            data.tracks[9].aircraft_category = aircraft_category_t::LIGHT;
            break;
         case 2:
            data.tracks[9].aircraft_category = aircraft_category_t::SMALL;
            break;
         case 3:
            data.tracks[9].aircraft_category = aircraft_category_t::LARGE;
            break;
         case 4:
            data.tracks[9].aircraft_category = aircraft_category_t::HVORTEX;
            break;
         case 5:
            data.tracks[9].aircraft_category = aircraft_category_t::HEAVY;
            break;
         case 6:
            data.tracks[9].aircraft_category = aircraft_category_t::HPERFORMANCE;
            break;
         case 7:
            data.tracks[9].aircraft_category = aircraft_category_t::ROTORCRAFT;
            break;
         case 8:
            data.tracks[9].aircraft_category = aircraft_category_t::GLIDER;
            break;
         case 9:
            data.tracks[9].aircraft_category = aircraft_category_t::LTA;
            break;
         case 10:
            data.tracks[9].aircraft_category = aircraft_category_t::PARACHUTE;
            break;
         case 11:
            data.tracks[9].aircraft_category = aircraft_category_t::UAV;
            break;
         case 12:
            data.tracks[9].aircraft_category = aircraft_category_t::SPACE;
            break;
         case 13: //enum_end
            data.tracks[9].aircraft_category = aircraft_category_t::aircraft_category_t_end;
            break;
        }
      } 
      { //READ enum data.tracks[9].aircraft_model
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1752];
         u.b[1] = raw[1753];
         u.b[2] = raw[1754];
         u.b[3] = raw[1755];
        switch(u.a) {
         case 0:
            data.tracks[9].aircraft_model = airframe::B717;
            break;
         case 1:
            data.tracks[9].aircraft_model = airframe::B727;
            break;
         case 2:
            data.tracks[9].aircraft_model = airframe::B737;
            break;
         case 3:
            data.tracks[9].aircraft_model = airframe::B747;
            break;
         case 4:
            data.tracks[9].aircraft_model = airframe::B757;
            break;
         case 5:
            data.tracks[9].aircraft_model = airframe::B767;
            break;
         case 6:
            data.tracks[9].aircraft_model = airframe::B777;
            break;
         case 7:
            data.tracks[9].aircraft_model = airframe::B787;
            break;
         case 8:
            data.tracks[9].aircraft_model = airframe::A310;
            break;
         case 9:
            data.tracks[9].aircraft_model = airframe::A319;
            break;
         case 10:
            data.tracks[9].aircraft_model = airframe::A320;
            break;
         case 11:
            data.tracks[9].aircraft_model = airframe::A321;
            break;
         case 12:
            data.tracks[9].aircraft_model = airframe::A330;
            break;
         case 13:
            data.tracks[9].aircraft_model = airframe::A340;
            break;
         case 14:
            data.tracks[9].aircraft_model = airframe::C172;
            break;
         case 15:
            data.tracks[9].aircraft_model = airframe::SCE;
            break;
         case 16:
            data.tracks[9].aircraft_model = airframe::MUG;
            break;
         case 17:
            data.tracks[9].aircraft_model = airframe::UNKNOWN_AIRFRAME;
            break;
         case 18: //enum_end
            data.tracks[9].aircraft_model = airframe::airframe_end;
            break;
        }
      } 
      { //READ bitfield source
         data.tracks[9].source_data.UNFUSED_TRACK = ( ((0x01u << 0) & raw[1756]) != 0x00u );
         data.tracks[9].source_data.FUSED_TRACK = ( ((0x01u << 1) & raw[1756]) != 0x00u );
         data.tracks[9].source_data.EO_IR_CAMERA_TRACK = ( ((0x01u << 2) & raw[1756]) != 0x00u );
         data.tracks[9].source_data.ADSB_TRACK = ( ((0x01u << 3) & raw[1756]) != 0x00u );
         data.tracks[9].source_data.ACAS_TRACK = ( ((0x01u << 4) & raw[1756]) != 0x00u );
         data.tracks[9].source_data.RADAR_TRACK = ( ((0x01u << 5) & raw[1756]) != 0x00u );
         data.tracks[9].source_data.LIDAR = ( ((0x01u << 6) & raw[1756]) != 0x00u );
         data.tracks[9].source_data.ACOUSTIC = ( ((0x01u << 7) & raw[1756]) != 0x00u );
         data.tracks[9].source_data.UNKNOWN_SOURCE = ( ((0x01u << 0) & raw[1757]) != 0x00u );
      } 
      { //READ data.tracks[9].track_state.relative_range        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1758];
         __convert_float32_to_bytes.b[1] = raw[1759];
         __convert_float32_to_bytes.b[2] = raw[1760];
         __convert_float32_to_bytes.b[3] = raw[1761];
         data.tracks[9].track_state.relative_range = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_bearing        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1762];
         __convert_float32_to_bytes.b[1] = raw[1763];
         __convert_float32_to_bytes.b[2] = raw[1764];
         __convert_float32_to_bytes.b[3] = raw[1765];
         data.tracks[9].track_state.relative_bearing = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.bearing_invalid        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[1766];
         data.tracks[9].track_state.bearing_invalid = __convert_uint8_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1767];
         __convert_float32_to_bytes.b[1] = raw[1768];
         __convert_float32_to_bytes.b[2] = raw[1769];
         __convert_float32_to_bytes.b[3] = raw[1770];
         data.tracks[9].track_state.relative_ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1771];
         __convert_float32_to_bytes.b[1] = raw[1772];
         __convert_float32_to_bytes.b[2] = raw[1773];
         __convert_float32_to_bytes.b[3] = raw[1774];
         data.tracks[9].track_state.relative_ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_altitude        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1775];
         __convert_int16_to_bytes.b[1] = raw[1776];
         data.tracks[9].track_state.relative_altitude = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_vertical_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1777];
         __convert_float32_to_bytes.b[1] = raw[1778];
         __convert_float32_to_bytes.b[2] = raw[1779];
         __convert_float32_to_bytes.b[3] = raw[1780];
         data.tracks[9].track_state.relative_vertical_speed = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[9].track_state.track_size
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1781];
         u.b[1] = raw[1782];
         u.b[2] = raw[1783];
         u.b[3] = raw[1784];
        switch(u.a) {
         case 0:
            data.tracks[9].track_state.track_size = track_size_t::UNKNOWN_TRACK_SIZE;
            break;
         case 1:
            data.tracks[9].track_state.track_size = track_size_t::SMALL_TRACK;
            break;
         case 2:
            data.tracks[9].track_state.track_size = track_size_t::MEDIUM_TRACK;
            break;
         case 3:
            data.tracks[9].track_state.track_size = track_size_t::LARGE_TRACK;
            break;
         case 4: //enum_end
            data.tracks[9].track_state.track_size = track_size_t::track_size_t_end;
            break;
        }
      } 
      { //READ data.tracks[9].track_state.relative_range_uncertainty        uint32        4 bytes
         __convert_uint32_to_bytes.b[0] = raw[1785];
         __convert_uint32_to_bytes.b[1] = raw[1786];
         __convert_uint32_to_bytes.b[2] = raw[1787];
         __convert_uint32_to_bytes.b[3] = raw[1788];
         data.tracks[9].track_state.relative_range_uncertainty = __convert_uint32_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_bearing_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1789];
         __convert_float32_to_bytes.b[1] = raw[1790];
         __convert_float32_to_bytes.b[2] = raw[1791];
         __convert_float32_to_bytes.b[3] = raw[1792];
         data.tracks[9].track_state.relative_bearing_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_ground_speed_north_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1793];
         __convert_float32_to_bytes.b[1] = raw[1794];
         __convert_float32_to_bytes.b[2] = raw[1795];
         __convert_float32_to_bytes.b[3] = raw[1796];
         data.tracks[9].track_state.relative_ground_speed_north_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_ground_speed_east_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1797];
         __convert_float32_to_bytes.b[1] = raw[1798];
         __convert_float32_to_bytes.b[2] = raw[1799];
         __convert_float32_to_bytes.b[3] = raw[1800];
         data.tracks[9].track_state.relative_ground_speed_east_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_altitude_uncertainty        int16        2 bytes
         __convert_int16_to_bytes.b[0] = raw[1801];
         __convert_int16_to_bytes.b[1] = raw[1802];
         data.tracks[9].track_state.relative_altitude_uncertainty = __convert_int16_to_bytes.a;
      } 
      { //READ data.tracks[9].track_state.relative_vertical_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1803];
         __convert_float32_to_bytes.b[1] = raw[1804];
         __convert_float32_to_bytes.b[2] = raw[1805];
         __convert_float32_to_bytes.b[3] = raw[1806];
         data.tracks[9].track_state.relative_vertical_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ bitfield state_variable
         data.tracks[9].aircraft_state.available_states.TOA = ( ((0x01u << 0) & raw[1807]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.LONGITUDE = ( ((0x01u << 1) & raw[1807]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.LATITUDE = ( ((0x01u << 2) & raw[1807]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.GEOMETRIC_ALTITUDE = ( ((0x01u << 3) & raw[1807]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.BAROMETRIC_ALTITUDE = ( ((0x01u << 4) & raw[1807]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.TRUE_HEADING = ( ((0x01u << 5) & raw[1807]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.MAGNETIC_HEADING = ( ((0x01u << 6) & raw[1807]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.GROUND_TRACK_ANGLE = ( ((0x01u << 7) & raw[1807]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.TRUE_AIRSPEED = ( ((0x01u << 0) & raw[1808]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.INDICATED_AISPEED = ( ((0x01u << 1) & raw[1808]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.GROUND_SPEED_NORTH = ( ((0x01u << 2) & raw[1808]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.GROUND_SPEED_EAST = ( ((0x01u << 3) & raw[1808]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.GEOMETRIC_VERTICAL_RATE = ( ((0x01u << 4) & raw[1808]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.PRESSURE_VERTICAL_RATE = ( ((0x01u << 5) & raw[1808]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.HEADING_RATE = ( ((0x01u << 6) & raw[1808]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.GROUND_TRACK_RATE = ( ((0x01u << 7) & raw[1808]) != 0x00u );
         data.tracks[9].aircraft_state.available_states.MODE = ( ((0x01u << 0) & raw[1809]) != 0x00u );
      } 
      { //READ data.tracks[9].aircraft_state.toa        uint64        8 bytes
         __convert_uint64_to_bytes.b[0] = raw[1810];
         __convert_uint64_to_bytes.b[1] = raw[1811];
         __convert_uint64_to_bytes.b[2] = raw[1812];
         __convert_uint64_to_bytes.b[3] = raw[1813];
         __convert_uint64_to_bytes.b[4] = raw[1814];
         __convert_uint64_to_bytes.b[5] = raw[1815];
         __convert_uint64_to_bytes.b[6] = raw[1816];
         __convert_uint64_to_bytes.b[7] = raw[1817];
         data.tracks[9].aircraft_state.toa = __convert_uint64_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.longitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1818];
         __convert_float64_to_bytes.b[1] = raw[1819];
         __convert_float64_to_bytes.b[2] = raw[1820];
         __convert_float64_to_bytes.b[3] = raw[1821];
         __convert_float64_to_bytes.b[4] = raw[1822];
         __convert_float64_to_bytes.b[5] = raw[1823];
         __convert_float64_to_bytes.b[6] = raw[1824];
         __convert_float64_to_bytes.b[7] = raw[1825];
         data.tracks[9].aircraft_state.longitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.latitude        float64        8 bytes
         __convert_float64_to_bytes.b[0] = raw[1826];
         __convert_float64_to_bytes.b[1] = raw[1827];
         __convert_float64_to_bytes.b[2] = raw[1828];
         __convert_float64_to_bytes.b[3] = raw[1829];
         __convert_float64_to_bytes.b[4] = raw[1830];
         __convert_float64_to_bytes.b[5] = raw[1831];
         __convert_float64_to_bytes.b[6] = raw[1832];
         __convert_float64_to_bytes.b[7] = raw[1833];
         data.tracks[9].aircraft_state.latitude = __convert_float64_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.geometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1834];
         __convert_float32_to_bytes.b[1] = raw[1835];
         __convert_float32_to_bytes.b[2] = raw[1836];
         __convert_float32_to_bytes.b[3] = raw[1837];
         data.tracks[9].aircraft_state.geometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.barometric_altitude        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1838];
         __convert_float32_to_bytes.b[1] = raw[1839];
         __convert_float32_to_bytes.b[2] = raw[1840];
         __convert_float32_to_bytes.b[3] = raw[1841];
         data.tracks[9].aircraft_state.barometric_altitude = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.true_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1842];
         __convert_float32_to_bytes.b[1] = raw[1843];
         __convert_float32_to_bytes.b[2] = raw[1844];
         __convert_float32_to_bytes.b[3] = raw[1845];
         data.tracks[9].aircraft_state.true_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.magnetic_heading        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1846];
         __convert_float32_to_bytes.b[1] = raw[1847];
         __convert_float32_to_bytes.b[2] = raw[1848];
         __convert_float32_to_bytes.b[3] = raw[1849];
         data.tracks[9].aircraft_state.magnetic_heading = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.ground_track_angle        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1850];
         __convert_float32_to_bytes.b[1] = raw[1851];
         __convert_float32_to_bytes.b[2] = raw[1852];
         __convert_float32_to_bytes.b[3] = raw[1853];
         data.tracks[9].aircraft_state.ground_track_angle = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.true_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1854];
         __convert_float32_to_bytes.b[1] = raw[1855];
         __convert_float32_to_bytes.b[2] = raw[1856];
         __convert_float32_to_bytes.b[3] = raw[1857];
         data.tracks[9].aircraft_state.true_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.indicated_air_speed        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1858];
         __convert_float32_to_bytes.b[1] = raw[1859];
         __convert_float32_to_bytes.b[2] = raw[1860];
         __convert_float32_to_bytes.b[3] = raw[1861];
         data.tracks[9].aircraft_state.indicated_air_speed = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.ground_speed_north        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1862];
         __convert_float32_to_bytes.b[1] = raw[1863];
         __convert_float32_to_bytes.b[2] = raw[1864];
         __convert_float32_to_bytes.b[3] = raw[1865];
         data.tracks[9].aircraft_state.ground_speed_north = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.ground_speed_east        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1866];
         __convert_float32_to_bytes.b[1] = raw[1867];
         __convert_float32_to_bytes.b[2] = raw[1868];
         __convert_float32_to_bytes.b[3] = raw[1869];
         data.tracks[9].aircraft_state.ground_speed_east = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.geometric_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1870];
         __convert_float32_to_bytes.b[1] = raw[1871];
         __convert_float32_to_bytes.b[2] = raw[1872];
         __convert_float32_to_bytes.b[3] = raw[1873];
         data.tracks[9].aircraft_state.geometric_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.pressure_vertical_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1874];
         __convert_float32_to_bytes.b[1] = raw[1875];
         __convert_float32_to_bytes.b[2] = raw[1876];
         __convert_float32_to_bytes.b[3] = raw[1877];
         data.tracks[9].aircraft_state.pressure_vertical_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.heading_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1878];
         __convert_float32_to_bytes.b[1] = raw[1879];
         __convert_float32_to_bytes.b[2] = raw[1880];
         __convert_float32_to_bytes.b[3] = raw[1881];
         data.tracks[9].aircraft_state.heading_rate = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.ground_track_rate        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1882];
         __convert_float32_to_bytes.b[1] = raw[1883];
         __convert_float32_to_bytes.b[2] = raw[1884];
         __convert_float32_to_bytes.b[3] = raw[1885];
         data.tracks[9].aircraft_state.ground_track_rate = __convert_float32_to_bytes.a;
      } 
      { //READ enum data.tracks[9].aircraft_state.mode
         union {
            uint32_t a;
            char b[4];
         } u;
         u.b[0] = raw[1886];
         u.b[1] = raw[1887];
         u.b[2] = raw[1888];
         u.b[3] = raw[1889];
        switch(u.a) {
         case 0:
            data.tracks[9].aircraft_state.mode = flight_mode::AT_GATE_HANGAR;
            break;
         case 1:
            data.tracks[9].aircraft_state.mode = flight_mode::TAXI;
            break;
         case 2:
            data.tracks[9].aircraft_state.mode = flight_mode::HOLD;
            break;
         case 3:
            data.tracks[9].aircraft_state.mode = flight_mode::TAKE_OFF;
            break;
         case 4:
            data.tracks[9].aircraft_state.mode = flight_mode::CLIMB;
            break;
         case 5:
            data.tracks[9].aircraft_state.mode = flight_mode::CRUISE;
            break;
         case 6:
            data.tracks[9].aircraft_state.mode = flight_mode::DESCENT;
            break;
         case 7:
            data.tracks[9].aircraft_state.mode = flight_mode::LAND;
            break;
         case 8: //enum_end
            data.tracks[9].aircraft_state.mode = flight_mode::flight_mode_end;
            break;
        }
      } 
      { //READ data.tracks[9].aircraft_state.horizonal_position_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1890];
         __convert_uint16_to_bytes.b[1] = raw[1891];
         data.tracks[9].aircraft_state.horizonal_position_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.pressure_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1892];
         __convert_uint16_to_bytes.b[1] = raw[1893];
         data.tracks[9].aircraft_state.pressure_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.geometric_altitude_uncertainty        uint16        2 bytes
         __convert_uint16_to_bytes.b[0] = raw[1894];
         __convert_uint16_to_bytes.b[1] = raw[1895];
         data.tracks[9].aircraft_state.geometric_altitude_uncertainty = __convert_uint16_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.ground_speed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1896];
         __convert_float32_to_bytes.b[1] = raw[1897];
         __convert_float32_to_bytes.b[2] = raw[1898];
         __convert_float32_to_bytes.b[3] = raw[1899];
         data.tracks[9].aircraft_state.ground_speed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.airspeed_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1900];
         __convert_float32_to_bytes.b[1] = raw[1901];
         __convert_float32_to_bytes.b[2] = raw[1902];
         __convert_float32_to_bytes.b[3] = raw[1903];
         data.tracks[9].aircraft_state.airspeed_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.heading_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1904];
         __convert_float32_to_bytes.b[1] = raw[1905];
         __convert_float32_to_bytes.b[2] = raw[1906];
         __convert_float32_to_bytes.b[3] = raw[1907];
         data.tracks[9].aircraft_state.heading_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.ground_track_angle_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1908];
         __convert_float32_to_bytes.b[1] = raw[1909];
         __convert_float32_to_bytes.b[2] = raw[1910];
         __convert_float32_to_bytes.b[3] = raw[1911];
         data.tracks[9].aircraft_state.ground_track_angle_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.heading_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1912];
         __convert_float32_to_bytes.b[1] = raw[1913];
         __convert_float32_to_bytes.b[2] = raw[1914];
         __convert_float32_to_bytes.b[3] = raw[1915];
         data.tracks[9].aircraft_state.heading_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.tracks[9].aircraft_state.ground_track_rate_uncertainty        float32        4 bytes
         __convert_float32_to_bytes.b[0] = raw[1916];
         __convert_float32_to_bytes.b[1] = raw[1917];
         __convert_float32_to_bytes.b[2] = raw[1918];
         __convert_float32_to_bytes.b[3] = raw[1919];
         data.tracks[9].aircraft_state.ground_track_rate_uncertainty = __convert_float32_to_bytes.a;
      } 
      { //READ data.trackCount        uint8        1 bytes
         __convert_uint8_to_bytes.b[0] = raw[1920];
         data.trackCount = __convert_uint8_to_bytes.a;
      } 
   }

}
