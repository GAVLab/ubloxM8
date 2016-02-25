/*
* Copyright (C) 2016 Swift Navigation Inc.
* Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
*
* This source is subject to the license found in the file 'LICENSE'
* which must be be distributed together with this source. All other
* rights reserved.
*
* THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
* PURPOSE.
*/

#ifndef NOVATELPYTHON_H
#define NOVATELPYTHON_H

#include "ublox_m8.h"

#include <boost/python.hpp>
using namespace boost::python;

namespace ublox_m8 {

#define SIZEOF_ARRAY(a) (sizeof(a) / sizeof(a[0]))

class UbloxM8Python : public UbloxM8
{
public:
  UbloxM8Python(std::string id);
  UbloxM8Python(std::string id, std::string raw_log_name);
  ~UbloxM8Python();

  template<typename T>
  static boost::python::tuple arr2tuple(T *arr, uint16_t len) {
    boost::python::list ret;
    for (uint8_t i = 0; i < len; i++) {
      ret.append(arr[i]);
    }
    return boost::python::tuple(ret);
  }

  PyObject * python_set_rxm_rawx_callback(PyObject *callback);
  PyObject * python_set_rxm_sfrbx_callback(PyObject *callback);
  PyObject * python_set_nav_posllh_callback(PyObject *callback);
  PyObject * python_set_nav_posecef_callback(PyObject *callback);
  PyObject * python_set_nav_velned_callback(PyObject *callback);
  PyObject * python_set_nav_velecef_callback(PyObject *callback);
  PyObject * python_set_nav_svinfo_callback(PyObject *callback);

  void rxm_rawx_callback(RxmRawX const &rawx, double const &timestamp);
  void rxm_sfrbx_callback(RxmSfrbX const &sfrb,
    double const &timestamp);
  void nav_posllh_callback(NavPosLLH const &pos,
    double const &timestamp);
  void nav_posecef_callback(NavPosECEF const &pos,
    double const &timestamp);
  void nav_velned_callback(NavVelNED const &vel,
    double const &timestamp);
  void nav_velecef_callback(NavVelECEF const &vel,
    double const &timestamp);
  void nav_svinfo_callback(NavSVInfo const &info,
    double const &timestamp);

private:
  std::string m_id;

  PyObject *python_rxm_rawx_callback_;
  PyObject *python_rxm_sfrbx_callback_;
  PyObject *python_nav_posllh_callback_;
  PyObject *python_nav_posecef_callback_;
  PyObject *python_nav_velned_callback_;
  PyObject *python_nav_velecef_callback_;
  PyObject *python_nav_svinfo_callback_;

  template<typename T>
  void call_python(PyObject *callable, T const &data,
    double const &timestamp);
};

class RxmRawXPython {
public:
  RxmRawXPython(RxmRawX const &rawx) {m_rawx = rawx;}
  UbloxHeader header() {return m_rawx.header;}
  double rcvTow() {return m_rawx.rcvTow;}
  uint16_t week() {return m_rawx.week;}
  int8_t leapS() {return m_rawx.leapS;}
  uint8_t numMeas() {return m_rawx.numMeas;}
  uint8_t recStat() {return m_rawx.recStat;}
  boost::python::tuple repeated_block() {
    return UbloxM8Python::arr2tuple(m_rawx.repeated_block, numMeas());
  }
  boost::python::tuple checksum() {
    return UbloxM8Python::arr2tuple(m_rawx.checksum,
      sizeof(m_rawx.checksum));
  }
private:
  RxmRawX m_rawx;
};

class RxmSfrbXPython {
public:
  RxmSfrbXPython(RxmSfrbX const &sfrbx) {m_sfrbx = sfrbx;}
  UbloxHeader header() {return m_sfrbx.header;}
  uint8_t gnssId() {return m_sfrbx.gnssId;}
  uint8_t svId() {return m_sfrbx.svId;}
  uint8_t reserved1() {return m_sfrbx.reserved1;}
  uint8_t freqId() {return m_sfrbx.freqId;}
  uint8_t numWords() {return m_sfrbx.numWords;}
  uint8_t reserved2() {return m_sfrbx.reserved2;}
  uint8_t version() {return m_sfrbx.version;}
  uint8_t reserved3() {return m_sfrbx.reserved3;}
  boost::python::tuple dwrds() {
    return UbloxM8Python::arr2tuple(m_sfrbx.dwrds, numWords());
  }
  boost::python::tuple checksum() {
    return UbloxM8Python::arr2tuple(m_sfrbx.checksum,
      sizeof(m_sfrbx.checksum));
  }
private:
  RxmSfrbX m_sfrbx;
};

class NavPosLLHPython {
public:
  NavPosLLHPython(NavPosLLH const &pos) {m_pos = pos;}
  UbloxHeader header() {return m_pos.header;}
  uint32_t iTOW() {return m_pos.iTOW;}
  int32_t longitude_scaled() {return m_pos.longitude_scaled;}
  int32_t latitude_scaled() {return m_pos.latitude_scaled;}
  int32_t height() {return m_pos.height;}
  int32_t height_mean_sea_level() {return m_pos.height_mean_sea_level;}
  uint32_t horizontal_accuracy() {return m_pos.horizontal_accuracy;}
  uint32_t vertical_accuracy() {return m_pos.vertical_accuracy;}
  boost::python::tuple checksum() {
    return UbloxM8Python::arr2tuple(m_pos.checksum,
      sizeof(m_pos.checksum));
  }
private:
  NavPosLLH m_pos;
};

class NavPosECEFPython {
public:
  NavPosECEFPython(NavPosECEF const &pos) {m_pos = pos;}
  UbloxHeader header() {return m_pos.header;}
  uint32_t iTOW() {return m_pos.iTOW;}
  int32_t ecefX() {return m_pos.ecefX;}
  int32_t ecefY() {return m_pos.ecefY;}
  int32_t ecefZ() {return m_pos.ecefZ;}
  int32_t pAcc() {return m_pos.pAcc;}
  boost::python::tuple checksum() {
    return UbloxM8Python::arr2tuple(m_pos.checksum,
      sizeof(m_pos.checksum));
  }
private:
  NavPosECEF m_pos;
};

class NavVelNEDPython {
public:
  NavVelNEDPython(NavVelNED const &vel) {m_vel = vel;}
  UbloxHeader header() {return m_vel.header;}
  uint32_t iTOW() {return m_vel.iTOW;}
  int32_t velocity_north() {return m_vel.velocity_north;}
  int32_t velocity_east() {return m_vel.velocity_east;}
  int32_t velocity_down() {return m_vel.velocity_down;}
  uint32_t speed() {return m_vel.speed;}
  uint32_t ground_speed() {return m_vel.ground_speed;}
  int32_t heading_scaled() {return m_vel.heading_scaled;}
  uint32_t speed_accuracy() {return m_vel.speed_accuracy;}
  uint32_t heading_accuracy() {return m_vel.heading_accuracy;}
  boost::python::tuple checksum() {
    return UbloxM8Python::arr2tuple(m_vel.checksum,
      sizeof(m_vel.checksum));
  }
private:
  NavVelNED m_vel;
};

class NavVelECEFPython {
public:
  NavVelECEFPython(NavVelECEF const &vel) {m_vel = vel;}
  UbloxHeader header() {return m_vel.header;}
  uint32_t iTOW() {return m_vel.iTOW;}
  int32_t ecefVX() {return m_vel.ecefVX;}
  int32_t ecefVY() {return m_vel.ecefVX;}
  int32_t ecefVZ() {return m_vel.ecefVX;}
  uint32_t sAcc() {return m_vel.sAcc;}
  boost::python::tuple checksum() {
    return UbloxM8Python::arr2tuple(m_vel.checksum,
      sizeof(m_vel.checksum));
  }
private:
  NavVelECEF m_vel;
};

class NavSVInfoPython {
public:
  NavSVInfoPython(NavSVInfo const &info) {m_info = info;}
  UbloxHeader header() {return m_info.header;}
  uint32_t iTOW() {return m_info.iTOW;}
  uint8_t numch() {return m_info.numch;}
  uint8_t global_flags() {return m_info.global_flags;}
  uint16_t reserved2() {return m_info.reserved2;}
  boost::python::tuple svinfo_repeated() {
    return UbloxM8Python::arr2tuple(m_info.svinfo_repeated, numch());
  }
  boost::python::tuple checksum() {
    return UbloxM8Python::arr2tuple(m_info.checksum,
      sizeof(m_info.checksum));
  }
private:
  NavSVInfo m_info;
};
}
#endif
