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

#include "ubloxM8/ublox_m8_python.h"

using namespace ublox_m8;

boost::posix_time::time_duration::tick_type MillisecondsSinceEpoch()
{
  return (boost::posix_time::microsec_clock::universal_time() -\
    boost::posix_time::ptime(boost::gregorian::date(1980, 1, 6))).\
    total_milliseconds();
}

UbloxM8Python::UbloxM8Python(std::string id)
  : UbloxM8()
  , m_id(id)
{
  set_time_handler(MillisecondsSinceEpoch);
}

UbloxM8Python::UbloxM8Python(std::string id, std::string raw_log_name)
  : UbloxM8()
  , m_id(id)
{
  set_time_handler(MillisecondsSinceEpoch);
  CreateRawLog(raw_log_name);
}

UbloxM8Python::~UbloxM8Python()
{
}

PyObject *
UbloxM8Python::python_set_rxm_rawx_callback(PyObject *callback)
{
  python_rxm_rawx_callback_ = callback;
  set_rxm_rawx_callback(
    boost::bind(&UbloxM8Python::rxm_rawx_callback, this, _1, _2));
  return Py_None;
}

PyObject *
UbloxM8Python::python_set_rxm_sfrbx_callback(PyObject *callback)
{
  python_rxm_sfrbx_callback_ = callback;
  set_rxm_sfrbx_callback(
    boost::bind(&UbloxM8Python::rxm_sfrbx_callback, this, _1, _2));
  return Py_None;
}

PyObject *
UbloxM8Python::python_set_nav_posllh_callback(PyObject *callback)
{
  python_nav_posllh_callback_ = callback;
  set_nav_pos_llh_callback(
    boost::bind(&UbloxM8Python::nav_posllh_callback, this, _1, _2));
  return Py_None;
}

PyObject *
UbloxM8Python::python_set_nav_posecef_callback(PyObject *callback)
{
  python_nav_posecef_callback_ = callback;
  set_nav_pos_ecef_callback(
    boost::bind(&UbloxM8Python::nav_posecef_callback, this, _1, _2));
  return Py_None;
}

PyObject *
UbloxM8Python::python_set_nav_velned_callback(PyObject *callback)
{
  python_nav_velned_callback_ = callback;
  set_nav_vel_ned_callback(
    boost::bind(&UbloxM8Python::nav_velned_callback, this, _1, _2));
  return Py_None;
}

PyObject *
UbloxM8Python::python_set_nav_velecef_callback(PyObject *callback)
{
  python_nav_velecef_callback_ = callback;
  set_nav_vel_ecef_callback(
    boost::bind(&UbloxM8Python::nav_velecef_callback, this, _1, _2));
  return Py_None;
}

PyObject *
UbloxM8Python::python_set_nav_svinfo_callback(PyObject *callback)
{
  python_nav_svinfo_callback_ = callback;
  set_nav_sv_info_callback(
    boost::bind(&UbloxM8Python::nav_svinfo_callback, this, _1, _2));
  return Py_None;
}

template<typename T>
void UbloxM8Python::call_python(PyObject *callable, T const &data,
  double const &timestamp)
{
  if (callable) {
    // Ensure that the current thread is ready to call the Python C API
    PyGILState_STATE state = PyGILState_Ensure();

    // invoke the python function
    try {
      boost::python::call<void>(callable, boost::ref(data), timestamp,
        m_id);
    }
    catch(error_already_set &) {
      PyObject *ptype, *pvalue, *ptraceback;
      PyErr_Fetch(&ptype, &pvalue, &ptraceback);
      //pvalue contains error message
      //ptraceback contains stack snapshot and many other information
      //(see python traceback structure)

      //Get error message
      std::cout << PyString_AsString(pvalue);
    }

    // release the global interpreter lock
    // so other threads can resume execution
    PyGILState_Release(state);
  }
}

void UbloxM8Python::rxm_rawx_callback(RxmRawX const &rawx,
  double const &timestamp)
{
  call_python(python_rxm_rawx_callback_, RxmRawXPython(rawx),
    timestamp);
}

void UbloxM8Python::rxm_sfrbx_callback(RxmSfrbX const &sfrbx,
  double const &timestamp)
{
  call_python(python_rxm_sfrbx_callback_, RxmSfrbXPython(sfrbx),
    timestamp);
}

void UbloxM8Python::nav_posllh_callback(NavPosLLH const &pos,
  double const &timestamp)
{
  call_python(python_nav_posllh_callback_, NavPosLLHPython(pos),
    timestamp);
}

void UbloxM8Python::nav_posecef_callback(NavPosECEF const &pos,
  double const &timestamp)
{
  call_python(python_nav_posecef_callback_, NavPosECEFPython(pos),
    timestamp);
}

void UbloxM8Python::nav_velned_callback(NavVelNED const &vel,
  double const &timestamp)
{
  call_python(python_nav_velned_callback_, NavVelNEDPython(vel),
    timestamp);
}

void UbloxM8Python::nav_velecef_callback(NavVelECEF const &vel,
  double const &timestamp)
{
  call_python(python_nav_velecef_callback_, NavVelECEFPython(vel),
    timestamp);
}

void UbloxM8Python::nav_svinfo_callback(NavSVInfo const &info,
  double const &timestamp)
{
  call_python(python_nav_svinfo_callback_, NavSVInfoPython(info),
    timestamp);
}

BOOST_PYTHON_MODULE(ublox_m8)
{
  class_<UbloxM8, boost::noncopyable>("UbloxM8")
    .def("Connect", &UbloxM8::Connect)
    .def("Disconnect", &UbloxM8::Disconnect)
    .def("SetCfgMsgRate", &UbloxM8::SetCfgMsgRate)
    .def("ReadFile", &UbloxM8::ReadFile)
  ;

  class_<UbloxM8Python, bases<UbloxM8>,
    boost::noncopyable>("UbloxM8Python", init<std::string>())
    .def(init<std::string, std::string>())
    .def("python_set_rxm_rawx_callback",
      &UbloxM8Python::python_set_rxm_rawx_callback)
    .def("python_set_rxm_sfrbx_callback",
      &UbloxM8Python::python_set_rxm_sfrbx_callback)
    .def("python_set_nav_posllh_callback",
      &UbloxM8Python::python_set_nav_posllh_callback)
    .def("python_set_nav_posecef_callback",
      &UbloxM8Python::python_set_nav_posecef_callback)
    .def("python_set_nav_velned_callback",
      &UbloxM8Python::python_set_nav_velned_callback)
    .def("python_set_nav_velecef_callback",
      &UbloxM8Python::python_set_nav_velecef_callback)
    .def("python_set_nav_svinfo_callback",
      &UbloxM8Python::python_set_nav_svinfo_callback)
  ;

  class_<UbloxHeader>("UbloxHeader")
    .def_readonly("sync1", &UbloxHeader::sync1)
    .def_readonly("sync2", &UbloxHeader::sync2)
    .def_readonly("message_class", &UbloxHeader::message_class)
    .def_readonly("message_id", &UbloxHeader::message_id)
    .def_readonly("payload_length", &UbloxHeader::payload_length)
  ;

  class_<RxmRawXRepeated>("RxmRawXRepeated")
    .def_readonly("prMes", &RxmRawXRepeated::prMes)
    .def_readonly("cpMes", &RxmRawXRepeated::cpMes)
    .def_readonly("doMes", &RxmRawXRepeated::doMes)
    .def_readonly("gnssId", &RxmRawXRepeated::gnssId)
    .def_readonly("svId", &RxmRawXRepeated::svId)
    .def_readonly("reserved2", &RxmRawXRepeated::reserved2)
    .def_readonly("freqId", &RxmRawXRepeated::freqId)
    .def_readonly("locktime", &RxmRawXRepeated::locktime)
    .def_readonly("cno", &RxmRawXRepeated::cno)
    .def_readonly("prStdev", &RxmRawXRepeated::prStdev)
    .def_readonly("cpStdev", &RxmRawXRepeated::cpStdev)
    .def_readonly("doStdev", &RxmRawXRepeated::doStdev)
    .def_readonly("trkStat", &RxmRawXRepeated::trkStat)
    .def_readonly("reserved3", &RxmRawXRepeated::reserved3)
  ;

  class_<RxmRawXPython>("RxmRawXPython", init<RxmRawX>())
    .add_property("header", &RxmRawXPython::header)
    .add_property("rcvTow", &RxmRawXPython::rcvTow)
    .add_property("week", &RxmRawXPython::week)
    .add_property("leapS", &RxmRawXPython::leapS)
    .add_property("numMeas", &RxmRawXPython::numMeas)
    .add_property("recStat", &RxmRawXPython::recStat)
    .add_property("repeated_block", &RxmRawXPython::repeated_block)
    .add_property("checksum", &RxmRawXPython::checksum)
  ;

  class_<RxmSfrbXPython>("RxmSfrbXPython", init<RxmSfrbX>())
    .add_property("header", &RxmSfrbXPython::header)
    .add_property("gnssId", &RxmSfrbXPython::gnssId)
    .add_property("svId", &RxmSfrbXPython::svId)
    .add_property("reserved1", &RxmSfrbXPython::reserved1)
    .add_property("freqId", &RxmSfrbXPython::freqId)
    .add_property("numWords", &RxmSfrbXPython::numWords)
    .add_property("reserved2", &RxmSfrbXPython::reserved2)
    .add_property("version", &RxmSfrbXPython::version)
    .add_property("reserved3", &RxmSfrbXPython::reserved3)
    .add_property("dwrds", &RxmSfrbXPython::dwrds)
    .add_property("checksum", &RxmSfrbXPython::checksum)
  ;

  class_<NavPosLLHPython>("NavPosLLHPython", init<NavPosLLH>())
    .add_property("header", &NavPosLLHPython::header)
    .add_property("iTOW", &NavPosLLHPython::iTOW)
    .add_property("longitude_scaled",
      &NavPosLLHPython::longitude_scaled)
    .add_property("latitude_scaled", &NavPosLLHPython::latitude_scaled)
    .add_property("height", &NavPosLLHPython::height)
    .add_property("height_mean_sea_level",
      &NavPosLLHPython::height_mean_sea_level)
    .add_property("horizontal_accuracy",
      &NavPosLLHPython::horizontal_accuracy)
    .add_property("vertical_accuracy",
      &NavPosLLHPython::vertical_accuracy)
    .add_property("checksum", &NavPosLLHPython::checksum)
  ;

  class_<NavPosECEFPython>("NavPosECEFPython", init<NavPosECEF>())
    .add_property("header", &NavPosECEFPython::header)
    .add_property("iTOW", &NavPosECEFPython::iTOW)
    .add_property("ecefX", &NavPosECEFPython::ecefX)
    .add_property("ecefY", &NavPosECEFPython::ecefY)
    .add_property("ecefZ", &NavPosECEFPython::ecefZ)
    .add_property("pAcc", &NavPosECEFPython::pAcc)
    .add_property("checksum", &NavPosECEFPython::checksum)
  ;

  class_<NavVelNEDPython>("NavVelNEDPython", init<NavVelNED>())
    .add_property("header", &NavVelNEDPython::header)
    .add_property("iTOW", &NavVelNEDPython::iTOW)
    .add_property("velocity_north", &NavVelNEDPython::velocity_north)
    .add_property("velocity_east", &NavVelNEDPython::velocity_east)
    .add_property("velocity_down", &NavVelNEDPython::velocity_down)
    .add_property("speed", &NavVelNEDPython::speed)
    .add_property("ground_speed", &NavVelNEDPython::ground_speed)
    .add_property("heading_scaled", &NavVelNEDPython::heading_scaled)
    .add_property("speed_accuracy", &NavVelNEDPython::speed_accuracy)
    .add_property("heading_accuracy",
      &NavVelNEDPython::heading_accuracy)
    .add_property("checksum", &NavVelNEDPython::checksum)
  ;

  class_<NavVelECEFPython>("NavVelECEFPython", init<NavVelECEF>())
    .add_property("header", &NavVelECEFPython::header)
    .add_property("iTOW", &NavVelECEFPython::iTOW)
    .add_property("ecefVX", &NavVelECEFPython::ecefVX)
    .add_property("ecefVY", &NavVelECEFPython::ecefVY)
    .add_property("ecefVZ", &NavVelECEFPython::ecefVZ)
    .add_property("sAcc", &NavVelECEFPython::sAcc)
    .add_property("checksum", &NavVelECEFPython::checksum)
  ;

  class_<SVInfoRepeatedBlock>("SVInfoRepeatedBlock")
    .def_readonly("ch_num", &SVInfoRepeatedBlock::ch_num)
    .def_readonly("svid", &SVInfoRepeatedBlock::svid)
    .def_readonly("flags", &SVInfoRepeatedBlock::flags)
    .def_readonly("quality", &SVInfoRepeatedBlock::quality)
    .def_readonly("cno", &SVInfoRepeatedBlock::cno)
    .def_readonly("elev", &SVInfoRepeatedBlock::elev)
    .def_readonly("azim", &SVInfoRepeatedBlock::azim)
    .def_readonly("prRes", &SVInfoRepeatedBlock::prRes)
  ;

  class_<NavSVInfoPython>("NavSVInfoPython", init<NavSVInfo>())
    .add_property("header", &NavSVInfoPython::header)
    .add_property("iTOW", &NavSVInfoPython::iTOW)
    .add_property("numch", &NavSVInfoPython::numch)
    .add_property("global_flags", &NavSVInfoPython::global_flags)
    .add_property("reserved2", &NavSVInfoPython::reserved2)
    .add_property("svinfo_repeated", &NavSVInfoPython::svinfo_repeated)
    .add_property("checksum", &NavSVInfoPython::checksum)
  ;
}
