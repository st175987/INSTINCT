// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SppSolutionLogger.hpp"

#include "Navigation/Transformations/Units.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/GNSS/SppSolution.hpp"

NAV::SppSolutionLogger::SppSolutionLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::ASCII;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { SppSolution::type() }, &SppSolutionLogger::writeObservation);
}

NAV::SppSolutionLogger::~SppSolutionLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SppSolutionLogger::typeStatic()
{
    return "SppSolutionLogger";
}

std::string NAV::SppSolutionLogger::type() const
{
    return typeStatic();
}

std::string NAV::SppSolutionLogger::category()
{
    return "Data Logger";
}

void NAV::SppSolutionLogger::guiConfig()
{
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
    }
}

[[nodiscard]] json NAV::SppSolutionLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::SppSolutionLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::SppSolutionLogger::flush()
{
    _filestream.flush();
}

bool NAV::SppSolutionLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    CommonLog::initialize();

    _filestream << "Time [s],GpsCycle,GpsWeek,GpsTow [s],"
                << "Pos ECEF X [m],Pos ECEF Y [m],Pos ECEF Z [m],Latitude [deg],Longitude [deg],Altitude [m],"
                << "North/South [m],East/West [m],"
                << "Vel ECEF X [m/s],Vel ECEF Y [m/s],Vel ECEF Z [m/s],Vel N [m/s],Vel E [m/s],Vel D [m/s],"
                << "Number satellites (pos),Number satellites (vel),"
                << "Receiver clock bias [s],"
                << "System time reference system,"
                << "GPS system time difference [s],"
                << "GAL system time difference [s],"
                << "GLO system time difference [s],"
                << "BDS system time difference [s],"
                << "QZSS system time difference [s],"
                << "IRNSS system time difference [s],"
                << "SBAS system time difference [s],"
                << "Receiver clock drift [s/s],"
                << "GPS System time drift difference [s/s],"
                << "GAL System time drift difference [s/s],"
                << "GLO System time drift difference [s/s],"
                << "BDS System time drift difference [s/s],"
                << "QZSS System time drift difference [s/s],"
                << "IRNSS System time drift difference [s/s],"
                << "SBAS System time drift difference [s/s],"
                << "X-ECEF StDev [m],Y-ECEF StDev [m],Z-ECEF StDev [m],"
                << "XY-ECEF StDev [m],XZ-ECEF StDev [m],YZ-ECEF StDev [m],"
                << "North StDev [m],East StDev [m],Down StDev [m],"
                << "NE-ECEF StDev [m],ND-ECEF StDev [m],ED-ECEF StDev [m],"
                << "X velocity ECEF StDev [m/s],Y velocity ECEF StDev [m/s],Z velocity ECEF StDev [m/s],"
                << "XY velocity StDev [m],XZ velocity StDev [m],YZ velocity StDev [m],"
                << "North velocity StDev [m/s],East velocity StDev [m/s],Down velocity StDev [m/s],"
                << "NE velocity StDev [m],ND velocity StDev [m],ED velocity StDev [m],"
                << "Receiver clock bias StDev [s],"
                << "GPS system time difference StDev [s],"
                << "GAL system time difference StDev [s],"
                << "GLO system time difference StDev [s],"
                << "BDS system time difference StDev [s],"
                << "QZSS system time difference StDev [s],"
                << "IRNSS system time difference StDev [s],"
                << "SBAS system time difference StDev [s],"
                << "Receiver clock drift StDev [s/s],"
                << "GPS system time drift difference StDev [s/s],"
                << "GAL system time drift difference StDev [s/s],"
                << "GLO system time drift difference StDev [s/s],"
                << "BDS system time drift difference StDev [s/s],"
                << "QZSS system time drift difference StDev [s/s],"
                << "IRNSS system time drift difference StDev [s/s],"
                << "SBAS system time drift difference StDev [s/s]"
                << std::endl;

    return true;
}

void NAV::SppSolutionLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::SppSolutionLogger::writeObservation(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const SppSolution>(queue.extract_front());

    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 12;

    if (!obs->insTime.empty())
    {
        _filestream << std::setprecision(valuePrecision) << std::round(calcTimeIntoRun(obs->insTime) * 1e9) / 1e9;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.toGPSweekTow().gpsCycle;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.toGPSweekTow().gpsWeek;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.toGPSweekTow().tow;
    }
    _filestream << "," << std::setprecision(valuePrecision);

    // -------------------------------------------------------- Position -----------------------------------------------------------

    if (!std::isnan(obs->e_position().x())) { _filestream << obs->e_position().x(); } // Pos ECEF X [m]
    _filestream << ",";
    if (!std::isnan(obs->e_position().y())) { _filestream << obs->e_position().y(); } // Pos ECEF Y [m]
    _filestream << ",";
    if (!std::isnan(obs->e_position().z())) { _filestream << obs->e_position().z(); } // Pos ECEF Z [m]
    _filestream << ",";
    if (!std::isnan(obs->lla_position().x())) { _filestream << rad2deg(obs->lla_position().x()); } // Latitude [deg]
    _filestream << ",";
    if (!std::isnan(obs->lla_position().y())) { _filestream << rad2deg(obs->lla_position().y()); } // Longitude [deg]
    _filestream << ",";
    if (!std::isnan(obs->lla_position().z())) { _filestream << obs->lla_position().z(); } // Altitude [m]
    _filestream << ",";
    if (!std::isnan(obs->lla_position().x()) && !std::isnan(obs->lla_position().y()))
    {
        auto localPosition = calcLocalPosition(obs->lla_position());
        _filestream << localPosition.northSouth << ","; // North/South [m]
        _filestream << localPosition.eastWest << ",";   // East/West [m]
    }
    else
    {
        _filestream << ",,";
    }

    // -------------------------------------------------------- Velocity -----------------------------------------------------------

    if (!std::isnan(obs->e_velocity().x())) { _filestream << obs->e_velocity().x(); } // Vel ECEF X [m/s]
    _filestream << ",";
    if (!std::isnan(obs->e_velocity().y())) { _filestream << obs->e_velocity().y(); } // Vel ECEF Y [m/s]
    _filestream << ",";
    if (!std::isnan(obs->e_velocity().z())) { _filestream << obs->e_velocity().z(); } // Vel ECEF Z [m/s]
    _filestream << ",";
    if (!std::isnan(obs->n_velocity().x())) { _filestream << obs->n_velocity().x(); } // Vel N [m/s]
    _filestream << ",";
    if (!std::isnan(obs->n_velocity().y())) { _filestream << obs->n_velocity().y(); } // Vel E [m/s]
    _filestream << ",";
    if (!std::isnan(obs->n_velocity().z())) { _filestream << obs->n_velocity().z(); } // Vel D [m/s]
    _filestream << ",";

    _filestream << obs->nSatellitesPosition << ","; // Number satellites (pos)
    _filestream << obs->nSatellitesVelocity << ","; // Number satellites (vel)

    if (!std::isnan(obs->recvClk.bias.value)) { _filestream << obs->recvClk.bias.value; } // Receiver clock bias [s]
    _filestream << ",";
    _filestream << obs->recvClk.referenceTimeSatelliteSystem; // System time reference system
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(GPS)) { _filestream << obs->recvClk.sysTimeDiff.at(GPS).value; } // GPS system time difference [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(GAL)) { _filestream << obs->recvClk.sysTimeDiff.at(GAL).value; } // GAL system time difference [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(GLO)) { _filestream << obs->recvClk.sysTimeDiff.at(GLO).value; } // GLO system time difference [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(BDS)) { _filestream << obs->recvClk.sysTimeDiff.at(BDS).value; } // BDS system time difference [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(QZSS)) { _filestream << obs->recvClk.sysTimeDiff.at(QZSS).value; } // QZSS system time difference [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(IRNSS)) { _filestream << obs->recvClk.sysTimeDiff.at(IRNSS).value; } // IRNSS system time difference [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(SBAS)) { _filestream << obs->recvClk.sysTimeDiff.at(SBAS).value; } // SBAS system time difference [s]
    _filestream << ",";

    if (!std::isnan(obs->recvClk.drift.value)) { _filestream << obs->recvClk.drift.value; } // Receiver clock drift [s/s]
    _filestream << ",";
    if (obs->recvClk.sysDriftDiff.contains(GPS)) { _filestream << obs->recvClk.sysDriftDiff.at(GPS).value; } // GPS system time drift difference [s]
    _filestream << ",";
    if (obs->recvClk.sysDriftDiff.contains(GAL)) { _filestream << obs->recvClk.sysDriftDiff.at(GAL).value; } // GAL system time drift difference [s]
    _filestream << ",";
    if (obs->recvClk.sysDriftDiff.contains(GLO)) { _filestream << obs->recvClk.sysDriftDiff.at(GLO).value; } // GLO system time drift difference [s]
    _filestream << ",";
    if (obs->recvClk.sysDriftDiff.contains(BDS)) { _filestream << obs->recvClk.sysDriftDiff.at(BDS).value; } // BDS system time drift difference [s]
    _filestream << ",";
    if (obs->recvClk.sysDriftDiff.contains(QZSS)) { _filestream << obs->recvClk.sysDriftDiff.at(QZSS).value; } // QZSS system time drift difference [s]
    _filestream << ",";
    if (obs->recvClk.sysDriftDiff.contains(IRNSS)) { _filestream << obs->recvClk.sysDriftDiff.at(IRNSS).value; } // IRNSS system time drift difference [s]
    _filestream << ",";
    if (obs->recvClk.sysDriftDiff.contains(SBAS)) { _filestream << obs->recvClk.sysDriftDiff.at(SBAS).value; } // SBAS system time drift difference [s]
    _filestream << ",";

    if (!std::isnan(obs->e_positionStdev()(0, 0))) { _filestream << obs->e_positionStdev()(0, 0); }; // X-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(1, 1))) { _filestream << obs->e_positionStdev()(1, 1); }; // Y-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(2, 2))) { _filestream << obs->e_positionStdev()(2, 2); }; // Z-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(0, 1))) { _filestream << obs->e_positionStdev()(0, 1); }; // XY-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(0, 2))) { _filestream << obs->e_positionStdev()(0, 2); }; // XZ-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(1, 2))) { _filestream << obs->e_positionStdev()(1, 2); }; // YZ-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(0, 0))) { _filestream << obs->n_positionStdev()(0, 0); }; // North StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(1, 1))) { _filestream << obs->n_positionStdev()(1, 1); }; // East StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(2, 2))) { _filestream << obs->n_positionStdev()(2, 2); }; // Down StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(0, 1))) { _filestream << obs->n_positionStdev()(0, 1); }; // NE-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(0, 2))) { _filestream << obs->n_positionStdev()(0, 2); }; // ND-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(1, 2))) { _filestream << obs->n_positionStdev()(1, 2); }; // ED-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(0, 0))) { _filestream << obs->e_velocityStdev()(0, 0); }; // X velocity ECEF StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(1, 1))) { _filestream << obs->e_velocityStdev()(1, 1); }; // Y velocity ECEF StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(2, 2))) { _filestream << obs->e_velocityStdev()(2, 2); }; // Z velocity ECEF StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(0, 1))) { _filestream << obs->e_velocityStdev()(0, 1); }; // XY velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(0, 2))) { _filestream << obs->e_velocityStdev()(0, 2); }; // XZ velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(1, 2))) { _filestream << obs->e_velocityStdev()(1, 2); }; // YZ velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(0, 0))) { _filestream << obs->n_velocityStdev()(0, 0); }; // North velocity StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(1, 1))) { _filestream << obs->n_velocityStdev()(1, 1); }; // East velocity StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(2, 2))) { _filestream << obs->n_velocityStdev()(2, 2); }; // Down velocity StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(0, 1))) { _filestream << obs->n_velocityStdev()(0, 1); }; // NE velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(0, 2))) { _filestream << obs->n_velocityStdev()(0, 2); }; // ND velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(1, 2))) { _filestream << obs->n_velocityStdev()(1, 2); }; // ED velocity StDev [m]
    _filestream << ",";

    if (!std::isnan(obs->recvClk.bias.stdDev)) { _filestream << obs->recvClk.bias.stdDev; } // Receiver clock bias StDev [s]
    _filestream << ",";

    if (obs->recvClk.sysTimeDiff.contains(GPS) && !std::isnan(obs->recvClk.sysTimeDiff.at(GPS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(GPS).stdDev; } // GPS system time difference StDev [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(GAL) && !std::isnan(obs->recvClk.sysTimeDiff.at(GAL).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(GAL).stdDev; } // GAL system time difference StDev [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(GLO) && !std::isnan(obs->recvClk.sysTimeDiff.at(GLO).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(GLO).stdDev; } // GLO system time difference StDev [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(BDS) && !std::isnan(obs->recvClk.sysTimeDiff.at(BDS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(BDS).stdDev; } // BDS system time difference StDev [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(QZSS) && !std::isnan(obs->recvClk.sysTimeDiff.at(QZSS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(QZSS).stdDev; } // QZSS system time difference StDev [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(IRNSS) && !std::isnan(obs->recvClk.sysTimeDiff.at(IRNSS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(IRNSS).stdDev; } // IRNSS system time difference StDev [s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(SBAS) && !std::isnan(obs->recvClk.sysTimeDiff.at(SBAS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(SBAS).stdDev; } // SBAS system time difference StDev [s]
    _filestream << ",";
    if (!std::isnan(obs->recvClk.drift.stdDev)) { _filestream << obs->recvClk.drift.stdDev; } // Receiver clock drift StDev [s/s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(GPS) && !std::isnan(obs->recvClk.sysTimeDiff.at(GPS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(GPS).stdDev; } // GPS system time drift difference StDev [s/s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(GAL) && !std::isnan(obs->recvClk.sysTimeDiff.at(GAL).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(GAL).stdDev; } // GAL system time drift difference StDev [s/s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(GLO) && !std::isnan(obs->recvClk.sysTimeDiff.at(GLO).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(GLO).stdDev; } // GLO system time drift difference StDev [s/s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(BDS) && !std::isnan(obs->recvClk.sysTimeDiff.at(BDS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(BDS).stdDev; } // BDS system time drift difference StDev [s/s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(QZSS) && !std::isnan(obs->recvClk.sysTimeDiff.at(QZSS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(QZSS).stdDev; } // QZSS system time drift difference StDev [s/s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(IRNSS) && !std::isnan(obs->recvClk.sysTimeDiff.at(IRNSS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(IRNSS).stdDev; } // IRNSS system time drift difference StDev [s/s]
    _filestream << ",";
    if (obs->recvClk.sysTimeDiff.contains(SBAS) && !std::isnan(obs->recvClk.sysTimeDiff.at(SBAS).stdDev)) { _filestream << obs->recvClk.sysTimeDiff.at(SBAS).stdDev; } // SBAS system time drift difference StDev [s/s]

    _filestream << '\n';
}