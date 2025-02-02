// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SinglePointPositioning.hpp"

#include <algorithm>
#include <unordered_set>
#include <ranges>

#include "util/Logger.hpp"
#include "util/Container/Vector.hpp"

#include "Navigation/Constants.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/SppSolution.hpp"

#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/Math/LeastSquares.hpp"

NAV::SinglePointPositioning::SinglePointPositioning()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 407, 506 };

    nm::CreateInputPin(this, NAV::GnssObs::type().c_str(), Pin::Type::Flow, { NAV::GnssObs::type() }, &SinglePointPositioning::recvGnssObs);
    updateNumberOfInputPins();

    nm::CreateOutputPin(this, NAV::SppSolution::type().c_str(), Pin::Type::Flow, { NAV::SppSolution::type() });
}

NAV::SinglePointPositioning::~SinglePointPositioning()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SinglePointPositioning::typeStatic()
{
    return "SinglePointPositioning - SPP";
}

std::string NAV::SinglePointPositioning::type() const
{
    return typeStatic();
}

std::string NAV::SinglePointPositioning::category()
{
    return "Data Processor";
}

void NAV::SinglePointPositioning::guiConfig()
{
    if (ImGui::BeginTable(fmt::format("Pin Settings##{}", size_t(id)).c_str(), inputPins.size() > 1 ? 3 : 2,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("Pin");
        ImGui::TableSetupColumn("# Sat");
        if (inputPins.size() > 3)
        {
            ImGui::TableSetupColumn("");
        }
        ImGui::TableHeadersRow();

        // Used to reset the member variabel _dragAndDropPinIndex in case no plot does a drag and drop action
        bool dragAndDropPinStillInProgress = false;

        auto showDragDropTargetPin = [this](size_t pinIdxTarget) {
            ImGui::Dummy(ImVec2(-1.F, 2.F));

            bool selectableDummy = true;
            ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.5F, 0.5F));
            ImGui::PushStyleColor(ImGuiCol_Header, IM_COL32(16, 173, 44, 79));
            ImGui::Selectable(fmt::format("[drop here]").c_str(), &selectableDummy, ImGuiSelectableFlags_None,
                              ImVec2(std::max(ImGui::GetColumnWidth(0), ImGui::CalcTextSize("[drop here]").x), 20.F));
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();

            if (ImGui::BeginDragDropTarget())
            {
                if (const ImGuiPayload* payloadData = ImGui::AcceptDragDropPayload(fmt::format("DND Pin {}", size_t(id)).c_str()))
                {
                    auto pinIdxSource = *static_cast<size_t*>(payloadData->Data);

                    if (pinIdxSource < pinIdxTarget)
                    {
                        --pinIdxTarget;
                    }

                    move(inputPins, pinIdxSource, pinIdxTarget);
                    flow::ApplyChanges();
                }
                ImGui::EndDragDropTarget();
            }
            ImGui::Dummy(ImVec2(-1.F, 2.F));
        };

        for (size_t pinIndex = 0; pinIndex < inputPins.size(); pinIndex++)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); // Pin

            if (pinIndex == INPUT_PORT_INDEX_GNSS_NAV_INFO && _dragAndDropPinIndex > static_cast<int>(INPUT_PORT_INDEX_GNSS_NAV_INFO))
            {
                showDragDropTargetPin(INPUT_PORT_INDEX_GNSS_NAV_INFO);
            }

            bool selectablePinDummy = false;
            ImGui::Selectable(fmt::format("{}##{}", inputPins.at(pinIndex).name, size_t(id)).c_str(), &selectablePinDummy);
            if (pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO && ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
            {
                dragAndDropPinStillInProgress = true;
                _dragAndDropPinIndex = static_cast<int>(pinIndex);
                // Data is copied into heap inside the drag and drop
                ImGui::SetDragDropPayload(fmt::format("DND Pin {}", size_t(id)).c_str(), &pinIndex, sizeof(pinIndex));
                ImGui::TextUnformatted(inputPins.at(pinIndex).name.c_str());
                ImGui::EndDragDropSource();
            }
            if (_dragAndDropPinIndex > 0 && pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO
                && pinIndex != static_cast<size_t>(_dragAndDropPinIndex - 1)
                && pinIndex != static_cast<size_t>(_dragAndDropPinIndex))
            {
                showDragDropTargetPin(pinIndex + 1);
            }
            if (pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO && ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("This item can be dragged to reorder the pins");
            }

            ImGui::TableNextColumn(); // # Sat
            if (const auto* gnssNavInfo = getInputValue<const GnssNavInfo>(pinIndex))
            {
                size_t usedSatNum = 0;
                std::string usedSats;
                std::string allSats;

                std::string filler = ", ";
                for (const auto& satellite : gnssNavInfo->satellites())
                {
                    if ((satellite.first.satSys & _filterFreq)
                        && std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satellite.first) == _excludedSatellites.end())
                    {
                        usedSatNum++;
                        usedSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
                    }
                    allSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
                }
                ImGui::TextUnformatted(fmt::format("{} / {}", usedSatNum, gnssNavInfo->nSatellites()).c_str());
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Used satellites: %s\n"
                                      "All  satellites: %s",
                                      usedSats.c_str(), allSats.c_str());
                }
            }

            if (inputPins.size() > 1)
            {
                ImGui::TableNextColumn(); // Delete
                if (ImGui::Button(fmt::format("x##{} - {}", size_t(id), pinIndex).c_str()))
                {
                    _nNavInfoPins--;
                    nm::DeleteInputPin(inputPins.at(pinIndex));
                    flow::ApplyChanges();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Delete the pin");
                }
            }
        }

        if (!dragAndDropPinStillInProgress)
        {
            _dragAndDropPinIndex = -1;
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); // Pin
        if (ImGui::Button(fmt::format("Add Pin##{}", size_t(id)).c_str()))
        {
            _nNavInfoPins++;
            LOG_DEBUG("{}: # Input Pins changed to {}", nameId(), _nNavInfoPins);
            flow::ApplyChanges();
            updateNumberOfInputPins();
        }

        ImGui::EndTable();
    }

    const float itemWidth = 250.0F * gui::NodeEditorApplication::windowFontRatio();

    ImGui::SetNextItemWidth(itemWidth);
    if (ShowFrequencySelector(fmt::format("Satellite Frequencies##{}", size_t(id)).c_str(), _filterFreq))
    {
        flow::ApplyChanges();
    }

    ImGui::SetNextItemWidth(itemWidth);
    if (ShowCodeSelector(fmt::format("Signal Codes##{}", size_t(id)).c_str(), _filterCode, _filterFreq))
    {
        flow::ApplyChanges();
    }

    ImGui::SetNextItemWidth(itemWidth);
    if (ShowSatelliteSelector(fmt::format("Excluded satellites##{}", size_t(id)).c_str(), _excludedSatellites))
    {
        flow::ApplyChanges();
    }

    double elevationMaskDeg = rad2deg(_elevationMask);
    ImGui::SetNextItemWidth(itemWidth);
    if (ImGui::InputDoubleL(fmt::format("Elevation mask##{}", size_t(id)).c_str(), &elevationMaskDeg, 0.0, 90.0, 5.0, 5.0, "%.1f°", ImGuiInputTextFlags_AllowTabInput))
    {
        _elevationMask = deg2rad(elevationMaskDeg);
        LOG_DEBUG("{}: Elevation mask changed to {}°", nameId(), elevationMaskDeg);
        flow::ApplyChanges();
    }

    ImGui::SetNextItemWidth(itemWidth);
    if (ImGui::Checkbox(fmt::format("Weighted LSE (position)##{}", size_t(id)).c_str(), &_useWeightedLeastSquares))
    {
        LOG_DEBUG("{}: Use weighted least squares changed to {}°", nameId(), _useWeightedLeastSquares);
        flow::ApplyChanges();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Compensation models##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(itemWidth - ImGui::GetStyle().IndentSpacing);
        if (ComboIonosphereModel(fmt::format("Ionosphere Model##{}", size_t(id)).c_str(), _ionosphereModel))
        {
            LOG_DEBUG("{}: Ionosphere Model changed to {}", nameId(), NAV::to_string(_ionosphereModel));
            flow::ApplyChanges();
        }
        if (ComboTroposphereModel(fmt::format("Troposphere Model##{}", size_t(id)).c_str(), _troposphereModels, itemWidth - ImGui::GetStyle().IndentSpacing))
        {
            flow::ApplyChanges();
        }
        ImGui::TreePop();
    }
}

[[nodiscard]] json NAV::SinglePointPositioning::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nNavInfoPins"] = _nNavInfoPins;
    j["frequencies"] = Frequency_(_filterFreq);
    j["codes"] = _filterCode;
    j["excludedSatellites"] = _excludedSatellites;
    j["elevationMask"] = rad2deg(_elevationMask);
    j["useWeightedLeastSquares"] = _useWeightedLeastSquares;
    j["ionosphereModel"] = _ionosphereModel;
    j["troposphereModels"] = _troposphereModels;

    return j;
}

void NAV::SinglePointPositioning::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("nNavInfoPins"))
    {
        j.at("nNavInfoPins").get_to(_nNavInfoPins);
        updateNumberOfInputPins();
    }
    if (j.contains("frequencies"))
    {
        uint64_t value = 0;
        j.at("frequencies").get_to(value);
        _filterFreq = Frequency_(value);
    }
    if (j.contains("codes"))
    {
        j.at("codes").get_to(_filterCode);
    }
    if (j.contains("excludedSatellites"))
    {
        j.at("excludedSatellites").get_to(_excludedSatellites);
    }
    if (j.contains("elevationMask"))
    {
        j.at("elevationMask").get_to(_elevationMask);
        _elevationMask = deg2rad(_elevationMask);
    }
    if (j.contains("useWeightedLeastSquares"))
    {
        j.at("useWeightedLeastSquares").get_to(_useWeightedLeastSquares);
    }
    if (j.contains("ionosphereModel"))
    {
        j.at("ionosphereModel").get_to(_ionosphereModel);
    }
    if (j.contains("troposphereModels"))
    {
        j.at("troposphereModels").get_to(_troposphereModels);
    }
}

bool NAV::SinglePointPositioning::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (std::all_of(inputPins.begin() + INPUT_PORT_INDEX_GNSS_NAV_INFO, inputPins.end(), [](const InputPin& inputPin) { return !inputPin.isPinLinked(); }))
    {
        LOG_ERROR("{}: You need to connect a GNSS NavigationInfo provider", nameId());
        return false;
    }

    _e_position = Eigen::Vector3d::Zero();
    _e_velocity = Eigen::Vector3d::Zero();
    _recvClk = {};

    LOG_DEBUG("SinglePointPositioning initialized");

    return true;
}

void NAV::SinglePointPositioning::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::SinglePointPositioning::updateNumberOfInputPins()
{
    while (inputPins.size() - INPUT_PORT_INDEX_GNSS_NAV_INFO < _nNavInfoPins)
    {
        nm::CreateInputPin(this, NAV::GnssNavInfo::type().c_str(), Pin::Type::Object, { NAV::GnssNavInfo::type() });
    }
    while (inputPins.size() - INPUT_PORT_INDEX_GNSS_NAV_INFO > _nNavInfoPins)
    {
        nm::DeleteInputPin(inputPins.back());
    }
}

void NAV::SinglePointPositioning::recvGnssObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Collection of all connected navigation data providers
    std::vector<const GnssNavInfo*> gnssNavInfos;
    for (size_t i = 0; i < _nNavInfoPins; i++)
    {
        if (const auto* gnssNavInfo = getInputValue<const GnssNavInfo>(INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
        {
            gnssNavInfos.push_back(gnssNavInfo);
        }
    }
    if (gnssNavInfos.empty()) { return; }

    // Collection of all connected Ionospheric Corrections
    IonosphericCorrections ionosphericCorrections;
    for (const auto* gnssNavInfo : gnssNavInfos)
    {
        for (const auto& correction : gnssNavInfo->ionosphericCorrections.data())
        {
            if (!ionosphericCorrections.contains(correction.satSys, correction.alphaBeta))
            {
                ionosphericCorrections.insert(correction.satSys, correction.alphaBeta, correction.data);
            }
        }
    }

    auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: Calculating SPP for [{}]", nameId(), gnssObs->insTime);

    auto sppSol
#ifdef TESTING
        = std::make_shared<SppSolutionExtended>();
#else
        = std::make_shared<SppSolution>();
#endif
    sppSol->insTime = gnssObs->insTime;

    // Data calculated for each observation
    struct CalcData
    {
        // Constructor
        explicit CalcData(const NAV::GnssObs::ObservationData& obsData, std::shared_ptr<NAV::SatNavData> satNavData)
            : obsData(obsData), satNavData(std::move(satNavData)) {}

        const NAV::GnssObs::ObservationData& obsData;          // GNSS Observation data
        std::shared_ptr<NAV::SatNavData> satNavData = nullptr; // Satellite Navigation data

        double satClkBias{};                    // Satellite clock bias [s]
        double satClkDrift{};                   // Satellite clock drift [s/s]
        Eigen::Vector3d e_satPos;               // Satellite position in ECEF frame coordinates [m]
        Eigen::Vector3d e_satVel;               // Satellite velocity in ECEF frame coordinates [m/s]
        double pseudorangeRate{ std::nan("") }; // Pseudorange rate [m/s]

        // Data recalculated each iteration

        bool skipped = false;                                            // Whether to skip the measurement
        Eigen::Vector3d e_lineOfSightUnitVector;                         // Line-of-sight unit vector in ECEF frame coordinates
        Eigen::Vector3d n_lineOfSightUnitVector;                         // Line-of-sight unit vector in NED frame coordinates
        double satElevation = calcSatElevation(n_lineOfSightUnitVector); // Elevation [rad]
        double satAzimuth = calcSatAzimuth(n_lineOfSightUnitVector);     // Azimuth [rad]
    };

    // Data calculated for each satellite (only satellites filtered by GUI filter & NAV data available)
    std::vector<CalcData> calcData;
    std::vector<SatelliteSystem> availSatelliteSystems; // List of satellite systems
    for (const auto& obsData : gnssObs->data)
    {
        auto satId = obsData.satSigId.toSatId();

        if ((obsData.satSigId.freq & _filterFreq)                                                                     // frequency is selected in GUI
            && (obsData.code & _filterCode)                                                                           // code is selected in GUI
            && obsData.pseudorange                                                                                    // has a valid pseudorange
            && std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satId) == _excludedSatellites.end()) // is not excluded
        {
            for (const auto& gnssNavInfo : gnssNavInfos)
            {
                if (auto satNavData = gnssNavInfo->searchNavigationData(satId, gnssObs->insTime)) // can calculate satellite position
                {
                    if (!satNavData->isHealthy())
                    {
                        LOG_DATA("{}: Satellite {} is skipped because the signal is not healthy.", nameId(), satId);

#ifdef TESTING
                        auto& sppExtendedData = (*sppSol)(obsData.satSigId.freq, obsData.satSigId.satNum, obsData.code);
                        sppExtendedData.skipped = true;
#endif
                        continue;
                    }
                    LOG_DATA("{}: Using observation from {} {}", nameId(), obsData.satSigId, obsData.code);
                    calcData.emplace_back(obsData, satNavData);
                    if (std::find(availSatelliteSystems.begin(), availSatelliteSystems.end(), satId.satSys) == availSatelliteSystems.end())
                    {
                        availSatelliteSystems.push_back(satId.satSys);
                    }
                    break;
                }
            }
        }
    }

    size_t nMeas = calcData.size();
    LOG_DATA("{}: nMeas {}", nameId(), nMeas);
    size_t nParam = 4 + availSatelliteSystems.size() - 1; // 3x pos, 1x clk, (N-1)x clkDiff

    // Frequency number (GLONASS only)
    int8_t freqNum = -128;

    // Find all observations providing a doppler measurement (for velocity calculation)
    size_t nDopplerMeas = 0;
    for (auto& calc : calcData)
    {
        const auto& obsData = calc.obsData;
        if (obsData.doppler)
        {
            nDopplerMeas++;
            // TODO: Find out what this is used for and find a way to use it, after the GLONASS orbit calculation is working
            if (obsData.satSigId.freq & (R01 | R02))
            {
                if (auto satNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(calc.satNavData))
                {
                    freqNum = satNavData->frequencyNumber;
                }
            }

            calc.pseudorangeRate = doppler2psrRate(obsData.doppler.value(), obsData.satSigId.freq, freqNum);
        }
    }

    // #####################################################################################################################################
    //                                                          Calculation
    // #####################################################################################################################################

    for (auto& calc : calcData) // Calculate satellite clock, position and velocity
    {
        const auto& obsData = calc.obsData;

        LOG_DATA("{}: satellite {}", nameId(), obsData.satSigId);
        LOG_DATA("{}:     pseudorange  {}", nameId(), obsData.pseudorange.value().value);

        auto satClk = calc.satNavData->calcClockCorrections(gnssObs->insTime, obsData.pseudorange.value().value, obsData.satSigId.freq);
        calc.satClkBias = satClk.bias;
        calc.satClkDrift = satClk.drift;
        LOG_DATA("{}:     satClkBias {}, satClkDrift {}", nameId(), calc.satClkBias, calc.satClkDrift);

        auto satPosVel = calc.satNavData->calcSatellitePosVel(satClk.transmitTime);
        calc.e_satPos = satPosVel.e_pos;
        calc.e_satVel = satPosVel.e_vel;
        LOG_DATA("{}:     e_satPos {}", nameId(), calc.e_satPos.transpose());
        LOG_DATA("{}:     e_satVel {}", nameId(), calc.e_satVel.transpose());

#ifdef TESTING
        auto& sppExtendedData = (*sppSol)(obsData.satSigId.freq, obsData.satSigId.satNum, obsData.code);
        sppExtendedData.transmitTime = satClk.transmitTime;
        sppExtendedData.satClkBias = satClk.bias;
        sppExtendedData.satClkDrift = satClk.drift;
        sppExtendedData.e_satPos = calc.e_satPos;
        sppExtendedData.e_satVel = calc.e_satVel;
#endif
    }

    if (nMeas < nParam)
    {
        LOG_ERROR("{}: [{}] Cannot calculate position because only {} valid measurements ({} needed). Try changing filter settings or reposition your antenna.",
                  nameId(), (gnssObs->insTime + std::chrono::seconds(gnssObs->insTime.leapGps2UTC())), nMeas, nParam);
        sppSol->nSatellitesPosition = nMeas;
        sppSol->nSatellitesVelocity = nDopplerMeas;
        invokeCallbacks(OUTPUT_PORT_INDEX_SPPSOL, sppSol);
        return;
    }

    // Measurement/Geometry matrix for the pseudorange
    Eigen::MatrixXd e_H_psr = Eigen::MatrixXd::Zero(static_cast<int>(nMeas), static_cast<int>(nParam));
    // Pseudorange estimates [m]
    Eigen::VectorXd psrEst = Eigen::VectorXd::Zero(static_cast<int>(nMeas));
    // Pseudorange measurements [m]
    Eigen::VectorXd psrMeas = Eigen::VectorXd::Zero(static_cast<int>(nMeas));
    // Pseudorange measurement error weight matrix
    Eigen::MatrixXd W_psr = Eigen::MatrixXd::Zero(static_cast<int>(nMeas), static_cast<int>(nMeas));

    // Measurement/Geometry matrix for the pseudorange-rate
    Eigen::MatrixXd e_H_r = Eigen::MatrixXd::Zero(static_cast<int>(nDopplerMeas), static_cast<int>(nParam));
    // Corrected pseudorange-rate estimates [m/s]
    Eigen::VectorXd psrRateEst = Eigen::VectorXd::Zero(static_cast<int>(nDopplerMeas));
    // Corrected pseudorange-rate measurements [m/s]
    Eigen::VectorXd psrRateMeas = Eigen::VectorXd::Zero(static_cast<int>(nDopplerMeas));
    // Pseudorange rate (doppler) measurement error weight matrix
    Eigen::MatrixXd W_psrRate = Eigen::MatrixXd::Zero(static_cast<int>(nDopplerMeas), static_cast<int>(nDopplerMeas));

    for (size_t o = 0; o < 10; o++)
    {
        // Keeps track of skipped meausrements (because of elevation mask, ...)
        size_t cntSkippedMeas = 0;

        LOG_DATA("{}: Iteration {}", nameId(), o);
        // Latitude, Longitude, Altitude of the receiver [rad, rad, m]
        Eigen::Vector3d lla_pos = trafo::ecef2lla_WGS84(_e_position);
        LOG_DATA("{}:     [{}] _e_position {}, {}, {}", nameId(), o, _e_position.x(), _e_position.y(), _e_position.z());
        LOG_DATA("{}:     [{}] lla_pos {}°, {}°, {}m", nameId(), o, rad2deg(lla_pos.x()), rad2deg(lla_pos.y()), lla_pos.z());
        LOG_DATA("{}:     [{}] _recvClk.bias {}", nameId(), o, _recvClk.bias.value);
        LOG_DATA("{}:     [{}] _recvClk.drift {}", nameId(), o, _recvClk.drift.value);

        std::vector<SatelliteSystem> satelliteSystems = availSatelliteSystems; // List of satellite systems

        SatelliteSystem_ usedSatelliteSystems = SatSys_None;
        for (size_t i = 0; i < nMeas; i++)
        {
            const auto& obsData = calcData[i].obsData;
            LOG_DATA("{}:     [{}] satellite {}", nameId(), o, obsData.satSigId);
            auto satId = obsData.satSigId.toSatId();

            // Line-of-sight unit vector in ECEF frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
            calcData[i].e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(_e_position, calcData[i].e_satPos);
            LOG_DATA("{}:     [{}]     e_lineOfSightUnitVector {}", nameId(), o, calcData[i].e_lineOfSightUnitVector.transpose());
            // Line-of-sight unit vector in NED frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
            calcData[i].n_lineOfSightUnitVector = trafo::n_Quat_e(lla_pos(0), lla_pos(1)) * calcData[i].e_lineOfSightUnitVector;
            LOG_DATA("{}:     [{}]     n_lineOfSightUnitVector {}", nameId(), o, calcData[i].n_lineOfSightUnitVector.transpose());
            // Elevation [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
            calcData[i].satElevation = calcSatElevation(calcData[i].n_lineOfSightUnitVector);
            LOG_DATA("{}:     [{}]     satElevation {}°", nameId(), o, rad2deg(calcData[i].satElevation));
            // Azimuth [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
            calcData[i].satAzimuth = calcSatAzimuth(calcData[i].n_lineOfSightUnitVector);
            LOG_DATA("{}:     [{}]     satAzimuth {}°", nameId(), o, rad2deg(calcData[i].satAzimuth));

#ifdef TESTING
            auto& sppExtendedData = (*sppSol)(obsData.satSigId.freq, obsData.satSigId.satNum, obsData.code);
            sppExtendedData.satElevation = calcData[i].satElevation;
            sppExtendedData.satAzimuth = calcData[i].satAzimuth;
#endif

            if (!_e_position.isZero() && calcData[i].satElevation < _elevationMask) // Do not check elevation mask when not having a valid position
            {
                cntSkippedMeas++;
                calcData[i].skipped = true;
                LOG_DATA("{}:     [{}]     [{}] Measurement is skipped because of elevation {:.1f}° and mask of {}° ({} valid measurements remaining)",
                         nameId(), o, obsData.satSigId, rad2deg(calcData[i].satElevation), rad2deg(_elevationMask), nMeas - cntSkippedMeas);

                if (!(usedSatelliteSystems & satId.satSys)
                    && calcData.begin() + static_cast<int64_t>(i + 1) != calcData.end()                                         // This is the last satellite and the system did not appear before
                    && std::none_of(calcData.begin() + static_cast<int64_t>(i + 1), calcData.end(), [&](const CalcData& data) { // The satellite system has no satellites available anymore
                           return data.obsData.satSigId.toSatId().satSys == satId.satSys;
                       }))
                {
                    LOG_DEBUG("{}: The satellite system {} won't be used this iteration because no satellite complies with the elevation mask.",
                              nameId(), satId.satSys);
                    nParam--;
                    satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), satId.satSys));
                }

#ifdef TESTING
                sppExtendedData.elevationMaskTriggered = true;
#endif

                if (nMeas - cntSkippedMeas < nParam)
                {
                    LOG_ERROR("{}: [{}] Cannot calculate position because only {} valid measurements ({} needed). Try changing filter settings or reposition your antenna.",
                              nameId(), gnssObs->insTime, nMeas - cntSkippedMeas, nParam);
                    sppSol->nSatellitesPosition = nMeas - cntSkippedMeas;
                    sppSol->nSatellitesVelocity = nDopplerMeas - cntSkippedMeas;
                    invokeCallbacks(OUTPUT_PORT_INDEX_SPPSOL, sppSol);
                    return;
                }
                continue;
            }
#ifdef TESTING
            sppExtendedData.elevationMaskTriggered = false;
#endif
            usedSatelliteSystems |= satId.satSys;
        }

        size_t ix = 0;
        size_t iv = 0;
        _recvClk.referenceTimeSatelliteSystem = satelliteSystems.front();
        for (const auto& availSatSys : satelliteSystems)
        {
            if (SatelliteSystem_(availSatSys) < SatelliteSystem_(_recvClk.referenceTimeSatelliteSystem))
            {
                _recvClk.referenceTimeSatelliteSystem = availSatSys;
            }
        }
        satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), _recvClk.referenceTimeSatelliteSystem));
        LOG_DATA("{}:     [{}] _recvClk.referenceTimeSatelliteSystem {} ({} other time systems)", nameId(), o, _recvClk.referenceTimeSatelliteSystem, satelliteSystems.size());

        for (auto& calc : calcData)
        {
            if (calc.skipped) { continue; }

            const auto& obsData = calc.obsData;
            LOG_DATA("{}:     [{}] satellite {}", nameId(), o, obsData.satSigId);
            auto satId = obsData.satSigId.toSatId();

            // #############################################################################################################################
            //                                                    Position calculation
            // #############################################################################################################################

            // Pseudorange measurement [m] - Groves ch. 8.5.3, eq. 8.48, p. 342
            psrMeas(static_cast<int>(ix)) = obsData.pseudorange.value().value /* + (multipath and/or NLOS errors) + (tracking errors) */;
            LOG_DATA("{}:     [{}]     psrMeas({}) {}", nameId(), o, ix, psrMeas(static_cast<int>(ix)));
            // Estimated modulation ionosphere propagation error [m]
            double dpsr_I = calcIonosphericTimeDelay(static_cast<double>(gnssObs->insTime.toGPSweekTow().tow), obsData.satSigId.freq, lla_pos,
                                                     calc.satElevation, calc.satAzimuth, _ionosphereModel, &ionosphericCorrections)
                            * InsConst::C;
            LOG_DATA("{}:     [{}]     dpsr_I {} [m] (Estimated modulation ionosphere propagation error)", nameId(), o, dpsr_I);

            auto tropo = calcTroposphericDelayAndMapping(gnssObs->insTime, lla_pos, calc.satElevation, calc.satAzimuth, _troposphereModels);
            LOG_DATA("{}:     [{}]     ZHD {}", nameId(), o, tropo.ZHD);
            LOG_DATA("{}:     [{}]     ZWD {}", nameId(), o, tropo.ZWD);
            LOG_DATA("{}:     [{}]     zhdMappingFactor {}", nameId(), o, tropo.zhdMappingFactor);
            LOG_DATA("{}:     [{}]     zwdMappingFactor {}", nameId(), o, tropo.zwdMappingFactor);

            // Estimated modulation troposphere propagation error [m]
            double dpsr_T = tropo.ZHD * tropo.zhdMappingFactor + tropo.ZWD * tropo.zwdMappingFactor;
            LOG_DATA("{}:     [{}]     dpsr_T {} [m] (Estimated modulation troposphere propagation error)", nameId(), o, dpsr_T);

            // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
            e_H_psr.block<1, 3>(static_cast<int>(ix), 0) = -calc.e_lineOfSightUnitVector;
            e_H_psr(static_cast<int>(ix), 3) = 1;
            for (size_t s = 0; s < satelliteSystems.size(); s++)
            {
                if (satId.satSys != _recvClk.referenceTimeSatelliteSystem
                    && satId.satSys == satelliteSystems.at(s))
                {
                    e_H_psr(static_cast<int>(ix), 4 + static_cast<int>(s)) = 1;
                }
            }
            LOG_DATA("{}:     [{}]     e_H_psr.row({}) {}", nameId(), o, ix, e_H_psr.row(static_cast<int>(ix)));

            // Sagnac correction - Springer Handbook ch. 19.1.1, eq. 19.7, p. 562
            double dpsr_ie = 1.0 / InsConst::C * (_e_position - calc.e_satPos).dot(InsConst::e_omega_ie.cross(_e_position));
            LOG_DATA("{}:     [{}]     dpsr_ie {}", nameId(), o, dpsr_ie);
            // Geometric distance [m]
            double geometricDist = (calc.e_satPos - _e_position).norm();
            LOG_DATA("{}:     [{}]     geometricDist {}", nameId(), o, geometricDist);
            // System time difference to GPS [s]
            double sysTimeDiff = satId.satSys != _recvClk.referenceTimeSatelliteSystem
                                     ? _recvClk.sysTimeDiff[satId.satSys].value
                                     : 0.0;

            // Pseudorange estimate [m]
            psrEst(static_cast<int>(ix)) = geometricDist
                                           + _recvClk.bias.value * InsConst::C
                                           + sysTimeDiff * InsConst::C
                                           - calc.satClkBias * InsConst::C
                                           + dpsr_I
                                           + dpsr_T
                                           + dpsr_ie;
            LOG_DATA("{}:     [{}]     psrEst({}) {}", nameId(), o, ix, psrEst(static_cast<int>(ix)));

            if (_useWeightedLeastSquares)
            {
                // Weight matrix - RTKLIB eq. E6.23, p. 158

                constexpr double ERR_BRDCI = 0.5;  // Broadcast iono model error factor (See GPS ICD ch. 20.3.3.5.2.5, p. 130: 50% reduction on RMS error)
                constexpr double ERR_SAAS = 0.3;   // Saastamoinen model error std [m] (maximum zenith wet delay - formulas with worst possible values)
                constexpr double ERR_CBIAS = 0.3;  // Code bias error Std (m)
                constexpr double EFACT_GPS = 1.0;  // Satellite system error factor GPS/GAL/QZS/BeiDou
                constexpr double EFACT_GLO = 1.5;  // Satellite system error factor GLONASS/IRNSS
                constexpr double EFACT_SBAS = 3.0; // Satellite system error factor SBAS

                double satSysErrFactor = satId.satSys & (GLO | IRNSS)
                                             ? EFACT_GLO
                                             : (satId.satSys == SBAS
                                                    ? EFACT_SBAS
                                                    : EFACT_GPS);
                double ele = std::max(calc.satElevation, deg2rad(5));

                // Code/Carrier-Phase Error Ratio - Measurement error standard deviation
                std::unordered_map<Frequency, double> codeCarrierPhaseErrorRatio = { { G01, 300.0 },
                                                                                     { G02, 300.0 },
                                                                                     { G05, 300.0 } };
                double carrierPhaseErrorA = 0.003; // Carrier-Phase Error Factor a [m] - Measurement error standard deviation
                double carrierPhaseErrorB = 0.003; // Carrier-Phase Error Factor b [m] - Measurement error standard deviation

                double varPsrMeas = std::pow(satSysErrFactor, 2) * std::pow(codeCarrierPhaseErrorRatio.at(G01), 2)
                                    * (std::pow(carrierPhaseErrorA, 2) + std::pow(carrierPhaseErrorB, 2) / std::sin(ele));
                LOG_DATA("{}:     [{}]     varPsrMeas {}", nameId(), o, varPsrMeas);

                double varEph = calc.satNavData->calcSatellitePositionVariance();
                LOG_DATA("{}:     [{}]     varEph {}", nameId(), o, varEph);
                double varIono = ratioFreqSquared(obsData.satSigId.freq.getL1(), obsData.satSigId.freq, freqNum, freqNum)
                                 * std::pow(dpsr_I * ERR_BRDCI, 2);
                LOG_DATA("{}:     [{}]     varIono {}", nameId(), o, varIono);
                double varTrop = dpsr_T == 0.0 ? 0.0 : std::pow(ERR_SAAS / (std::sin(calc.satElevation) + 0.1), 2);
                LOG_DATA("{}:     [{}]     varTrop {}", nameId(), o, varTrop);
                double varBias = std::pow(ERR_CBIAS, 2);
                LOG_DATA("{}:     [{}]     varBias {}", nameId(), o, varBias);

                double varErrors = varPsrMeas + varEph + varIono + varTrop + varBias;
                LOG_DATA("{}:     [{}]     varErrors {}", nameId(), o, varErrors);

                W_psr(static_cast<int>(ix), static_cast<int>(ix)) = 1.0 / varErrors;
                LOG_DATA("{}:     [{}]     W_psr({},{}) {}", nameId(), o, ix, ix, W_psr(static_cast<int>(ix), static_cast<int>(ix)));
            }

            LOG_DATA("{}:     [{}]     dpsr({}) {}", nameId(), o, ix, psrMeas(static_cast<int>(ix)) - psrEst(static_cast<int>(ix)));

            // #############################################################################################################################
            //                                                    Velocity calculation
            // #############################################################################################################################

            if (nDopplerMeas - cntSkippedMeas >= nParam && !std::isnan(calc.pseudorangeRate))
            {
                // Measurement/Geometry matrix - Groves ch. 9.4.1, eq. 9.144, p. 412
                e_H_r.row(static_cast<int>(iv)) = e_H_psr.row(static_cast<int>(ix));

                // Pseudorange-rate measurement [m/s] - Groves ch. 8.5.3, eq. 8.48, p. 342
                psrRateMeas(static_cast<int>(iv)) = calc.pseudorangeRate /* + (multipath and/or NLOS errors) + (tracking errors) */;
                LOG_DATA("{}:     [{}]     psrRateMeas({}) {}", nameId(), o, iv, psrRateMeas(static_cast<int>(iv)));

                // Range-rate Sagnac correction - Groves ch. 8.5.3, eq. 8.46, p. 342
                double dpsr_dot_ie = InsConst::omega_ie / InsConst::C
                                     * (calc.e_satVel.y() * _e_position.x() + calc.e_satPos.y() * _e_velocity.x()
                                        - calc.e_satVel.x() * _e_position.y() - calc.e_satPos.x() * _e_velocity.y());
                LOG_DATA("{}:     [{}]     dpsr_dot_ie {}", nameId(), o, dpsr_dot_ie);
                // System time drift difference to GPS [s/s]
                double sysDriftDiff = satId.satSys != _recvClk.referenceTimeSatelliteSystem
                                          ? _recvClk.sysDriftDiff[satId.satSys].value
                                          : 0.0;

                // Pseudorange-rate estimate [m/s] - Groves ch. 9.4.1, eq. 9.142, p. 412 (Sagnac correction different sign)
                psrRateEst(static_cast<int>(iv)) = calc.e_lineOfSightUnitVector.transpose() * (calc.e_satVel - _e_velocity)
                                                   + _recvClk.drift.value * InsConst::C
                                                   + sysDriftDiff * InsConst::C
                                                   - calc.satClkDrift * InsConst::C
                                                   - dpsr_dot_ie;
                LOG_DATA("{}:     [{}]     psrRateEst({}) {}", nameId(), o, iv, psrRateEst(static_cast<int>(iv)));

                if (_useWeightedLeastSquares)
                {
                    // Weight matrix

                    double dopplerFrequency = 1; // Doppler Frequency error factor [Hz] - Measurement error standard deviation

                    double varDopMeas = std::pow(dopplerFrequency, 2);
                    LOG_DATA("{}:     [{}]     varDopMeas {}", nameId(), o, varDopMeas);

                    double varEph = calc.satNavData->calcSatellitePositionVariance();
                    LOG_DATA("{}:     [{}]     varEph {}", nameId(), o, varEph);

                    double varErrors = varDopMeas + varEph;
                    LOG_DATA("{}:     [{}]     varErrors {}", nameId(), o, varErrors);

                    W_psrRate(static_cast<int>(iv), static_cast<int>(iv)) = 1.0 / varErrors;
                    LOG_DATA("{}:     [{}]     W_psrRate({},{}) {}", nameId(), o, iv, iv, W_psrRate(static_cast<int>(iv), static_cast<int>(iv)));
                }

                iv++;
            }
#ifdef TESTING
            auto& sppExtendedData = (*sppSol)(obsData.satSigId.freq, obsData.satSigId.satNum, obsData.code);
            sppExtendedData.pseudorangeRate = calc.pseudorangeRate;
            sppExtendedData.dpsr_I = dpsr_I;
            sppExtendedData.dpsr_T = dpsr_T;
            sppExtendedData.geometricDist = geometricDist;
#endif

            ix++;
        }

        // #################################################################################################################################
        //                                                     Least squares solution
        // #################################################################################################################################

        // ---------------------------------------------------------- Position -------------------------------------------------------------
        LOG_DATA("{}:     [{}] e_H_psr \n{}", nameId(), o, e_H_psr.topRows(ix));
        if (_useWeightedLeastSquares)
        {
            LOG_DATA("{}:     [{}] W_psr \n{}", nameId(), o, W_psr.topLeftCorner(ix, ix));
        }
        LOG_DATA("{}:     [{}] psrMeas {}", nameId(), o, psrMeas.topRows(ix).transpose());
        LOG_DATA("{}:     [{}] psrEst {}", nameId(), o, psrEst.topRows(ix).transpose());

        // Difference between measured and estimated pseudorange
        Eigen::VectorXd dpsr = psrMeas.topRows(ix) - psrEst.topRows(ix);
        LOG_DATA("{}:     [{}] dpsr {}", nameId(), o, dpsr.transpose());

        LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> lsq;

        // [x, y, z, clkBias, sysTimeDiff...] - Groves ch. 9.4.1, eq. 9.141, p. 412
        if (_useWeightedLeastSquares)
        {
            lsq = solveWeightedLinearLeastSquaresUncertainties(e_H_psr.topRows(ix), W_psr.topLeftCorner(ix, ix), dpsr);
            LOG_DATA("{}:     [{}] dx (wlsq) {}, {}, {}, {}", nameId(), o, lsq.solution(0), lsq.solution(1), lsq.solution(2), lsq.solution(3));
            LOG_DATA("{}:     [{}] stdev_dx (wlsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
        }
        else
        {
            lsq = solveLinearLeastSquaresUncertainties(e_H_psr.topRows(ix), dpsr);
            LOG_DATA("{}:     [{}] dx (lsq) {}, {}, {}, {}", nameId(), o, lsq.solution(0), lsq.solution(1), lsq.solution(2), lsq.solution(3));
            LOG_DATA("{}:     [{}] stdev_dx (lsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
        }

        _e_position += lsq.solution.head<3>();
        _recvClk.bias.value += lsq.solution(3) / InsConst::C;
        for (size_t s = 0; s < satelliteSystems.size(); s++)
        {
            int idx = 4 + static_cast<int>(s);
            _recvClk.sysTimeDiff[satelliteSystems.at(s)].value += lsq.solution(idx) / InsConst::C;
            _recvClk.sysTimeDiff[satelliteSystems.at(s)].stdDev = std::sqrt(lsq.variance(idx, idx)) / InsConst::C;
        }

        sppSol->nSatellitesPosition = ix;
        if (ix > nParam) // Standard deviation can only be calculated with more measurements than estimated parameters
        {
            sppSol->setPositionAndStdDev_e(_e_position, lsq.variance.topLeftCorner<3, 3>().cwiseSqrt());
            _recvClk.bias.stdDev = std::sqrt(lsq.variance(3, 3)) / InsConst::C;
        }
        else
        {
            sppSol->setPosition_e(_e_position);
            _recvClk.bias.stdDev = std::nan("");
        }
        sppSol->recvClk.bias = _recvClk.bias;
        sppSol->recvClk.referenceTimeSatelliteSystem = _recvClk.referenceTimeSatelliteSystem;
        sppSol->recvClk.sysTimeDiff = _recvClk.sysTimeDiff;

        bool solInaccurate = lsq.solution.norm() > 1e-4;

        // ---------------------------------------------------------- Velocity -------------------------------------------------------------
        if (iv >= nParam)
        {
            LOG_DATA("{}:     [{}] e_H_r \n{}", nameId(), o, e_H_r.topRows(iv));
            if (_useWeightedLeastSquares)
            {
                LOG_DATA("{}:     [{}] W_psrRate \n{}", nameId(), o, W_psrRate.topLeftCorner(iv, iv));
            }
            LOG_DATA("{}:     [{}] psrRateMeas {}", nameId(), o, psrRateMeas.topRows(iv).transpose());
            LOG_DATA("{}:     [{}] psrRateEst {}", nameId(), o, psrRateEst.topRows(iv).transpose());

            // Difference between measured and estimated pseudorange rates
            Eigen::VectorXd dpsr_dot = psrRateMeas.topRows(iv) - psrRateEst.topRows(iv);
            LOG_DATA("{}:     [{}] dpsr_dot {}", nameId(), o, dpsr_dot.transpose());

            // [vx, vy, vz, clkDrift, sysDriftDiff...] - Groves ch. 9.4.1, eq. 9.141, p. 412
            if (_useWeightedLeastSquares)
            {
                lsq = solveWeightedLinearLeastSquaresUncertainties(e_H_r.topRows(iv), W_psrRate.topLeftCorner(iv, iv), dpsr_dot);
                LOG_DATA("{}:     [{}] dv (wlsq) {}", nameId(), o, lsq.solution.transpose());
                LOG_DATA("{}:     [{}] stdev_dv (wlsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
            }
            else
            {
                lsq = solveLinearLeastSquaresUncertainties(e_H_r.topRows(iv), dpsr_dot);
                LOG_DATA("{}:     [{}] dv (lsq) {}", nameId(), o, lsq.solution.transpose());
                LOG_DATA("{}:     [{}] stdev_dv (lsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
            }

            _e_velocity += lsq.solution.head<3>();
            _recvClk.drift.value += lsq.solution(3) / InsConst::C;
            for (size_t s = 0; s < satelliteSystems.size(); s++)
            {
                int idx = 4 + static_cast<int>(s);
                _recvClk.sysDriftDiff[satelliteSystems.at(s)].value += lsq.solution(idx) / InsConst::C;
                _recvClk.sysDriftDiff[satelliteSystems.at(s)].stdDev = std::sqrt(lsq.variance(idx, idx)) / InsConst::C;
            }

            sppSol->nSatellitesVelocity = iv;
            if (iv > nParam) // Standard deviation can only be calculated with more measurements than estimated parameters
            {
                sppSol->setVelocityAndStdDev_e(_e_velocity, lsq.variance.topLeftCorner<3, 3>().cwiseSqrt());
                _recvClk.drift.stdDev = std::sqrt(lsq.variance(3, 3)) / InsConst::C;
            }
            else
            {
                sppSol->setVelocity_e(_e_velocity);
                _recvClk.drift.stdDev = std::nan("");
            }
            sppSol->recvClk.drift = _recvClk.drift;
            sppSol->recvClk.sysDriftDiff = _recvClk.sysDriftDiff;

            solInaccurate |= lsq.solution.norm() > 1e-4;
        }
        else
        {
            LOG_WARN("{}: [{}] Cannot calculate velocity because only {} valid doppler measurements ({} needed). Try changing filter settings or reposition your antenna.",
                     nameId(), gnssObs->insTime, iv, nParam);
            continue;
        }

        if (!solInaccurate)
        {
            break;
        }
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_SPPSOL, sppSol);
}