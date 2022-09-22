// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RtklibPosConverter.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/State/PosVel.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"

NAV::RtklibPosConverter::RtklibPosConverter()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = false;

    nm::CreateOutputPin(this, "PosVel", Pin::Type::Flow, { NAV::PosVel::type() });

    nm::CreateInputPin(this, "RtklibPosObs", Pin::Type::Flow, { NAV::RtklibPosObs::type() }, &RtklibPosConverter::receiveObs);
}

NAV::RtklibPosConverter::~RtklibPosConverter()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::RtklibPosConverter::typeStatic()
{
    return "RtklibPosConverter";
}

std::string NAV::RtklibPosConverter::type() const
{
    return typeStatic();
}

std::string NAV::RtklibPosConverter::category()
{
    return "Converter";
}

bool NAV::RtklibPosConverter::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::RtklibPosConverter::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto rtklibPosObs = std::static_pointer_cast<const RtklibPosObs>(queue.extract_front());

    auto posVelObs = std::make_shared<PosVel>();

    posVelObs->insTime = rtklibPosObs->insTime;
    posVelObs->setPosition_e(rtklibPosObs->e_position());
    posVelObs->setVelocity_e(rtklibPosObs->e_velocity());

    invokeCallbacks(OUTPUT_PORT_INDEX_POSVEL, posVelObs);
}