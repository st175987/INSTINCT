// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Frequency.hpp"

#include "util/Assert.h"
#include <cmath>
#include <imgui.h>
#include <imgui_internal.h>

namespace NAV
{

Frequency Frequency::fromString(const std::string& typeString)
{
    if (typeString == "B1")
    {
        return B01;
    }
    if (typeString == "B2")
    {
        return B08;
    }
    if (typeString == "B3")
    {
        return B06;
    }
    if (typeString == "B1-2")
    {
        return B02;
    }
    if (typeString == "B2a")
    {
        return B05;
    }
    if (typeString == "B2b")
    {
        return B07;
    }
    if (typeString == "E1")
    {
        return E01;
    }
    if (typeString == "E5a")
    {
        return E05;
    }
    if (typeString == "E6")
    {
        return E06;
    }
    if (typeString == "E5b")
    {
        return E07;
    }
    if (typeString == "E5")
    {
        return E08;
    }
    if (typeString == "L1")
    {
        return G01;
    }
    if (typeString == "L2")
    {
        return G02;
    }
    if (typeString == "L5")
    {
        return G05;
    }
    if (typeString == "I5")
    {
        return I05;
    }
    if (typeString == "IS")
    {
        return I09;
    }
    if (typeString == "Q1")
    {
        return J01;
    }
    if (typeString == "Q2")
    {
        return J02;
    }
    if (typeString == "Q5")
    {
        return J05;
    }
    if (typeString == "Q6" || typeString == "QLEX")
    {
        return J06;
    }
    if (typeString == "G1")
    {
        return R01;
    }
    if (typeString == "G2")
    {
        return R02;
    }
    if (typeString == "G3")
    {
        return R03;
    }
    if (typeString == "G1a")
    {
        return R04;
    }
    if (typeString == "G2a")
    {
        return R06;
    }
    if (typeString == "S1")
    {
        return S01;
    }
    if (typeString == "S5")
    {
        return S05;
    }

    return Freq_None;
}

Frequency::operator std::string() const
{
    const std::string filler = " | ";
    std::string str;
    if (value & G01)
    {
        str += (!str.empty() ? filler : "") + "L1";
    }
    if (value & G02)
    {
        str += (!str.empty() ? filler : "") + "L2";
    }
    if (value & G05)
    {
        str += (!str.empty() ? filler : "") + "L5";
    }
    if (value & E01)
    {
        str += (!str.empty() ? filler : "") + "E1";
    }
    if (value & E05)
    {
        str += (!str.empty() ? filler : "") + "E5a";
    }
    if (value & E06)
    {
        str += (!str.empty() ? filler : "") + "E6";
    }
    if (value & E07)
    {
        str += (!str.empty() ? filler : "") + "E5b";
    }
    if (value & E08)
    {
        str += (!str.empty() ? filler : "") + "E5";
    }
    if (value & R01)
    {
        str += (!str.empty() ? filler : "") + "G1";
    }
    if (value & R02)
    {
        str += (!str.empty() ? filler : "") + "G2";
    }
    if (value & R03)
    {
        str += (!str.empty() ? filler : "") + "G3";
    }
    if (value & R04)
    {
        str += (!str.empty() ? filler : "") + "G1a";
    }
    if (value & R06)
    {
        str += (!str.empty() ? filler : "") + "G2a";
    }
    if (value & B01)
    {
        str += (!str.empty() ? filler : "") + "B1";
    }
    if (value & B08)
    {
        str += (!str.empty() ? filler : "") + "B2";
    }
    if (value & B06)
    {
        str += (!str.empty() ? filler : "") + "B3";
    }
    if (value & B02)
    {
        str += (!str.empty() ? filler : "") + "B1-2";
    }
    if (value & B05)
    {
        str += (!str.empty() ? filler : "") + "B2a";
    }
    if (value & B07)
    {
        str += (!str.empty() ? filler : "") + "B2b";
    }
    if (value & J01)
    {
        str += (!str.empty() ? filler : "") + "Q1";
    }
    if (value & J02)
    {
        str += (!str.empty() ? filler : "") + "Q2";
    }
    if (value & J05)
    {
        str += (!str.empty() ? filler : "") + "Q5";
    }
    if (value & J06)
    {
        str += (!str.empty() ? filler : "") + "Q6";
    }
    if (value & I05)
    {
        str += (!str.empty() ? filler : "") + "I5";
    }
    if (value & I09)
    {
        str += (!str.empty() ? filler : "") + "IS";
    }
    if (value & S01)
    {
        str += (!str.empty() ? filler : "") + "S1";
    }
    if (value & S05)
    {
        str += (!str.empty() ? filler : "") + "S5";
    }

    if (!str.empty())
    {
        return str;
    }
    return "None";
}

SatelliteSystem Frequency::GetSatelliteSystemForFrequency(Frequency freq)
{
    switch (Frequency_(freq))
    {
    case B01:
    case B02:
    case B05:
    case B06:
    case B07:
    case B08:
        return BDS;
    case E01:
    case E05:
    case E06:
    case E07:
    case E08:
        return GAL;
    case G01:
    case G02:
    case G05:
        return GPS;
    case I05:
    case I09:
        return IRNSS;
    case J01:
    case J02:
    case J05:
    case J06:
        return QZSS;
    case R01:
    case R02:
    case R03:
    case R04:
    case R06:
        return GLO;
    case S01:
    case S05:
        return SBAS;
    case Freq_None:
        return SatSys_None;
    }

    return SatSys_None;
}

double Frequency::GetFrequency(Frequency freq, int8_t num)
{
    switch (Frequency_(freq))
    {
    case B01: // Beidou B1 (1575.42 MHz)
        return 1575.42e6;
    case B02: // Beidou B1-2 (1561.098 MHz)
        return 1561.098e6;
    case B05: // Beidou B2a (1176.45 MHz)
        return 1176.45e6;
    case B06: // Beidou B3 (1268.52 MHz)
        return 1268.52e6;
    case B07: // Beidou B2b (1207.14 MHz)
        return 1207.14e6;
    case B08: // Beidou B2 (B2a + B2b) (1191.795 MHz)
        return 1191.795e6;
    case E01: // Galileo, "E1" (1575.42 MHz)
        return 1575.42e6;
    case E05: // Galileo E5a (1176.45 MHz)
        return 1176.45e6;
    case E06: // Galileo E6 (1278.75 MHz)
        return 1278.75e6;
    case E07: // Galileo E5b (1207.14 MHz)
        return 1207.14e6;
    case E08: // Galileo E5 (E5a + E5b) (1191.795 MHz)
        return 1191.795e6;
    case G01: // GPS L1 (1575.42 MHz)
        return 1575.42e6;
    case G02: // GPS L2 (1227.6 MHz)
        return 1227.6e6;
    case G05: // GPS L5 (1176.45 MHz)
        return 1176.45e6;
    case I05: // IRNSS L5 (1176.45 MHz)
        return 1176.45e6;
    case I09: // IRNSS S (2492.028 MHz)
        return 2492.028e6;
    case J01: // QZSS L1 (1575.42 MHz)
        return 1575.42e6;
    case J02: // QZSS L2 (1227.6 MHz)
        return 1227.6e6;
    case J05: // QZSS L5 (1176.45 MHz)
        return 1176.45e6;
    case J06: // QZSS L6 / LEX (1278.75 MHz)
        return 1278.75e6;
    case R01: // GLONASS, "G1" (1602 MHZ)
        INS_ASSERT_USER_ERROR(num >= -7 && num <= 6, "GLONASS G1 frequency numbers have to be in the range [-7, +6] (all satellites launched after 2005)");
        return (1602.0 + num * 9.0 / 16.0) * 1e6;
    case R02: // GLONASS, "G2" (1246 MHz)
        INS_ASSERT_USER_ERROR(num >= -7 && num <= 6, "GLONASS G1 frequency numbers have to be in the range [-7, +6] (all satellites launched after 2005)");
        return (1246.0 + num * 7.0 / 16.0) * 1e6;
    case R03: // GLONASS, "G3" (1202.025 MHz)
        return 1202.025e6;
    case R04: // GLONASS, "G1a" (1600.995 MHZ)
        return 1600.995e6;
    case R06: // GLONASS, "G2a" (1248.06 MHz)
        return 1248.06e6;
    case S01: // SBAS L1 (1575.42 MHz)
        return 1575.42e6;
    case S05: // SBAS L5 (1176.45 MHz)
        return 1176.45e6;
    case Freq_None:
        return std::nan("");
    }

    return std::nan("");
}

void to_json(json& j, const Frequency& data)
{
    j = std::string(data);
}
void from_json(const json& j, Frequency& data)
{
    data = Frequency::fromString(j.get<std::string>());
}

bool ShowFrequencySelector(const char* label, Frequency& frequency)
{
    bool valueChanged = false;
    if (ImGui::BeginCombo(label, std::string(frequency).c_str(), ImGuiComboFlags_HeightLargest))
    {
        if (ImGui::BeginTable(fmt::format("{} Table", label).c_str(), 7, ImGuiTableFlags_BordersInnerV))
        {
            for (uint64_t satSys = 0xFF; satSys < 0xFFUL << (7 * 8); satSys = satSys << 8UL)
            {
                ImGui::TableSetupColumn(std::string(SatelliteSystem(SatelliteSystem_(satSys))).c_str());
            }
            ImGui::TableHeadersRow();
            for (uint64_t f = 0; f < 8; f++)
            {
                ImGui::TableNextRow();
                for (int c = 0; c < 7; c++)
                {
                    uint64_t flag = (1UL << (f + static_cast<uint64_t>(c) * 8));
                    auto text = std::string(Frequency(Frequency_(flag)));
                    if (text == "None")
                    {
                        continue;
                    }
                    ImGui::TableSetColumnIndex(c);
                    if (c >= 1)
                    {
                        ImGui::BeginDisabled();
                    }
                    ImU64 value = Frequency_(frequency);
                    if (ImGui::CheckboxFlags(text.c_str(), &value, flag))
                    {
                        frequency = Frequency_(value);
                        valueChanged = true;
                    }
                    if (c >= 1)
                    {
                        ImGui::EndDisabled();
                    }
                }
            }
            ImGui::EndTable();
        }
        ImGui::EndCombo();
    }
    return valueChanged;
}

} // namespace NAV
