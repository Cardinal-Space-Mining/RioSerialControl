#pragma once

#include <string_view>
#include <memory>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <wpi/sendable/SendableRegistry.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/StringMap.h>


class SenderNT {
public:
    inline SenderNT(std::string_view src_name) : SenderNT( nt::NetworkTableInstance::GetDefault().GetTable(src_name) ) {}
    inline SenderNT(std::shared_ptr<nt::NetworkTable> src) : table(src) {}
    inline ~SenderNT() = default;

    void putData(std::string_view key, wpi::Sendable* data);
    void putData(wpi::Sendable* data);

    void updateValues();

private:
    wpi::StringMap<wpi::SendableRegistry::UID> data_tables{};
    const std::shared_ptr<nt::NetworkTable> table;

};
