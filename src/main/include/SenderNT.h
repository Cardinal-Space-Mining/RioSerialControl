#pragma once

#include <string_view>
#include <memory>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <wpi/sendable/SendableRegistry.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/StringMap.h>


/** SenderNT represents a generalized and cut-down version of the SmartDashboard class.
 * It functionally manages sendable objects and allows their NT data to be updated. */
class SenderNT {
public:
    /** @param src_name - the table under which all sendable objects will be added */
    inline SenderNT(std::string_view src_name) : SenderNT( nt::NetworkTableInstance::GetDefault().GetTable(src_name) ) {}
    /** @param src - the table under which all sendable objects will be added */
    inline SenderNT(std::shared_ptr<nt::NetworkTable> src) : table(src) {}
    inline ~SenderNT() = default;

    /** Add a sendable object under the specified identifier */
    void putData(wpi::Sendable* data, std::string_view key);
    /** Add a sendable object */
    void putData(wpi::Sendable* data);

    /** Update the nt values of all managed sendable instances */
    void updateValues();

private:
    wpi::StringMap<wpi::SendableRegistry::UID> data_tables{};
    const std::shared_ptr<nt::NetworkTable> table;

};
