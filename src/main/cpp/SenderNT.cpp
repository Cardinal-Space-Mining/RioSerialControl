#include "SenderNT.h"

#include <string>

#include <frc/smartdashboard/SendableBuilderImpl.h>


/** Implementation is more or less copied from the SmartDashboard methods */

void SenderNT::putData(wpi::Sendable* data, std::string_view key) {
    if(data) {
        wpi::SendableRegistry::UID& uid = this->data_tables[key];       // creates a new empty slot if not already in the map

        if(data != wpi::SendableRegistry::GetSendable(uid)) {
            uid = wpi::SendableRegistry::GetUniqueId(data);

            std::unique_ptr<frc::SendableBuilderImpl> builder = std::make_unique<frc::SendableBuilderImpl>();
            frc::SendableBuilderImpl* builder_ref = builder.get();      // must store since we std::move the container on publish >>

            builder_ref->SetTable( this->table->GetSubTable(key) );
            wpi::SendableRegistry::Publish(uid, std::move(builder));
            builder_ref->StartListeners();
        }
    }
}
void SenderNT::putData(wpi::Sendable* data) {
    std::string name = wpi::SendableRegistry::GetName(data);
    if(!name.empty()) this->putData(data, name);
}

void SenderNT::updateValues() {
    for(wpi::StringMapEntry<wpi::SendableRegistry::UID>& data : this->data_tables) {
        wpi::SendableRegistry::Update( data.getValue() );
    }
}
