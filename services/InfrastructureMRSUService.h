/*
* Artery V2X Simulation Framework
* Copyright 2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_INFRASTRUCTUREMRSUSERVICE_H_9QTWT1BY
#define ARTERY_INFRASTRUCTUREMRSUSERVICE_H_9QTWT1BY

#include "artery/application/ItsG5Service.h"
#include "artery/networking/PositionProvider.h"
#include <vanetza/units/length.hpp>
#include <string>

#include "artery/envmod/sensor/FovSensor.h"
#include "artery/envmod/service/CollectivePerceptionMockMessage.h"
#include "artery/envmod/service/CollectivePerceptionMockService.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/InitStages.h"
#include "artery/utility/PointerCheck.h"    
#include <omnetpp/checkandcast.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <iostream>
#include <fstream>
#include <cmath>


#include "artery/envmod/service/EnvmodPrinter.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/traci/VehicleController.h"
#include <boost/units/io.hpp>
#include <omnetpp/clog.h>

namespace artery
{

class InfrastructureMRSUService : public artery::ItsG5Service
{
    public:
        virtual ~InfrastructureMRSUService();

    protected:
        void initialize() override;
        void handleMessage(omnetpp::cMessage*) override;
        void generatePacket();
        void handleAggregateMode(CollectivePerceptionMockMessage* cpm);
        void handleRelayMode(CollectivePerceptionMockMessage* cpm);
        /////
        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
        //void generatingPacket(omnetpp::cPacket*, std::vector<CollectivePerceptionMockMessage::ObjectContainer>& objectContainersingtesting);
        //void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*, std::vector<CollectivePerceptionMockMessage::ObjectContainer>& objectContainersingtesting);
        //void InfrastructureMockService::addingPacket(const vanetza::btp::DataIndication&, cPacket* packet, std::vector<CollectivePerceptionMockMessage::ObjectContainer>& objectContainersingtesting);
        /////

    private:
        int mHostId = 0;
        int mSequenceNumber = 0;
        const PositionProvider* mPositionProvider = nullptr;
        omnetpp::cMessage* mTrigger = nullptr;
        omnetpp::SimTime mInterval = omnetpp::SimTime::ZERO;
        vanetza::units::Length mDisseminationRadius;
        std::string mPacketName;
        unsigned mPacketPriority = 0;
        unsigned mMessageLength = 0;
        
        unsigned mDccProfile = 0;
        omnetpp::SimTime rsuCPMInterval = omnetpp::SimTime::ZERO;
        
        int generateCPMfromRSUcounter=0;
	std::string RSUoperatingMode;	
	int stationType=1;
	omnetpp::cPacket* testpacket = new CollectivePerceptionMockMessage();
    	std::vector<CollectivePerceptionMockMessage::ObjectContainer> objectContainersingtesting;
    	std::vector<double> currentobjectSpeeds;
    	
    	
        
        
        
        
};

} // namespace artery

#endif /* ARTERY_INFRASTRUCTUREMRSUSERVICE_H_9QTWT1BY */

