/*
 * Artery V2X Simulation Framework
 * Copyright 2019 Raphael Riebl et al.
 * The code is modified to handle mobile RSU-assisted CPM dissemination by the authors Thenuka Karunathilake (thenukaramesh@gmail.com)
 */

#include "artery/application/InfrastructureMRSUService.h"
#include "artery/application/InfrastructureMockMessage.h"
#include "artery/envmod/sensor/FovSensor.h"
#include "artery/envmod/service/CollectivePerceptionMockMessage.h"
#include "artery/envmod/service/CollectivePerceptionMockService.h"
#include "artery/envmod/service/EnvmodPrinter.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/InitStages.h"
#include "artery/utility/PointerCheck.h"

#include <boost/format.hpp>
#include <boost/units/io.hpp>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <omnetpp/cwatch.h>
#include <omnetpp/clog.h>
#include <omnetpp/checkandcast.h>

#include <iostream>
#include <fstream>
#include <cmath>
#include <numeric>

std::ofstream RSUfile("results/dissem/RSUfile.csv");
std::ofstream RSUfileattransmission("results/dissem/RSUtransmissions.csv");
std::ofstream RSUtransmittedCPMS("results/dissem/RSUtransmittedCPMS.csv");
std::ofstream RSUCPMInterval("results/dissem/RSUCPMInterval.csv");

int CPMsfromRsu = 0;
double nextCPMfromRSUInterval = 1;


namespace artery {

Define_Module(InfrastructureMRSUService)

namespace {
    using namespace omnetpp;
    const simsignal_t immSentSignal = cComponent::registerSignal("ImmSent");
}

InfrastructureMRSUService::~InfrastructureMRSUService() {
    cancelAndDelete(mTrigger);
}

void InfrastructureMRSUService::initialize() {
    ItsG5Service::initialize();
    mTrigger = new omnetpp::cMessage("trigger infrastructure mock message");
    mPositionProvider = &getFacilities().get_const<PositionProvider>();
    mHostId = getFacilities().get_const<Identity>().host->getId();

    mPacketName = (boost::format("%1% mock-up packet") % this->getName()).str();
    mMessageLength = par("messageLength");
    if (mMessageLength < par("messageLengthMin").intValue()) {
        mMessageLength = par("messageLengthMin");
    } else if (mMessageLength > par("messageLengthMax").intValue()) {
        mMessageLength = par("messageLengthMax");
    }
    omnetpp::createWatch("messageLength", mMessageLength);

    mDisseminationRadius = par("disseminationRadius").doubleValue() * boost::units::si::meter;
    mPacketPriority = par("packetPriority");
    mDccProfile = par("dccProfile");
    mInterval = par("generationInterval");
    rsuCPMInterval = par("rsuCPMInterval");
    RSUoperatingMode = par("RSUoperatingMode").stdstringValue();	
    

    scheduleAt(omnetpp::simTime() + par("generationOffset"), mTrigger);
}

void InfrastructureMRSUService::handleMessage(omnetpp::cMessage* msg) {
    if (msg == mTrigger) {
        nextCPMfromRSUInterval = 0.1;

        if (!objectContainersingtesting.empty()) {
            generatePacket();
        }

        scheduleAt(omnetpp::simTime() + rsuCPMInterval, mTrigger);
    } else {
        ItsG5Service::handleMessage(msg);
    }
}

void InfrastructureMRSUService::generatePacket() {
    generateCPMfromRSUcounter++;

    auto packet = new CollectivePerceptionMockMessage();
    packet->setObjectContainers(std::move(objectContainersingtesting));
    packet->setSourceStation(mHostId);
    packet->setSourceStationType(stationType);

    RSUfileattransmission.open("results/dissem/RSUtransmissions.csv", std::ios_base::app);

    std::vector<double> currentobjectSpeeds;
    for (const auto& container : packet->getObjectContainers()) {
        if (!container.object.expired()) {
            const auto& vd = container.object.lock()->getVehicleData();

            RSUfileattransmission << CPMsfromRsu << " "
                                  << packet->getObjectContainers().size() << " "
                                  << mHostId << " "
                                  << packet->getSourceStation() << " "
                                  << packet->getCreationTime() << " "
                                  << omnetpp::simTime() << " "
                                  << container.timeOfMeasurement << " "
                                  << vd.station_id() << " "
                                  << vd.longitude() << " "
                                  << vd.latitude() << " "
                                  << vd.speed() << " "
                                  << vd.heading() << std::endl;

            currentobjectSpeeds.push_back(vd.speed().value());
        }
    }

    RSUCPMInterval.open("results/dissem/RSUCPMInterval.csv", std::ios_base::app);
    double sum = std::accumulate(currentobjectSpeeds.begin(), currentobjectSpeeds.end(), 0.0);
    double averageobjectSpeed=sum/currentobjectSpeeds.size();

    nextCPMfromRSUInterval=0.1;
    if (boost::size(currentobjectSpeeds) > 0) 
    {	
    	nextCPMfromRSUInterval= ((0.1-0.05)/(0-27.778))*averageobjectSpeed + 0.1; // linear function to select the next cpm generation time at the RSU
    	if (nextCPMfromRSUInterval < 0.05) 
    	{	
    		nextCPMfromRSUInterval=0.05;
    	}
    		
    }
    RSUCPMInterval << mHostId << " "
                   << omnetpp::simTime() << " "
                   << nextCPMfromRSUInterval << " "
                   << averageobjectSpeed << std::endl;

    RSUCPMInterval.close();
    RSUfileattransmission.close();

    RSUtransmittedCPMS.open("results/dissem/RSUtransmittedCPMS.csv", std::ios_base::app);
    RSUtransmittedCPMS << CPMsfromRsu << " "
                       << mHostId << " "
                       << packet->getObjectContainers().size() << " "
                       << omnetpp::simTime() << std::endl;
    RSUtransmittedCPMS.close();

    CPMsfromRsu++;

    using namespace vanetza;
    btp::DataRequestB req;
    req.destination_port = host_cast<InfrastructureMRSUService::port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(mDccProfile);
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    request(req, packet);
}

void InfrastructureMRSUService::indicate(const vanetza::btp::DataIndication&, cPacket* packet) {
    auto cpm = omnetpp::check_and_cast<CollectivePerceptionMockMessage*>(packet);

    if (cpm->getSourceStationType() != stationType && cpm->getSourceStation() != mHostId) {
        if (RSUoperatingMode == "aggregate") {
            handleAggregateMode(cpm);
        } else if (RSUoperatingMode == "relay") {
            handleRelayMode(cpm);
        }
    }

    delete packet;
}

void InfrastructureMRSUService::handleAggregateMode(CollectivePerceptionMockMessage* cpm) {
    RSUfile.open("results/dissem/RSUfile.csv", std::ios_base::app);

    for (const auto& container : cpm->getObjectContainers()) {
        if (!container.object.expired()) {
            const auto& vd = container.object.lock()->getVehicleData();

            CollectivePerceptionMockMessage::ObjectContainer newContainer;
            newContainer.object = container.object.lock();
            newContainer.timeOfMeasurement = cpm->getCreationTime();

            bool found = false;
            for (auto it = objectContainersingtesting.rbegin(); it != objectContainersingtesting.rend(); ++it) {
                if (!it->object.expired()) {
                    const auto& existingVd = it->object.lock()->getVehicleData();
                    if (existingVd.station_id() == vd.station_id()) {
                        found = true;
                        if (it->timeOfMeasurement < newContainer.timeOfMeasurement) {
                            objectContainersingtesting.erase(std::next(it).base());
                            objectContainersingtesting.emplace_back(std::move(newContainer));
                        }
                        break;
                    }
                }
            }

            if (!found) {
                objectContainersingtesting.emplace_back(std::move(newContainer));
            }

            RSUfile << mHostId << " "
                   << cpm->getSourceStation() << " "
                   << cpm->getCreationTime() << " "
                   << omnetpp::simTime() << " "
                   << container.timeOfMeasurement << " "
                   << vd.station_id() << " "
                   << vd.longitude() << " "
                   << vd.latitude() << " "
                   << vd.speed() << " "
                   << vd.heading() << std::endl;
        }
    }

    RSUfile.close();
}

void InfrastructureMRSUService::handleRelayMode(CollectivePerceptionMockMessage* cpm) {
    if (!cpm->getObjectContainers().empty()) {
        cpm->setSourceStation(mHostId);
        cpm->setSourceStationType(stationType);

        RSUfileattransmission.open("results/dissem/RSUtransmissions.csv", std::ios_base::app);
        for (const auto& container : cpm->getObjectContainers()) {
            if (!container.object.expired()) {
                const auto& vd = container.object.lock()->getVehicleData();

                RSUfileattransmission << generateCPMfromRSUcounter << " "
                                      << cpm->getObjectContainers().size() << " "
                                      << mHostId << " "
                                      << cpm->getSourceStation() << " "
                                      << cpm->getCreationTime() << " "
                                      << omnetpp::simTime() << " "
                                      << container.timeOfMeasurement << " "
                                      << vd.station_id() << " "
                                      << vd.longitude() << " "
                                      << vd.latitude() << " "
                                      << vd.speed() << " "
                                      << vd.heading() << std::endl;
            }
        }
        RSUfileattransmission.close();

        RSUtransmittedCPMS.open("results/dissem/RSUtransmittedCPMS.csv", std::ios_base::app);
        RSUtransmittedCPMS << CPMsfromRsu << " "
                           << mHostId << " "
                           << cpm->getObjectContainers().size() << " "
                           << omnetpp::simTime() << std::endl;
        RSUtransmittedCPMS.close();

        std::cout << "CPMs from RSU " << CPMsfromRsu << std::endl;
        CPMsfromRsu++;

        using namespace vanetza;
        btp::DataRequestB req;
        req.destination_port = host_cast<InfrastructureMRSUService::port_type>(getPortNumber());
        req.gn.transport_type = geonet::TransportType::SHB;
        req.gn.traffic_class.tc_id(mDccProfile);
        req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

        request(req, cpm);
        generateCPMfromRSUcounter++;
    }
}

} // namespace artery

