/*
 * Artery V2X Simulation Framework
 * Copyright 2019 Raphael Riebl et al.
 * The code is modified to CPM prioritizing optimization with AHP algorithm and to generate results CSV files by the authors Thenuka Karunatilake (thenukaramesh@gmail.com) and Akhil Neela (akhilsimha74@gmail.com)
 */

#include "artery/envmod/sensor/FovSensor.h"
#include "artery/envmod/service/CollectivePerceptionMockMessage.h"
#include "artery/envmod/service/CollectivePerceptionServiceAHP.h"
#include "artery/envmod/service/EnvmodPrinter.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/InitStages.h"
#include "artery/utility/PointerCheck.h"
#include <omnetpp/checkandcast.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <boost/units/io.hpp>
#include <unordered_set>
#include <omnetpp/clog.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>


#include "CPMGlobals.h"

std::ofstream CPMstatsV2V;
std::ofstream CPMstatstotal;
std::ofstream CPMstatsforRSUmessages;
std::ofstream VehicletransmittedCPMS;
int CPMsfromVehicles = 0;

std::ofstream DelayAHP;
std::ofstream AwarenessAHP;
std::ofstream CPMDataAHP;


namespace artery {

Define_Module(CollectivePerceptionServiceAHP)

namespace {

omnetpp::simsignal_t camSentSignal = omnetpp::cComponent::registerSignal(
        "CamSent");
omnetpp::simsignal_t cpmSentSignal = omnetpp::cComponent::registerSignal(
        "CpmSent");
omnetpp::simsignal_t cpmReceivedSignal = omnetpp::cComponent::registerSignal(
            "CpmReceived");

} // namespace

CollectivePerceptionServiceAHP::~CollectivePerceptionServiceAHP() {
    cancelAndDelete (mTrigger);
}

int CollectivePerceptionServiceAHP::numInitStages() const {
    // This method returns the total number of initialization stages required by the service.
    return InitStages::Total;
}

void CollectivePerceptionServiceAHP::initialize(int stage) {
    if (stage == InitStages::Prepare) {
        ItsG5Service::initialize();
        mPositionProvider = &getFacilities().get_const<PositionProvider>();
        mVehicleDataProvidernew = &getFacilities().get_const<VehicleDataProvider>();
        mEnvironmentModel = &getFacilities().get_const<LocalEnvironmentModel>();
        mLocalEnvironmentModel = getFacilities().get_mutable_ptr <
                                 LocalEnvironmentModel > (); // Added for Sensor Data

        mDccProfile = par("dccProfile");
        mLengthHeader = par("lengthHeader");
        mLengthFovContainer = par("lengthFovContainer");
        mLengthObjectContainer = par("lengthObjectContainer");

        mGenerateAfterCam = par("generateAfterCam");
        mCpmOffset = par("cpmOffset");
        mCpmInterval = par("cpmInterval");
        //CpmValidityInterval = par("")


        // Calculate Awareness every 0.1s.
        awarenessTimer = new omnetpp::cMessage("AwarenessTimer");
        scheduleAt(omnetpp::simTime() + 0.1, awarenessTimer);
        
        useDynamicDissemination = par("useDynamicDissemination").boolValue();
        receivedFromStationType1Recently = false;
	lastStationType1Reception = omnetpp::simTime();


        EV_INFO << "INITIAL VALUE:" << mCpmInterval;
    } else if (stage == InitStages::Self) {
        mTrigger = new omnetpp::cMessage("triggger mock-up CPM");
        if (mGenerateAfterCam) {
            subscribe (camSentSignal);
        }
        mHostId = getFacilities().get_const<Identity>().host->getId();
    } else if (stage == InitStages::Propagate) {
        for (const Sensor *sensor : mEnvironmentModel->getSensors()) {
            // consider only sensors with a field of view
            if (auto fovSensor = dynamic_cast<const FovSensor*>(sensor)) {
                CollectivePerceptionMockMessage::FovContainer fovContainer;
                fovContainer.sensorId = sensor->getId();
                fovContainer.position = sensor->position();
                fovContainer.fov = fovSensor->getFieldOfView();
                mFovContainers.emplace_back(std::move(fovContainer));
                mSensors.insert(sensor);
            }
        }
    }
}

void CollectivePerceptionServiceAHP::handleMessage(omnetpp::cMessage *msg) {
    if (msg == awarenessTimer) {
        calculateAwareness();
        // Schedule the next awareness calculation
        scheduleAt(omnetpp::simTime() + 0.1, awarenessTimer);
    } else if (msg == mTrigger) {
        generatePacket();
    } else {
        ItsG5Service::handleMessage(msg);
    }
}

void CollectivePerceptionServiceAHP::receiveSignal(omnetpp::cComponent*,
        omnetpp::simsignal_t signal, omnetpp::cObject *obj, omnetpp::cObject*) {
    if (signal == camSentSignal) {
        scheduleAt(omnetpp::simTime() + mCpmOffset, mTrigger);
    }
}

void CollectivePerceptionServiceAHP::trigger() {

    
    auto &allObjects = mLocalEnvironmentModel->allObjects();
    std::unordered_set<int> radarStationIDs = getNeighboursStationIDs(allObjects, "Radar");

    // Call the function for "CA" category
    std::unordered_set<int> caStationIDs = getNeighboursStationIDs(allObjects, "CA");

    std::unordered_set<int> mergedStationIDs = radarStationIDs;
    mergedStationIDs.insert(caStationIDs.begin(), caStationIDs.end());
    mergedStationIDs.insert(cpStationIDs.begin(), cpStationIDs.end());

    // Calculate the differences and the number of differences
    std::unordered_set<int> newMergedStationIDs;
    std::vector<int> differences;
    for (int stationID : mergedStationIDs) {
        if (prevMergedStationIDs.find(stationID) == prevMergedStationIDs.end()) {
            differences.push_back(stationID);
        }
        newMergedStationIDs.insert(stationID);
    }
    int numDifferences = differences.size();

    // Print the sets using EV_INFO
    EV_INFO << '|' << mHostId << '|' << "Radar Station IDs      : ";
    for (int stationID : radarStationIDs) {
        EV_INFO << stationID << " ";
    }
    EV_INFO << "\n";

    EV_INFO << '|' << mHostId << '|' << "CA Station IDs         : ";
    for (int stationID : caStationIDs) {
        EV_INFO << stationID << " ";
    }
    EV_INFO << "\n";

    EV_INFO << '|' << mHostId << '|' << "CP Station IDs         : ";
    for (int stationID : cpStationIDs) {
        EV_INFO << stationID << " ";
    }
    EV_INFO << "\n";

    EV_INFO << '|' << mHostId << '|' << "Merged Station IDs     : ";
    for (int stationID : mergedStationIDs) {
        EV_INFO << stationID << " ";
    }
    EV_INFO << "\n";

    // Print the differences and the number of differences
    EV_INFO << '|' << mHostId << '|' << "Newly Added Station IDs: ";
    for (int stationID : differences) {
        EV_INFO << stationID << " ";
    }
    EV_INFO << "\n";
    EV_INFO << '|' << mHostId << '|' << "Number of Differences  : " << numDifferences << "\n";
    prevMergedStationIDs.clear();
    // Update prevMergedStationIDs with the current set for the next call
    prevMergedStationIDs = newMergedStationIDs;


    artery::GeoPosition position = getFacilities().get_const<traci::VehicleController>().getGeoPosition();

    // Access the latitude and longitude values
    double latitude = boost::units::quantity_cast<double>(position.latitude);
    double longitude = boost::units::quantity_cast<double>(position.longitude);

    EV << getFacilities().get_const<traci::VehicleController>().getVehicleId() << " Latitude:" << latitude << " Longitude: " << longitude << std::endl;

    // Hotspot Regions for LuST Scenario

    std::vector<Point> regions = {
        {49.620460, 6.070255, 0.8},   //4 Way Junction
        {49.619355, 6.078061, 0.8},
        {49.616435, 6.098363, 0.8},
        {49.613285, 6.119873, 0.8},
        {49.609565, 6.119403, 0.8},
        {49.606340, 6.120000, 0.8},
        {49.608037, 6.112062, 0.8},
        {49.606542, 6.119926, 0.8},
        {49.604659, 6.133518, 0.8},
        {49.603739, 6.136707, 0.8},
        {49.598525, 6.140053, 0.8},
        {49.597427, 6.140180, 0.8},
        {49.612254, 6.125923, 0.8},
        {49.613294, 6.125987, 0.8},
        {49.613809, 6.127838, 0.7},
        {49.614625, 6.130362, 0.7},
        {49.604739, 6.133482, 0.7},
        {49.609433, 6.126402, 0.7},
        {49.621636, 6.151841, 0.7},
        {49.629565, 6.166936, 0.7},
        {49.603136, 6.074107, 0.6}, //3 Way Junction
        {49.603863, 6.078053, 0.6},
        {49.603424, 6.083034, 0.6},
        {49.603501, 6.084832, 0.6},
        {49.604836, 6.094679, 0.6},
        {49.609631, 6.097451, 0.6},
        {49.612383, 6.079651, 0.6},
        {49.622086, 6.058395, 0.6},
        {49.619924, 6.073975, 0.6},
        {49.616149, 6.100710, 0.6},
        {49.612226, 6.126703, 0.6},
        {49.613276, 6.128045, 0.5},
        {49.607164, 6.133421, 0.5},
        {49.598182, 6.132267, 0.5},
        {49.594182, 6.117183, 0.5},
        {49.594141, 6.117125, 0.5},
        {49.571866, 6.145145, 0.5},
        {49.568809, 6.161038, 0.5},
        {49.591619, 6.126486, 0.5},
        {49.592724, 6.128183, 0.5},
        {49.598382, 6.055189, 1.0}, //RoundAbouts
        {49.611563, 6.059672, 1.0},
        {49.615192, 6.060453, 1.0},
        {49.621974, 6.051766, 1.0},
        {49.628821, 6.065523, 1.0},
        {49.633028, 6.064881, 1.0},
        {49.647204, 6.084485, 1.0},
        {49.645743, 6.101026, 1.0},
        {49.610645, 6.070976, 0.9},
        {49.608077, 6.073630, 0.9},
        {49.611855, 6.080292, 0.9},
        {49.618788, 6.081821, 0.9},
        {49.602249, 6.099671, 0.9},
        {49.606386, 6.106329, 0.9},
        {49.599824, 6.118631, 0.9},
        {49.598674, 6.137880, 0.9},
        {49.597254, 6.135031, 0.9},
        {49.585584, 6.133578, 0.9},
        {49.587008, 6.115783, 0.9},
        {49.635442, 6.176684, 0.9},
        {49.606326, 6.106394, 1.0},
        {49.614777, 6.161521, 0.4}, //Other Junction
        {49.614149, 6.162546, 0.4},
        {49.610127, 6.167240, 0.4},
        {49.608581, 6.163302, 0.4},
        {49.607005, 6.157474, 0.4},
        {49.609828, 6.148025, 0.4},
        {49.610510, 6.146740, 0.4},
        {49.610371, 6.138883, 0.4},
        {49.610088, 6.136932, 0.4},
        {49.616189, 6.143550, 0.4},
        {49.613900, 6.138663, 0.4},
        {49.613072, 6.131189, 0.4},
        {49.605156, 6.143203, 0.4},
        {49.595381, 6.152972, 0.4},
    };

    // Hotspot Regions for CN+ Scenario

    // std::vector<Point> regions = {
    //     {53.105103, 8.850357, 1},
    //     {53.105494, 8.850655, 0.8},
    // };


    double radius = 30.0; // in meters

    for (const Point& point : regions) {
        double distance = haversineDistance(latitude, longitude, point.latitude, point.longitude);
        if (distance <= radius) {
            EV_INFO << "Vehicle is in the vicinity of a point with value: " << point.value << std::endl;
            hotspot = point.value;
            // You can assign the point's value to a variable here
            // based on your requirements.
        }
    }

    double importanceRatio = static_cast<double>(radarStationIDs.size()) / static_cast<double>(mergedStationIDs.size());
    double uniquenessRatio = static_cast<double>(numDifferences) / static_cast<double>(mergedStationIDs.size());


    // AHP Algorithm Implementation ---------------------

    // Define the input values
    std::vector<double> inputs = {importanceRatio, uniquenessRatio, hotspot};
    
    //  comparison matrix
    std::vector<std::vector<double>> comparisonMatrix = {
        {1.0, 5.0, 1.0 / 7.0},
        {1.0 / 5.0, 1.0, 1.0 / 3.0},
        {7.0, 3.0, 1.0}
    };

    // Calculate priorities
    std::vector<double> priorities = calculatePriorities(comparisonMatrix, inputs);

    // Calculate the final output (assuming a weighted sum)
    double finalOutput = 0.0;
    for (size_t i = 0; i < priorities.size(); ++i) {
        finalOutput += priorities[i] * inputs[i];
    }

    double minOutput = 0.1;
    double maxOutput = 1.0;
    double finalOutput1 = minOutput + (maxOutput - minOutput) * (1 - finalOutput);
    // Print the calculated final output
    //std::cout << finalOutput << "Final Output: " << finalOutput1 << std::endl;


    if (finalOutput1 > 0)
        mCpmInterval = finalOutput1;
    //std::cout<<useDynamicDissemination<<" "<<receivedFromStationType1Recently<<std::endl;    
    if (useDynamicDissemination) {
    	
        if (receivedFromStationType1Recently && omnetpp::simTime() - lastStationType1Reception <= 0.1) {
            mCpmInterval = 1.0; // 1000 ms
            //std::cout<<"have a CPM from RSU less than 100 ms ago so setting CPM interval to 1000ms"<<std::endl;
        } else {
            receivedFromStationType1Recently = false;
            mCpmInterval = mCpmInterval;
           // std::cout<<"have a CPM from RSU but not less than 100 ms ago so setting CPM interval to "<< mCpmInterval <<std::endl;
            
        }
    } else {
    	mCpmInterval =mCpmInterval;
    	//std::cout<<useDynamicDissemination<<" not using dynamic dissemination "<< mCpmInterval <<std::endl;
    }    
    
    //std::cout<<"  "<<std::endl;
    // End of Algorithm Implementation

    EV_INFO << '|' << mHostId << '|' << "CPM INTERVAL: " << mCpmInterval << std::endl;


    if (!mGenerateAfterCam && !mTrigger->isScheduled()) {
        auto channel =
            getFacilities().get_const<MultiChannelPolicy>().primaryChannel(
                vanetza::aid::CP);
        auto netifc = notNullPtr(
                          getFacilities().get_const<NetworkInterfaceTable>().select(
                              channel));
        vanetza::dcc::TransmitRateThrottle *trc = notNullPtr(
                    netifc->getDccEntity().getTransmitRateThrottle());
        vanetza::dcc::TransmissionLite tx {
            static_cast<vanetza::dcc::Profile>(mDccProfile), 0 };
        const omnetpp::SimTime interval = std::chrono::duration<double>(
                                              trc->interval(tx)).count(); // This is calculating the interval for transmission based on the transmit rate
        // throttle and the transmission profile.
        const omnetpp::SimTime next = omnetpp::simTime() + mCpmOffset;
        //mCpmInterval = uniform (0.1,1);
        if (mTrigger->getArrivalTime() + std::max(interval, mCpmInterval)
                <= next) {
            scheduleAt(next, mTrigger);
            CPMDataAHP.open("results/AHP/CPMDataAHP.csv", std::ios_base::app);
            CPMDataAHP << mHostId << "," << omnetpp::simTime() << "," << next
                 << "," << mCpmInterval << "," << mCpmOffset << "," << importanceRatio << "," <<  uniquenessRatio << ',' << hotspot << "\n";
            CPMDataAHP.close();
        }
    }

}

void CollectivePerceptionServiceAHP::indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket* packet)
{
    auto cpm = omnetpp::check_and_cast<CollectivePerceptionMockMessage*>(packet);
    
    CPMstatsV2V.open("results/utility/CPMstatsV2V.csv", std::ios_base::app);
    CPMstatstotal.open("results/utility/CPMstatstotal.csv", std::ios_base::app);
    CPMstatsforRSUmessages.open("results/utility/CPMstatsforRSUmessages.csv", std::ios_base::app);
    
    if (useDynamicDissemination && cpm->getSourceStationType() == 1) {
        lastStationType1Reception = omnetpp::simTime();
        receivedFromStationType1Recently = true;
    }
  
    
    for (int i=0; i<boost::size(cpm->getObjectContainers());i++ )
    {
    	
		if (! cpm->getObjectContainers()[i].object.expired()) {
        	const VehicleDataProvider& vd = cpm->getObjectContainers()[i].object.lock()->getVehicleData();
        	
        	if(vd.station_id() != mVehicleDataProvidernew->station_id()){
        	
				
				
				CPMstatstotal<<CPMsfromVehicles<<" "<<mHostId<<" "<<cpm->getSourceStation()<<" "<<cpm->getCreationTime()<<" "<<omnetpp::simTime()<<" "<<cpm->getSourceStationType()<<" "<<cpm->getObjectContainers()[i].timeOfMeasurement<<" "<<mVehicleDataProvidernew->station_id()<<" "<<mVehicleDataProvidernew->speed()<<" "<<mVehicleDataProvidernew->longitude()<<" "<<mVehicleDataProvidernew->latitude()<<" "<<mVehicleDataProvidernew->heading()<< " " << vd.station_id()<< " " << vd.longitude()<< " " << vd.latitude()<< " " << vd.speed()<< " " << " " << vd.heading()<< std::endl;
				
				
    	
        	
       	}
        	
        	if(vd.station_id() != mVehicleDataProvidernew->station_id() && cpm->getSourceStationType() == 1){
        	
				
				
				CPMstatsforRSUmessages<<CPMsfromVehicles<<" "<<mHostId<<" "<<cpm->getSourceStation()<<" "<<cpm->getCreationTime()<<" "<<omnetpp::simTime()<<" "<<cpm->getSourceStationType()<<" "<<cpm->getObjectContainers()[i].timeOfMeasurement<<" "<<mVehicleDataProvidernew->station_id()<<" "<<mVehicleDataProvidernew->speed()<<" "<<mVehicleDataProvidernew->longitude()<<" "<<mVehicleDataProvidernew->latitude()<<" "<<mVehicleDataProvidernew->heading()<< " " << vd.station_id()<< " " << vd.longitude()<< " " << vd.latitude()<< " " << vd.speed()<< " " << " " << vd.heading()<< std::endl;
				
				
    	
        	
       	}
        	
        	
        	if(vd.station_id() != mVehicleDataProvidernew->station_id()&& cpm->getSourceStationType() == 0){
        	
				
				
				CPMstatsV2V<<CPMsfromVehicles<<" "<<mHostId<<" "<<cpm->getSourceStation()<<" "<<cpm->getCreationTime()<<" "<<omnetpp::simTime()<<" "<<cpm->getSourceStationType()<<" "<<cpm->getObjectContainers()[i].timeOfMeasurement<<" "<<mVehicleDataProvidernew->station_id()<<" "<<mVehicleDataProvidernew->speed()<<" "<<mVehicleDataProvidernew->longitude()<<" "<<mVehicleDataProvidernew->latitude()<<" "<<mVehicleDataProvidernew->heading()<< " " << vd.station_id()<< " " << vd.longitude()<< " " << vd.latitude()<< " " << vd.speed()<< " " << " " << vd.heading()<< std::endl;
						
        	
       	}
       	
		
		}
    }
    CPMstatsforRSUmessages.close();
    CPMstatsV2V.close();
    CPMstatstotal.close();
   
    
    const std::vector<CollectivePerceptionMockMessage::ObjectContainer>& objectContainers = cpm->getObjectContainers();

    omnetpp::SimTime delay = 0;
    omnetpp::SimTime currentTime = omnetpp::simTime();
    omnetpp::SimTime creationTime = cpm->getCreationTime();
    delay = currentTime - creationTime;
    double delayInSeconds = delay.dbl();


    // Update the matrix with new data
    for (const auto& objectContainer : objectContainers) {
        ObjectData objectData;
        objectData.creationTime = cpm->getCreationTime();
        objectData.objectId = objectContainer.objectId;
        dataMatrix.push_back(objectData);
    }


    // Print data before update
    EV_INFO << '|' << mHostId << '|' << omnetpp::simTime() << " Data before update:" << std::endl;
    for (const auto& data : dataMatrix) {
        EV_INFO << '|' << mHostId << '|' << "Creation Time: " << data.creationTime << ", Object ID: " << data.objectId << std::endl;
    }

    // Remove old data from the matrix
    dataMatrix.erase(
        std::remove_if(dataMatrix.begin(), dataMatrix.end(),
    [&](const ObjectData & data) {
        return (currentTime - data.creationTime) >= 1.1;
    }),
    dataMatrix.end()
    );

    // Add vehicle numbers to the set
    cpStationIDs.clear();  // Clear the set before adding new numbers
    for (const auto& data : dataMatrix) {
        if (data.objectId != mHostId) {  // Exclude mHostId
            cpStationIDs.insert(data.objectId);
        }
    }


    // Print data after update
    EV_INFO << '|' << mHostId << '|' << "Data after update:" << std::endl;
    for (const auto& data : dataMatrix) {
        EV_INFO << '|' << mHostId << '|' << "Creation Time: " << data.creationTime << ", Object ID: " << data.objectId << std::endl;
    }

    // Print the contents of the vehicleNumbersSet
    EV_INFO << '|' << mHostId << '|' << "Vehicle Numbers Set:" ;
    for (int vehicleNumber : cpStationIDs) {
        EV_INFO << vehicleNumber << " ";
    }
    // Append data to the existing file
    DelayAHP.open("results/AHP/DelayAHP.csv", std::ios_base::app);
    DelayAHP << omnetpp::simTime() << "," << delay << "\n";
    DelayAHP.close();

    emit(cpmReceivedSignal, cpm);
    delete packet;


}


void CollectivePerceptionServiceAHP::generatePacket() {
    using namespace vanetza;
    btp::DataRequestB req;
    req.destination_port = host_cast < PortNumber > (getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(mDccProfile);
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto packet = new CollectivePerceptionMockMessage();
    packet->setByteLength(mLengthHeader);

    if (mFovLast + mFovInterval <= omnetpp::simTime()) { //Check if it's time to update the field of view based on the last update time and the interval.
        packet->setFovContainers(mFovContainers);
        packet->addByteLength(mLengthFovContainer * mFovContainers.size());
        mFovLast = omnetpp::simTime();
    }

    std::vector < CollectivePerceptionMockMessage::ObjectContainer
    > objectContainers;
    using TrackedObject = LocalEnvironmentModel::TrackedObject;
    for (const TrackedObject &object : mEnvironmentModel->allObjects()) {
        const LocalEnvironmentModel::Tracking &tracking = object.second;
        if (tracking.expired()) {
            // skip objects with lost tracking
            continue;
        }

        for (const auto &sensor : tracking.sensors()) {
            std::string sensorName = sensor.first->getSensorName();
            // To Print out data regarding all the detected vehicles by which car and how.
            //EV_INFO << '|' << mHostId << '|' <<"Sensor Name : " << sensorName << " Sensor ID: " << sensor.first->getId() << std::endl; // This iterates one sensor by another.
            if (sensorName != "SeeThrough") {
                if (mSensors.find(sensor.first) != mSensors.end()) {

                    const auto obj_ptr2 = object.first.lock();
                    if (!obj_ptr2) {
                        continue; /*< objects remain in tracking briefly after leaving simulation */
                    }
                    const auto& vd = obj_ptr2->getVehicleData();
                    
                    int objId = vd.station_id();
		    double x = boost::units::quantity_cast<double>(vd.longitude());
		    double y = boost::units::quantity_cast<double>(vd.latitude());
		    double speed = boost::units::quantity_cast<double>(vd.speed());
		    double heading = boost::units::quantity_cast<double>(vd.heading());
		    omnetpp::SimTime now = omnetpp::simTime();

		    bool include = false;
		    
		    auto it = lastKnownObjects.find(objId);
		    if (it == lastKnownObjects.end()) {
		    	include = true;
		    } else {
		    	const auto& last = it->second;
		        double dx = x - last.x;
		        double dy = y - last.y;
		        double distMoved = std::sqrt(dx*dx + dy*dy) * 111139; // convert degrees to meters (approx)
		        double speedDiff = std::abs(speed - last.speed);
		        double headingDiff = std::abs(heading - last.heading);
		        omnetpp::SimTime timeDiff = now - last.lastIncluded;

		        if (distMoved >= 4.0 || speedDiff >= 0.5 || headingDiff >= 4.0 || timeDiff >= 1.0) {
		        	include = true;
		        	//std::cout<<"Inclusion Rules statisfied"<<std::endl;
		        }else{
		        	//std::cout<<"Inclusion Rules not statisfied"<<std::endl;
		        }
		        }

		        if (include) {
		            CollectivePerceptionMockMessage::ObjectContainer objectContainer;
		            objectContainer.object = object.first;
		            objectContainer.objectId = objId;
		            objectContainer.sensorId = sensor.first->getId();
		            objectContainer.timeOfMeasurement = sensor.second.last();
		            objectContainers.emplace_back(std::move(objectContainer));

		            lastKnownObjects[objId] = {x, y, speed, heading, now};
		        }
		    
                    //EV_DETAIL << '|' << mHostId << '|' << " Detected station ID: " << vd.station_id() << " Sensor ID:" << sensor.first->getId() << std::endl;


                    //CollectivePerceptionMockMessage::ObjectContainer objectContainer; // create a container.
                    //objectContainer.object = object.first;
                    //objectContainer.objectId = vd.station_id();
                    //objectContainer.sensorId = sensor.first->getId();
                    //objectContainer.timeOfMeasurement = sensor.second.last();
                    //objectContainers.emplace_back(std::move(objectContainer));
                }
            }
        }

    }

    
   
    // If no objects included and less than 1s has passed since last CPM, abort
    if (objectContainers.empty() && (omnetpp::simTime() - lastCpmSentTime < 1.0)) {
        delete packet;
        return; // skip CPM transmission
    }
    
    VehicletransmittedCPMS.open("results/utility/VehicletransmittedCPMS.csv", std::ios_base::app);
    VehicletransmittedCPMS<<CPMsfromVehicles<<" "<<mHostId<<" "<<omnetpp::simTime()<<" "<<boost::size(objectContainers)<<std::endl;
    VehicletransmittedCPMS.close();
    
    packet->addByteLength(mLengthObjectContainer * objectContainers.size());
    packet->setObjectContainers(std::move(objectContainers));
    packet->setSourceStation(mHostId);
    packet->setSourceStationType(stationType);
//EV_INFO << "MockService.cc : CPM MESSAGE SENT from Station ID: " << mHostId;

    
    CPMsfromVehicles++;
	
    lastCpmSentTime = omnetpp::simTime();
    emit(cpmSentSignal, packet);

    request(req, packet);
}

void CollectivePerceptionServiceAHP::printSensorObjectList(const std::string& title, const TrackedObjectsFilterRange& objs)
{
    EV_DETAIL << "--- " << title << " (" << boost::size(objs) << " objects) ---" << std::endl;

    for (const auto& obj : objs)
    {
        const auto obj_ptr = obj.first.lock();
        if (!obj_ptr) {
            continue; /*< objects remain in tracking briefly after leaving simulation */
        }
        const auto& vd = obj_ptr->getVehicleData();
        EV_DETAIL << '|' << mHostId << '|'<< " station ID: " << vd.station_id() << std::endl;
    }
}



// Function to get unique station IDs
std::unordered_set<int> CollectivePerceptionServiceAHP::getNeighboursStationIDs(const LocalEnvironmentModel::TrackedObjects& objs, const std::string& category) {
    std::unordered_set<int> uniqueStationIDs;
    auto filteredObjs = filterBySensorCategory(objs, category);

    for (const auto& obj : filteredObjs) {
        const auto obj_ptr = obj.first.lock();
        if (obj_ptr) {
            const auto& vd = obj_ptr->getVehicleData();
            uniqueStationIDs.insert(vd.station_id());
        }
    }

    return uniqueStationIDs;
}


void CollectivePerceptionServiceAHP::calculateAwareness() {
    auto &allObjects = mLocalEnvironmentModel->allObjects();

    std::unordered_set<int> radarStationIDs = getNeighboursStationIDs(allObjects, "Radar");
    std::unordered_set<int> caStationIDs = getNeighboursStationIDs(allObjects, "CA");

    std::unordered_set<int> allUniqueStationIDs;
    allUniqueStationIDs.insert(radarStationIDs.begin(), radarStationIDs.end());
    allUniqueStationIDs.insert(caStationIDs.begin(), caStationIDs.end());
    allUniqueStationIDs.insert(cpStationIDs.begin(), cpStationIDs.end());
    allUniqueStationIDs.erase(mHostId);

    std::unordered_set<int> seeThroughStationIDs = getNeighboursStationIDs(allObjects, "see-through");
    double divisionResult = 0.0;

    std::unordered_set<int> commonIDs;
    for (int id : allUniqueStationIDs) {
        if (seeThroughStationIDs.count(id) > 0) {
            commonIDs.insert(id);
        }
    }

    if (commonIDs.size() == seeThroughStationIDs.size()) {
        divisionResult = 1.0;
    } else if (commonIDs.empty()) {
        divisionResult = 0.0;
    } else {
        divisionResult = static_cast<double>(commonIDs.size()) / seeThroughStationIDs.size();
    }

    std::unordered_set<int> differenceIDs;
    for (int id : allUniqueStationIDs) {
        if (seeThroughStationIDs.count(id) == 0) {
            differenceIDs.insert(id);
        }
    }

    int knownVehicles = commonIDs.size();
    int wrongVehicleCount = differenceIDs.size();
    int seeThroughSize = seeThroughStationIDs.size();

    // Write the data to the CSV file
    AwarenessAHP.open("results/AHP/AwarenessAHP.csv", std::ios_base::app);
    AwarenessAHP << omnetpp::simTime() << "," << "Host ID: " << mHostId << "," << divisionResult << ","
         << seeThroughSize << "," << knownVehicles << "," << wrongVehicleCount << "," << "\n";
    AwarenessAHP.close();
}

std::vector<double> CollectivePerceptionServiceAHP::calculatePriorities(const std::vector<std::vector<double>>& comparisonMatrix, const std::vector<double>& inputs) {
    size_t n = comparisonMatrix.size();
    Eigen::MatrixXd eigenMatrix(n, n);

    // Convert the pairwise comparison matrix to an Eigen matrix
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            eigenMatrix(i, j) = comparisonMatrix[i][j];
        }
    }

    // Calculate eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> solver(eigenMatrix);
    Eigen::VectorXd eigenvalues = solver.eigenvalues().real(); // Real parts of eigenvalues
    Eigen::MatrixXd eigenvectors = solver.eigenvectors().real(); // Real parts of eigenvectors

    // Find the eigenvector corresponding to the largest eigenvalue
    size_t maxEigenvalueIndex = 0;
    for (size_t i = 1; i < eigenvalues.size(); ++i) {
        if (eigenvalues[i] > eigenvalues[maxEigenvalueIndex]) {
            maxEigenvalueIndex = i;
        }
    }

    // Extract the eigenvector corresponding to the largest eigenvalue
    Eigen::VectorXd prioritiesVector = eigenvectors.col(maxEigenvalueIndex);

    // Normalize the priorities
    double totalPriority = prioritiesVector.sum();
    for (size_t i = 0; i < n; ++i) {
        prioritiesVector[i] /= totalPriority;
    }

    // Convert Eigen vector to std::vector
    std::vector<double> priorities(n);
    for (size_t i = 0; i < n; ++i) {
        priorities[i] = prioritiesVector[i];
    }

    return priorities;
}


double CollectivePerceptionServiceAHP::haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
               std::sin(dLon / 2) * std::sin(dLon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return EARTH_RADIUS * c;
}



} // namespace artery
