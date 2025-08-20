/*
* Artery V2X Simulation Framework
* Copyright 2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_COLLECTIVEPERCEPTIONSERVICEAHP_H_V08YXH9S
#define ARTERY_COLLECTIVEPERCEPTIONSERVICEAHP_H_V08YXH9S

#include "artery/envmod/service/CollectivePerceptionMockMessage.h"
#include "artery/application/ItsG5Service.h"
#include "artery/networking/PositionProvider.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include <unordered_set>
#include <vector>

namespace artery
{
    struct ObjectData {
    omnetpp::simtime_t creationTime;
    int objectId;
};

struct Point {
    double latitude;
    double longitude;
    double value;
};

const double EARTH_RADIUS = 6371000; // Earth's radius in meters

class CollectivePerceptionServiceAHP : public ItsG5Service
{
    public:
        virtual ~CollectivePerceptionServiceAHP();

    protected:
        int numInitStages() const override;
        void initialize(int stage) override;
        void trigger() override;
        void handleMessage(omnetpp::cMessage*) override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
        void generatePacket();
        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
        void printSensorObjectList(const std::string& title, const TrackedObjectsFilterRange&);
        std::unordered_set<int> getNeighboursStationIDs(const LocalEnvironmentModel::TrackedObjects& objs, const std::string& category);
        void calculateAwareness();
        double calculateWeightedSum(const std::vector<std::vector<double>>& , const std::vector<double>& );
        std::vector<double> calculatePriorities(const std::vector<std::vector<double>>& , const std::vector<double>& );
        double haversineDistance(double , double , double , double );
        


    private:
        int uniqueness = 0;
        double hotspot=0;
        int radarSize, camSize, seeThroughSize, detectedVehciles = 0;
        double awareness_ratio = 0.0;
        double uniqueness_ratio=0.0;
        std::unordered_set<int> prevMergedStationIDs;
        std::unordered_set<int> STprevMergedStationIDs;
        std::unordered_set<int> cpStationIDs;   
        std::vector<ObjectData> dataMatrix;
        std::unordered_set<int> vehicleNumbersSet;
        double output = 0.0;
        omnetpp::cMessage *awarenessTimer;

        
        int mHostId = 0;
        std::string mEgoId;
        const PositionProvider* mPositionProvider = nullptr;
        const LocalEnvironmentModel* mEnvironmentModel = nullptr;
        LocalEnvironmentModel* mLocalEnvironmentModel;
        omnetpp::cMessage* mTrigger = nullptr;
        bool mGenerateAfterCam;
        omnetpp::SimTime delay;
        omnetpp::SimTime mCpmOffset;
        omnetpp::SimTime mCpmInterval;
        omnetpp::SimTime CpmValidityInterval;
        omnetpp::SimTime mFovInterval = omnetpp::SimTime::ZERO;
        omnetpp::SimTime mFovLast = omnetpp::SimTime::ZERO;
        std::vector<CollectivePerceptionMockMessage::FovContainer> mFovContainers;
        std::unordered_set<const Sensor*> mSensors;
        unsigned mDccProfile = 0;
        unsigned mLengthHeader = 0;
        unsigned mLengthFovContainer = 0;
        unsigned mLengthObjectContainer = 0;
        omnetpp::SimTime lastCpmSentTime = 0;
	bool useDynamicDissemination;
	bool receivedFromStationType1Recently;
	omnetpp::SimTime lastStationType1Reception;
 
        
        // New struct to track last known object state
	struct LastKnownObjectState {
	    double x;
	    double y;
	    double speed;
	    double heading;
	    omnetpp::SimTime lastIncluded;
	};

	// Add to class definition (ideally in header, added here for clarity)
	std::unordered_map<int, LastKnownObjectState> lastKnownObjects;
        
        const VehicleDataProvider* mVehicleDataProvidernew = nullptr;
        int stationType=0;
};

} // namespace artery

#endif /* ARTERY_COLLECTIVEPERCEPTIONSERVICEAHP_H_V08YXH9S */
