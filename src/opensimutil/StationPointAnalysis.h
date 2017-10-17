#ifndef STATION_POINT_ANALYSIS_H
#define STATION_POINT_ANALYSIS_H

#include <OpenSim/Simulation/Model/Analysis.h>

#include "internal/OpenSimUtilDLL.h"

namespace OpenSim {
    /**
     * Keeps track the position, velocity and acceleration of a station point
     * defined on a body.
     *
     * @author Dimitar Stanev
     */
    class OpenSimUtil_API StationPointAnalysis : public Analysis {
        OpenSim_DECLARE_CONCRETE_OBJECT(StationPointAnalysis, Analysis);
    public:

        StationPointAnalysis(Model* model, std::string bodyName,
                             SimTK::Vec3 markerOffset);

        void setModel(Model& aModel) override;

        int begin(SimTK::State& s) override;

        int step(const SimTK::State& s, int stepNumber) override;

        int end(SimTK::State& s) override;

        int printResults(
            const std::string &aBaseName, const std::string &aDir = "",
            double aDT = -1.0, const std::string &aExtension = ".sto") override;

        int print(const std::string &path);

        void getStationState(double t, SimTK::Vec3& pos,
                             SimTK::Vec3& vel, SimTK::Vec3& acc) const;

        Storage& getStorage();

    protected:

        int record(const SimTK::State& s);

    private:

        std::string bodyName;
        SimTK::Vec3 markerOffset;
        Storage storage;

        void constructDescription();

        void constructColumnLabels();

        void setupStorage();
    }; // end of class
}; // end of namespace

#endif
