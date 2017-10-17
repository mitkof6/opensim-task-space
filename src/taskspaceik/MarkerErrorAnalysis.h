#ifndef MARKER_ERROR_ANALYSIS_H
#define MARKER_ERROR_ANALYSIS_H

#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Common/MarkerData.h>

#include "internal/TaskSpaceIKDLL.h"

namespace OpenSim
{
    /**
     * Keeps track of the marker error metrics.
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceIK_API MarkerErrorAnalysis : public Analysis
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(MarkerErrorAnalysis, Analysis);
    public:

        MarkerErrorAnalysis(Model* model, const MarkerData& markerData);

        void setModel(Model& aModel) override;

        int begin(SimTK::State& s) override;

        int step(const SimTK::State& s, int stepNumber) override;

        int end(SimTK::State& s) override;

        int printResults(
            const std::string &aBaseName, const std::string &aDir = "",
            double aDT = -1.0, const std::string &aExtension = ".sto") override;

        int print(const std::string &path);

        Storage& getStorage();

        double getCurrentRMS() const;

    protected:

        int record(const SimTK::State& s);

    private:

        Storage storage;

        MarkerData markerData;

        double rms;

        void constructDescription();

        void constructColumnLabels();

        void setupStorage();
    }; // end of class
}; // end of namespace

#endif
