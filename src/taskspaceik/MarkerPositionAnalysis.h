#ifndef MARKER_POSITION_ANALYSIS_H
#define MARKER_POSITION_ANALYSIS_H

#include <OpenSim/Simulation/Model/Analysis.h>

#include "internal/TaskSpaceIKDLL.h"

namespace OpenSim
{
    /**
     * Keeps track of the marker positions.
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceIK_API MarkerPositionAnalysis : public Analysis
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(MarkerPositionAnalysis, Analysis);
    public:

        MarkerPositionAnalysis(Model* model);

        void setModel(Model& aModel) override;

        int begin(SimTK::State& s) override;

        int step(const SimTK::State& s, int stepNumber) override;

        int end(SimTK::State& s) override;

        int printResults(
            const std::string &aBaseName, const std::string &aDir = "",
            double aDT = -1.0, const std::string &aExtension = ".sto") override;

        int print(const std::string &path);

        Storage& getStorage();

    protected:

        int record(const SimTK::State& s);

    private:

        Storage storage;

        void constructDescription();

        void constructColumnLabels();

        void setupStorage();
    }; // end of class
}; // end of namespace

#endif
