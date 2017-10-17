#include "ConstraintAnalysis.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>
#include "CommonUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

ConstraintAnalysis::ConstraintAnalysis(Model* model)
    : Analysis(model) {
    setupStorage();
}

void ConstraintAnalysis::setModel(Model& aModel) {
    Super::setModel(aModel);
    setupStorage();
}

int ConstraintAnalysis::begin(SimTK::State& s) {
    if (!proceed()) return(0);

    constructColumnLabels();
    storage.reset(s.getTime());

    int status = 0;
    if (storage.getSize() <= 0) {
        status = record(s);
    }

    return(status);
}

int ConstraintAnalysis::step(const SimTK::State& s, int stepNumber) {
    if (!proceed(stepNumber)) return(0);

    record(s);

    return(0);
}

int ConstraintAnalysis::end(SimTK::State& s) {
    if (!proceed()) return(0);

    record(s);

    return(0);
}

int ConstraintAnalysis::print(const std::string &path) {
    storage.print(path);

    return(0);
}

int ConstraintAnalysis::record(const SimTK::State& s) {
    _model->getMultibodySystem().realize(s, SimTK::Stage::Report);
    int n = _model->getNumCoordinates();
    int nb = _model->getNumBodies();
    int m = s.getNMultipliers();

    Vector data(m + 6 * nb, 0.0);

    auto lambda = _model->getMatterSubsystem().getConstraintMultipliers(s);
    data(0, m) = lambda;

    // TODO should improve the conversation of constraint forces in general
    Vector_<SpatialVec> consBodyFrc;
    Vector consMobFrc;
    _model->getMatterSubsystem().calcConstraintForcesFromMultipliers(
        s, -lambda, consBodyFrc, consMobFrc);

    consBodyFrc[0] = -consBodyFrc[0]; // Ground is "welded" at origin
    for (int i = 1; i < consBodyFrc.size(); i++) {
        auto body = _model->getBodySet()[i - 1].getMobilizedBody();
        auto p_BM = body.getOutboardFrame(s).p();
        const Rotation& R_GB = body.getBodyTransform(s).R();
        consBodyFrc[i] = shiftForceFromTo(consBodyFrc[i],
                                          Vec3(0),
					  body.getBodyTransform(s).p() + R_GB*p_BM);
        consBodyFrc[i][0] = Vec3(0);
    }
    consBodyFrc[1] = consBodyFrc[0];
    Vector temp(6 * consBodyFrc.size(), 0.0);
    for (int i = 1; i < consBodyFrc.size(); i++) {
        temp(6 * (i - 1), 3) = Vector(consBodyFrc[i][0]);
        temp(6 * (i - 1) + 3, 3) = Vector(consBodyFrc[i][1]);
    }
    data(m, 6 * nb) = temp(0, 6 * consBodyFrc.size());

    storage.append(s.getTime(), data.size(), &data[0]);

    return(0);
}

void ConstraintAnalysis::constructDescription() {
    string descrip;

    descrip = "\nThis file contains the record of custom variables\n";
    descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
    if (getInDegrees()) {
        descrip += "\nAngles are in degrees.";
    }
    else {
        descrip += "\nAngles are in radians.";
    }
    descrip += "\n\n";

    setDescription(descrip);
}

void ConstraintAnalysis::constructColumnLabels() {
    if (_model == NULL) return;
    if (getColumnLabels().getSize() > 0) return;

    Array<string> labels;
    labels.append("time");

    int NC = _model->getMatterSubsystem().getNumConstraints();
    for (int i = 0; i < NC; i++) {
        labels.append("lambda_" + changeToString(i));
    }

    for (int i = 0; i < _model->getNumBodies(); i++) {
        for (int j = 0; j < 6; j++) {
            labels.append("reaction_forces" + changeToString(i) + "_" +
                          changeToString(j));
        }
    }

    setColumnLabels(labels);
}

void ConstraintAnalysis::setupStorage() {
    setName("ConstraintAnalysis");
    storage.reset(0);
    constructDescription();
    storage.setDescription(getDescription());
}

int ConstraintAnalysis::printResults(const string &aBaseName,
                                     const string &aDir, double aDT,
				     const string &aExtension) {
    Storage::printResult(&storage, aBaseName + "_" + getName(), aDir, aDT, aExtension);

    return(0);
}

Storage& ConstraintAnalysis::getStorage() {
    return storage;
}
