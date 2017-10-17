#include "DynamicCompensator.h"

#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

const string DynamicCompensator::CACHE_C = "cache-C";
const string DynamicCompensator::CACHE_G = "cache-G";
const string DynamicCompensator::CACHE_F = "cache-F";

DynamicCompensator::DynamicCompensator() {
}

DynamicCompensator::~DynamicCompensator() {
}

Vector DynamicCompensator::g(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_G, value, Vector);

    calcGravity(s, *_model, value);

    POST_PROCESS_CACHE(s, CACHE_G, value, Vector);
}

Vector DynamicCompensator::c(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_C, value, Vector);

    calcCoriolis(s, *_model, value);

    POST_PROCESS_CACHE(s, CACHE_C, value, Vector);
}

Vector OpenSim::DynamicCompensator::f(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_F, value, Vector);

    calcTotalGeneralizedForces(s, workingModel, value);

    value = c(s) + value;// add Coriolis because they are not computed
    //value = c(s) - g(s);// correct

    POST_PROCESS_CACHE(s, CACHE_F, value, Vector);
}

SimTK::Vector DynamicCompensator::calcJointDumping(const State& s,
                                                   double dumping) const {
    return dumping * s.getU();
}

void DynamicCompensator::addExcludedForces(OpenSim::Force* force) {
    excludedForces.push_back(force);
}

void DynamicCompensator::extendAddToSystem(MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    Vector vec;

    addCacheVariable(CACHE_C, vec, Stage::Velocity);
    addCacheVariable(CACHE_G, vec, Stage::Position);
    addCacheVariable(CACHE_F, vec, Stage::Velocity);
}

void DynamicCompensator::extendInitStateFromProperties(State& s) const {
    Super::extendInitStateFromProperties(s);

    auto self = const_cast<DynamicCompensator*>(this);
    // a model may be programmatically constructed
    if (_model->getInputFileName() == "Unassigned") {
        _model->print("temp.osim");
        _model->setInputFileName("temp.osim");
    }
    self->workingModel = Model(_model->getInputFileName());
    self->workingModel.updForceSet() = _model->updForceSet();

    for (auto force : excludedForces) {
        try {
            int index = self->workingModel.updForceSet().getIndex(
                force->getName(), 0);
            self->workingModel.updForceSet().remove(index);
        }
        catch (...) {
            cout << "Can't remove force: " << force->getName() << endl;
        }
    }

    self->workingModel.initSystem();
}
