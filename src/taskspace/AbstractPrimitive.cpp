#include "AbstractPrimitive.h"

#include <OpenSim/Simulation/Model/Model.h>

#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/* AbstractPrimitive **********************************************************/

const string AbstractPrimitive::CACHE_MInv = "cache-MInv";
const string AbstractPrimitive::CACHE_J = "cache-J";
const string AbstractPrimitive::CACHE_JP = "cache-JP";
const string AbstractPrimitive::CACHE_JT = "cache-JT";
const string AbstractPrimitive::CACHE_JPT = "cache-JPT";
const string AbstractPrimitive::CACHE_L = "cache-L";
const string AbstractPrimitive::CACHE_LP = "cache-LP";
const string AbstractPrimitive::CACHE_LInv = "cache-LInv";
const string AbstractPrimitive::CACHE_LPInv = "cache-LPInv";
const string AbstractPrimitive::CACHE_JBar = "cache-JBar";
const string AbstractPrimitive::CACHE_JPBar = "cache-JPBar";
const string AbstractPrimitive::CACHE_JBarT = "cache-JBarT";
const string AbstractPrimitive::CACHE_JPBarT = "cache-JPBarT";
const string AbstractPrimitive::CACHE_B = "cache-B";
const string AbstractPrimitive::CACHE_N = "cache-N";
const string AbstractPrimitive::CACHE_NP = "cache-NP";
const string AbstractPrimitive::CACHE_NT = "cache-NT";
const string AbstractPrimitive::CACHE_NPT = "cache-NPT";
const string AbstractPrimitive::CACHE_PT = "cache-PT";

AbstractPrimitive::AbstractPrimitive() {
}

AbstractPrimitive::~AbstractPrimitive() {
}

AbstractPrimitive::PrimitiveType AbstractPrimitive::getPrimitiveType() const {
    return primitiveType;
}

string AbstractPrimitive::getFromPrimitiveType(PrimitiveType type) const {
    if (type == TASK) {
        return "TASK";
    }
    else {
        OpenSim::Exception("Unrecognized task type", __FILE__, __LINE__);
        return "UNKNOWN";
    }
}

string AbstractPrimitive::getPrimitiveTypeAsString() const {
    if (primitiveType == TASK) {
        return "TASK";
    }
    else {
        OpenSim::Exception("Unrecognized task type", __FILE__, __LINE__);
        return "UNKNOWN";
    }
}

void AbstractPrimitive::setGoal(const SimTK::State& s, const SimTK::Vector& g) {
    xddot = g;
}

Vector AbstractPrimitive::getGoal() const {
    return xddot;
}

void AbstractPrimitive::setOverrideGoal(const Vector& g) {
    xddot = g;
}

void AbstractPrimitive::setPT(const State& s, const Matrix& pt) {
    Matrix& value = updCacheVariableValue<Matrix>(s, CACHE_PT);

    value = pt;

    markCacheVariableValid(s, CACHE_PT);
    markCacheVariableInvalid(s, CACHE_JP);
    markCacheVariableInvalid(s, CACHE_JPT);
    markCacheVariableInvalid(s, CACHE_LP);
    markCacheVariableInvalid(s, CACHE_LPInv);
    markCacheVariableInvalid(s, CACHE_JPBarT);
    markCacheVariableInvalid(s, CACHE_JPBar);
    markCacheVariableInvalid(s, CACHE_NP);
    markCacheVariableInvalid(s, CACHE_NPT);
}

void AbstractPrimitive::setMInv(const SimTK::State& s, const SimTK::Matrix& MInv) {
    Matrix& value = updCacheVariableValue<Matrix>(s, CACHE_MInv);

    value = MInv;

    markCacheVariableValid(s, CACHE_MInv);
    markCacheVariableInvalid(s, CACHE_L);
    markCacheVariableInvalid(s, CACHE_LP);
    markCacheVariableInvalid(s, CACHE_LInv);
    markCacheVariableInvalid(s, CACHE_LPInv);
    markCacheVariableInvalid(s, CACHE_JPBarT);
    markCacheVariableInvalid(s, CACHE_JPBar);
    markCacheVariableInvalid(s, CACHE_JBarT);
    markCacheVariableInvalid(s, CACHE_JBar);
    markCacheVariableInvalid(s, CACHE_NP);
    markCacheVariableInvalid(s, CACHE_NPT);
    markCacheVariableInvalid(s, CACHE_N);
    markCacheVariableInvalid(s, CACHE_NT);
}

void AbstractPrimitive::extendAddToSystem(MultibodySystem& system) const {
    ModelComponent::extendAddToSystem(system);

    Matrix mat;
    Vector vec;
    addCacheVariable(CACHE_MInv, mat, Stage::Position);
    addCacheVariable(CACHE_J, mat, Stage::Position);
    addCacheVariable(CACHE_JP, mat, Stage::Position);
    addCacheVariable(CACHE_JT, mat, Stage::Position);
    addCacheVariable(CACHE_JPT, mat, Stage::Position);
    addCacheVariable(CACHE_L, mat, Stage::Position);
    addCacheVariable(CACHE_LP, mat, Stage::Position);
    addCacheVariable(CACHE_LInv, mat, Stage::Position);
    addCacheVariable(CACHE_LPInv, mat, Stage::Position);
    addCacheVariable(CACHE_JBar, mat, Stage::Position);
    addCacheVariable(CACHE_JPBar, mat, Stage::Position);
    addCacheVariable(CACHE_JBarT, mat, Stage::Position);
    addCacheVariable(CACHE_JPBarT, mat, Stage::Position);
    addCacheVariable(CACHE_B, vec, Stage::Velocity);
    addCacheVariable(CACHE_N, mat, Stage::Position);
    addCacheVariable(CACHE_NP, mat, Stage::Position);
    addCacheVariable(CACHE_NT, mat, Stage::Position);
    addCacheVariable(CACHE_NPT, mat, Stage::Position);
    addCacheVariable(CACHE_PT, mat, Stage::Position);
}

Matrix AbstractPrimitive::JP(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JP, value, Matrix);

    value = ~JPT(s);

    POST_PROCESS_CACHE(s, CACHE_JP, value, Matrix);
}

Matrix AbstractPrimitive::JT(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JT, value, Matrix);

    value = J(s).transpose();

    POST_PROCESS_CACHE(s, CACHE_JT, value, Matrix);
}

Matrix AbstractPrimitive::JPT(const State & s) const {
    PRE_PROCESS_CACHE(s, CACHE_JPT, value, Matrix);

    value = PT(s) * JT(s);

    POST_PROCESS_CACHE(s, CACHE_JPT, value, Matrix);
}

Matrix AbstractPrimitive::L(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_L, value, Matrix);

    svdInverse(LInv(s), value);

    POST_PROCESS_CACHE(s, CACHE_L, value, Matrix);
}

Matrix AbstractPrimitive::LP(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_LP, value, Matrix);

    svdInverse(LPInv(s), value);

    POST_PROCESS_CACHE(s, CACHE_LP, value, Matrix);
}

Matrix AbstractPrimitive::LInv(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_LInv, value, Matrix);

    //calcLInv(s, *_model, J(s), JT(s), value);
    value = J(s) * MInv(s) * JT(s);

    POST_PROCESS_CACHE(s, CACHE_LInv, value, Matrix);
}

Matrix AbstractPrimitive::LPInv(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_LPInv, value, Matrix);

    //calcLInv(s, *_model, J(s), JPT(s), value);
    value = J(s) * MInv(s) * JPT(s);

    POST_PROCESS_CACHE(s, CACHE_LPInv, value, Matrix);
}

Matrix AbstractPrimitive::JBar(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JBar, value, Matrix);

    value = ~JBarT(s);

    POST_PROCESS_CACHE(s, CACHE_JBar, value, Matrix);
}

Matrix AbstractPrimitive::JPBar(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JPBar, value, Matrix);

    value = ~JPBarT(s);

    POST_PROCESS_CACHE(s, CACHE_JPBar, value, Matrix);
}

Matrix AbstractPrimitive::JBarT(const State & s) const {
    PRE_PROCESS_CACHE(s, CACHE_JBarT, value, Matrix);

    //calcJBarT(s, *_model, J(s), L(s), value);
    //calcJBarT(s, *_model, J(s), LP(s), value);
    value = LP(s) * J(s) * MInv(s);

    POST_PROCESS_CACHE(s, CACHE_JBarT, value, Matrix);
}

Matrix AbstractPrimitive::JPBarT(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JPBarT, value, Matrix);

    //calcJBarT(s, *_model, J(s), LP(s), value);
    value = JBarT(s) * PT(s);

    POST_PROCESS_CACHE(s, CACHE_JPBarT, value, Matrix);
}

Matrix AbstractPrimitive::N(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_N, value, Matrix);

    value = ~NT(s);

    POST_PROCESS_CACHE(s, CACHE_N, value, Matrix);
}

Matrix AbstractPrimitive::NP(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_NP, value, Matrix);

    value = ~NPT(s);

    POST_PROCESS_CACHE(s, CACHE_NP, value, Matrix);
}

Matrix AbstractPrimitive::NT(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_NT, value, Matrix);

    value = 1 - JPT(s) * JBarT(s);

    POST_PROCESS_CACHE(s, CACHE_NT, value, Matrix);
}

Matrix AbstractPrimitive::NPT(const State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_NPT, value, Matrix);

    value = 1 - JT(s) * JPBarT(s);

    POST_PROCESS_CACHE(s, CACHE_NPT, value, Matrix);
}

Matrix AbstractPrimitive::PT(const State& s) const {
    if (!isCacheVariableValid(s, CACHE_PT)) {
        throw OpenSim::Exception(
            "The prioritized null space matrix should be set externally",
            __FILE__, __LINE__);
    }
    return getCacheVariableValue<Matrix>(s, CACHE_PT);
}

Matrix AbstractPrimitive::MInv(const State& s) const {
    if (!isCacheVariableValid(s, CACHE_MInv)) {
        throw OpenSim::Exception(
            "The inverse inertial matrix should be set externally",
            __FILE__, __LINE__);
    }
    return getCacheVariableValue<Matrix>(s, CACHE_MInv);
}
