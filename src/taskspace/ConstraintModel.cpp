#include "ConstraintModel.h"
#include "OpenSimUtil.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;

/* ConstraintModel ************************************************************/

const std::string ConstraintModel::CACHE_M = "cache-M";
const std::string ConstraintModel::CACHE_MInv = "cache-MInv";
const std::string ConstraintModel::CACHE_Mc = "cache-McInv";
const std::string ConstraintModel::CACHE_McInv = "cache-McInv";
const std::string ConstraintModel::CACHE_Jc = "cache-Jc";
const std::string ConstraintModel::CACHE_JcT = "cache-JcT";
const std::string ConstraintModel::CACHE_JcBar = "cache-JcBar";
const std::string ConstraintModel::CACHE_JcBarT = "cache-JcBarT";
const std::string ConstraintModel::CACHE_NcT = "cache-NcT";
const std::string ConstraintModel::CACHE_b = "cache-b";
const std::string ConstraintModel::CACHE_bc = "cache-bc";
const std::string ConstraintModel::CACHE_JcTLambda = "cache-JcTLambda";
const std::string ConstraintModel::CACHE_lambda = "cache-lambda";

ConstraintModel::ConstraintModel() {
}

ConstraintModel::~ConstraintModel() {
}

SimTK::Matrix ConstraintModel::Jc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_Jc, value, Matrix);

    _model->getMatterSubsystem().calcG(s, value);

    POST_PROCESS_CACHE(s, CACHE_Jc, value, Matrix);
}

SimTK::Matrix ConstraintModel::JcT(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JcT, value, Matrix);

    value = ~Jc(s);

    POST_PROCESS_CACHE(s, CACHE_JcT, value, Matrix);
}

SimTK::Matrix ConstraintModel::M(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_M, value, Matrix);

    _model->getMatterSubsystem().calcM(s, value);

    POST_PROCESS_CACHE(s, CACHE_M, value, Matrix);
}

SimTK::Matrix ConstraintModel::MInv(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_MInv, value, Matrix);

    _model->getMatterSubsystem().calcMInv(s, value);

    POST_PROCESS_CACHE(s, CACHE_MInv, value, Matrix);
}

SimTK::Vector ConstraintModel::b(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_b, value, Vector);

    _model->getMatterSubsystem().calcBiasForAccelerationConstraints(s, value);
    value = -1.0 * value;

    POST_PROCESS_CACHE(s, CACHE_b, value, Vector);
}

SimTK::Matrix ConstraintModel::JcBar(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JcBar, value, Matrix);

    value = ~JcBarT(s);

    POST_PROCESS_CACHE(s, CACHE_JcBar, value, Matrix);
}

SimTK::Matrix ConstraintModel::NcT(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_NcT, value, Matrix);

    value = 1 - JcT(s) * JcBarT(s);

    POST_PROCESS_CACHE(s, CACHE_NcT, value, Matrix);
}

SimTK::Matrix ConstraintModel::McInv(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_McInv, value, Matrix);

    svdInverse(Mc(s), value);

    POST_PROCESS_CACHE(s, CACHE_McInv, value, Matrix);
}

void ConstraintModel::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    Matrix mat;
    Vector vec;
    addCacheVariable(CACHE_M, mat, Stage::Position);
    addCacheVariable(CACHE_MInv, mat, Stage::Position);
    addCacheVariable(CACHE_Jc, mat, Stage::Position);
    addCacheVariable(CACHE_JcT, mat, Stage::Position);
    addCacheVariable(CACHE_b, vec, Stage::Velocity);
    addCacheVariable(CACHE_NcT, mat, Stage::Position);
    addCacheVariable(CACHE_JcBar, mat, Stage::Position);
    addCacheVariable(CACHE_JcBarT, mat, Stage::Position);
    addCacheVariable(CACHE_Mc, mat, Stage::Position);
    addCacheVariable(CACHE_McInv, mat, Stage::Position);
    addCacheVariable(CACHE_bc, vec, Stage::Velocity);
    addCacheVariable(CACHE_JcTLambda, vec, Stage::Velocity);
    addCacheVariable(CACHE_lambda, vec, Stage::Velocity);
}

/* DeSapioConstraintModel *****************************************************/

const std::string DeSapioConstraintModel::CACHE_Lc = "cache-Lc";

DeSapioConstraintModel::DeSapioConstraintModel()
    : ConstraintModel() {
}

DeSapioConstraintModel::~DeSapioConstraintModel() {
}

SimTK::Matrix DeSapioConstraintModel::Lc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_Lc, value, Matrix);

    Matrix temp = Jc(s) * MInv(s) * JcT(s);
    svdInverse(temp, value);

    POST_PROCESS_CACHE(s, CACHE_Lc, value, Matrix);
}

SimTK::Matrix DeSapioConstraintModel::JcBarT(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JcBarT, value, Matrix);

    value = Lc(s) * Jc(s) * MInv(s);

    POST_PROCESS_CACHE(s, CACHE_JcBarT, value, Matrix);
}

SimTK::Matrix DeSapioConstraintModel::Mc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_Mc, value, Matrix);

    value = M(s);

    POST_PROCESS_CACHE(s, CACHE_Mc, value, Matrix);
}

SimTK::Vector DeSapioConstraintModel::bc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_bc, value, Vector);

    value = -1.0 * JcT(s) * Lc(s) * b(s);

    POST_PROCESS_CACHE(s, CACHE_bc, value, Vector);
}

SimTK::Vector DeSapioConstraintModel::JcTLambda(const SimTK::State& s,
						const SimTK::Vector& tau,
						const SimTK::Vector& f) const {
    PRE_PROCESS_CACHE(s, CACHE_JcTLambda, value, Vector);

    value = JcT(s) * lambda(s, tau, f);

    POST_PROCESS_CACHE(s, CACHE_JcTLambda, value, Vector);
}

SimTK::Vector DeSapioConstraintModel::lambda(const SimTK::State& s,
					     const SimTK::Vector& tau,
					     const SimTK::Vector& f) const {
    PRE_PROCESS_CACHE(s, CACHE_lambda, value, Vector);

    value = JcBarT(s) * (tau - f) - Lc(s) * b(s);

    POST_PROCESS_CACHE(s, CACHE_lambda, value, Vector);
}

void DeSapioConstraintModel::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    Matrix mat;
    addCacheVariable(CACHE_Lc, mat, Stage::Position);
}

/* AghiliConstraintModel ******************************************************/

AghiliConstraintModel::AghiliConstraintModel()
    : ConstraintModel() {
}

AghiliConstraintModel::~AghiliConstraintModel() {
}

SimTK::Matrix AghiliConstraintModel::JcBar(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JcBar, value, Matrix);

    Matrix temp;
    pseudoInverse(Jc(s), true, value);

    POST_PROCESS_CACHE(s, CACHE_JcBar, value, Matrix);
}

SimTK::Matrix AghiliConstraintModel::JcBarT(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JcBarT, value, Matrix);

    value = ~JcBar(s);

    POST_PROCESS_CACHE(s, CACHE_JcBarT, value, Matrix);
}

SimTK::Matrix AghiliConstraintModel::Mc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_Mc, value, Matrix);

    value = M(s) + NcT(s) * M(s) - M(s) * NcT(s);

    POST_PROCESS_CACHE(s, CACHE_Mc, value, Matrix);
}

SimTK::Vector AghiliConstraintModel::bc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_bc, value, Vector);

    value = -1.0 * M(s) * JcBar(s) * b(s);

    POST_PROCESS_CACHE(s, CACHE_bc, value, Vector);
}

SimTK::Vector AghiliConstraintModel::JcTLambda(const SimTK::State& s,
					       const SimTK::Vector& tau,
					       const SimTK::Vector& f) const {
    PRE_PROCESS_CACHE(s, CACHE_JcTLambda, value, Vector);

    Matrix P = (1 - NcT(s));
    Matrix a = M(s) * McInv(s);
    Matrix mu = P * a;
    Vector fPer = P * f, tauPer = P * tau;
    Vector fPar = NcT(s) * f, tauPar = NcT(s) * tau;

    value = tauPer - fPer - mu * (fPar - fPar - bc(s));

    POST_PROCESS_CACHE(s, CACHE_JcTLambda, value, Vector);
}

SimTK::Vector AghiliConstraintModel::lambda(const SimTK::State& s,
					    const SimTK::Vector& tau,
					    const SimTK::Vector& f) const {
    PRE_PROCESS_CACHE(s, CACHE_lambda, value, Vector);

    value = JcBarT(s) * JcTLambda(s, tau, f);

    POST_PROCESS_CACHE(s, CACHE_lambda, value, Vector);
}

/* UnconstraintModel **********************************************************/

UnconstraintModel::UnconstraintModel()
    : ConstraintModel() {
}

UnconstraintModel::~UnconstraintModel() {
}

SimTK::Matrix UnconstraintModel::JcBar(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JcBar, value, Matrix);

    POST_PROCESS_CACHE(s, CACHE_JcBar, value, Matrix);
}

SimTK::Matrix UnconstraintModel::JcBarT(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JcBarT, value, Matrix);

    POST_PROCESS_CACHE(s, CACHE_JcBarT, value, Matrix);
}

SimTK::Matrix UnconstraintModel::Mc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_Mc, value, Matrix);

    value = M(s);

    POST_PROCESS_CACHE(s, CACHE_Mc, value, Matrix);
}

SimTK::Vector UnconstraintModel::bc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_bc, value, Vector);

    value = Vector(s.getNU(), 0.0);

    POST_PROCESS_CACHE(s, CACHE_bc, value, Vector);
}

SimTK::Matrix UnconstraintModel::NcT(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_NcT, value, Matrix);

    value = Matrix(s.getNU(), s.getNU());
    value = 1;

    POST_PROCESS_CACHE(s, CACHE_NcT, value, Matrix);
}

SimTK::Vector UnconstraintModel::JcTLambda(const SimTK::State& s,
					   const SimTK::Vector& tau,
					   const SimTK::Vector& f) const {
    PRE_PROCESS_CACHE(s, CACHE_JcTLambda, value, Vector);

    value = Vector(s.getNU(), 0.0);

    POST_PROCESS_CACHE(s, CACHE_JcTLambda, value, Vector);
}

SimTK::Vector UnconstraintModel::lambda(const SimTK::State& s,
					const SimTK::Vector& tau,
					const SimTK::Vector& f) const {
    PRE_PROCESS_CACHE(s, CACHE_lambda, value, Vector);

    value = Vector(s.getNMultipliers(), 0.0);

    POST_PROCESS_CACHE(s, CACHE_lambda, value, Vector);
}

/* DeSapioAghiliConstraintModel ***********************************************/

const std::string DeSapioAghiliConstraintModel::CACHE_Lc = "cache-Lc";

DeSapioAghiliConstraintModel::DeSapioAghiliConstraintModel()
    : ConstraintModel() {
}

DeSapioAghiliConstraintModel::~DeSapioAghiliConstraintModel() {
}

SimTK::Matrix DeSapioAghiliConstraintModel::Lc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_Lc, value, Matrix);

    Matrix temp = Jc(s) * MInv(s) * JcT(s);
    svdInverse(temp, value);

    POST_PROCESS_CACHE(s, CACHE_Lc, value, Matrix);
}

SimTK::Matrix DeSapioAghiliConstraintModel::JcBar(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JcBar, value, Matrix);

    value = ~JcBarT(s);

    POST_PROCESS_CACHE(s, CACHE_JcBar, value, Matrix);
}

SimTK::Matrix DeSapioAghiliConstraintModel::JcBarT(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_JcBarT, value, Matrix);

    value = Lc(s) * Jc(s) * MInv(s);

    POST_PROCESS_CACHE(s, CACHE_JcBarT, value, Matrix);
}

SimTK::Matrix DeSapioAghiliConstraintModel::Mc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_Mc, value, Matrix);

    value = M(s) + NcT(s) * M(s) - M(s) * NcT(s);

    POST_PROCESS_CACHE(s, CACHE_Mc, value, Matrix);
}

SimTK::Vector DeSapioAghiliConstraintModel::bc(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_bc, value, Vector);

    value = -1.0 * M(s) * JcBar(s) * b(s);

    POST_PROCESS_CACHE(s, CACHE_bc, value, Vector);
}

SimTK::Vector DeSapioAghiliConstraintModel::JcTLambda(const SimTK::State& s,
						      const SimTK::Vector& tau,
						      const SimTK::Vector& f) const {
    PRE_PROCESS_CACHE(s, CACHE_JcTLambda, value, Vector);

    Matrix P = (1 - NcT(s));
    Matrix a = M(s) * McInv(s);
    Matrix mu = P * a;
    Vector fPer = P * f, tauPer = P * tau;
    Vector fPar = NcT(s) * f, tauPar = NcT(s) * tau;

    value = tauPer - fPer - mu * (fPar - fPar - bc(s));

    POST_PROCESS_CACHE(s, CACHE_JcTLambda, value, Vector);
}

SimTK::Vector DeSapioAghiliConstraintModel::lambda(const SimTK::State& s,
						   const SimTK::Vector& tau,
						   const SimTK::Vector& f) const {
    PRE_PROCESS_CACHE(s, CACHE_lambda, value, Vector);

    value = JcBarT(s) * JcTLambda(s, tau, f);

    POST_PROCESS_CACHE(s, CACHE_lambda, value, Vector);
}

void DeSapioAghiliConstraintModel::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    Matrix mat;
    addCacheVariable(CACHE_Lc, mat, Stage::Position);
}
