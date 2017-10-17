#include "OpenSimUtil.h"

#include <simbody/internal/AssemblyCondition_Markers.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

#include "CommonUtil.h"

using namespace SimTK;
using namespace std;

/* I/O ************************************************************************/

void dumpMarkersFromStorage(OpenSim::Storage& markers, std::fstream& file) {
    OpenSim::Array<std::string> labels = markers.getColumnLabels();
    //cout << "Labels: " << labels.size() << endl;

    file << "PathFileType\t4\t(X/Y/Z)\tptf.trc\n"
         << "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n"
         << "60.00\t60.00\t"
         << markers.getSize() << "\t"
         << (labels.getSize() - 1) / 3
         << "\tm\t60.00\t1\t"
         << markers.getSize() << "\n"
         << "Frame#\tTime\t";

    for (int i = 1; i < labels.getSize(); i = i + 3) {
        file << labels[i].substr(0, labels[i].length() - 2) << "\t\t\t\t";
    }
    file << std::endl << "\t\t";

    int j = 1;
    for (int i = 1; i < labels.getSize(); i = i + 3) {
        int index = j;
        file << "X" << index << "\t"
             << "Y" << index << "\t"
             << "Z" << index << "\t\t";
        j++;
    }
    file << std::endl;

    //cout << "Markers: " << markers.getSize() << endl;
    double scale = 1;
    for (int i = 0; i < markers.getSize(); i++) {
        OpenSim::StateVector* sv = markers.getStateVector(i);
        OpenSim::Array<double> data = sv->getData();

        file << i + 1 << "\t" << sv->getTime() << "\t";
        for (int i = 0; i < data.size(); i = i + 3) {
            file << scale * data[i] << "\t"
                 << scale * data[i + 1] << "\t"
                 << scale * data[i + 2] << "\t" << "\t";
        }
        file << std::endl;
    }
}

SimTK::Matrix readMatFile(std::string path) {
    SimTK::Matrix data;
    std::vector< std::vector<double> > temp;

    std::fstream f(path);
    if (!f.is_open()) {
        throw OpenSim::Exception("Can't open the file: " + path, __FILE__, __LINE__);
    }

    while (!f.eof()) {
        std::string line;
        std::getline(f, line);
        std::stringstream stream(line);

        double value;
        std::vector<double> row;
        while (stream >> value) {
            row.push_back(value);
        }

        if (row.size() != 0) {
            temp.push_back(row);
        }
    }

    data.resize(temp.size(), temp[0].size());
    for (unsigned int i = 0; i < temp.size(); i++) {
        for (unsigned int j = 0; j < temp[0].size(); j++) {
            data.set(i, j, temp[i].at(j));
        }
    }

    return data;
}

SimTK::Matrix readCSVFile(std::string path, int skip) {
    SimTK::Matrix data;
    std::vector< std::vector<double> > temp;

    std::fstream f(path);
    if (!f.is_open()) {
        throw OpenSim::Exception("Can't open the file: " + path, __FILE__, __LINE__);
    }

    int s = 0;
    while (!f.eof()) {
        std::string line;
        std::getline(f, line);

        if (s < skip) {
            s++;
            continue;
        }

        std::stringstream stream(line);
        std::string field;

        std::vector<double> row;
        while (std::getline(stream, field, ',')) {
            std::stringstream fs(field);
            double value = 0.0;
            fs >> value;

            row.push_back(value);
        }

        if (row.size() != 0) {
            temp.push_back(row);
        }
    }

    data.resize(temp.size(), temp[0].size());
    for (unsigned int i = 0; i < temp.size(); i++) {
        for (unsigned int j = 0; j < temp[0].size(); j++) {
            data.set(i, j, temp[i].at(j));
        }
    }

    return data;
}

void exportBodyGraphviz(OpenSim::Model& model, std::string path) {
    std::ofstream f;
    f.open(path);

    if (!f.is_open()) {
        throw OpenSim::Exception("Can't open the file: " + path, __FILE__, __LINE__);
    }

    OpenSim::JointSet js = model.getJointSet();

    f << "digraph bodyGraph {\n";
    for (int i = 0; i < js.getSize(); i++) {
        f << "\t" << js.get(i).getParentFrame().getName() << " -> "
            " [label=\"" << js.get(i).getName() << "\"];\n";
    }
    f << "}";

    f.close();
}

void exportMuscleGraphviz(OpenSim::Model& model, std::string path) {
    std::ofstream f;
    f.open(path);

    if (!f.is_open()) {
        throw OpenSim::Exception("Can't open the file: " + path, __FILE__, __LINE__);
    }

    OpenSim::Set<OpenSim::Muscle> ms = model.getMuscles();

    f << "digraph muscleGraph {\n";
    for (int i = 0; i < ms.getSize(); i++) {
        OpenSim::PathPointSet ps = ms.get(i).getGeometryPath().getPathPointSet();

        for (int j = 0; j < ps.getSize(); j++) {
            if (j > 0) {
                if (ps.get(j).getBodyName() == ps.get(j - 1).getBodyName()) {
                    continue;
                }
            }

            f << "\t" << ms.get(i).getName() << " -> "
              << ps.get(j).getBodyName() << ";\n";
        }
    }
    f << "}";

    f.close();
}

void exportVector(SimTK::Vector vec, std::string path, std::ios_base::openmode mode) {
    std::ofstream f;
    f.open(path, mode);

    if (!f.is_open()) {
        throw OpenSim::Exception("Can't open the file: " + path, __FILE__, __LINE__);
    }

    f << std::left;
    for (int i = 0; i < vec.size(); i++) {
        f << vec[i] << std::setw(4) << "\t";
    }
    f << "\n";

    f.close();
}

/* Conversations **************************************************************/

Vector spatialToVector(const SpatialVec& spatial) {
    Vec3 o = spatial[0];
    Vec3 p = spatial[1];
    Vector vec(6, 0.0);
    vec[0] = o[0];
    vec[1] = o[1];
    vec[2] = o[2];
    vec[3] = p[0];
    vec[4] = p[1];
    vec[5] = p[2];

    return vec;
}

Vector toVector(const Vec3& o, const Vec3& p) {
    Vector spatial(6, 0.0);
    spatial[0] = o[0];
    spatial[1] = o[1];
    spatial[2] = o[2];
    spatial[3] = p[0];
    spatial[4] = p[1];
    spatial[5] = p[2];

    return spatial;
}

Vector toDeg(const Vector& v, int first, int last) {
    Vector deg = v;
    for (int i = first; i <= last; i++) {
        deg[i] = SimTK::convertRadiansToDegrees(v[i]);
    }
    return deg;
}

Mat33 toMat33(Matrix T) {
    assert(T.ncol() == 3 && T.nrow() == 3);

    Mat33 t;

    t[0][0] = T[0][0];
    t[0][1] = T[0][1];
    t[0][2] = T[0][2];
    t[1][0] = T[1][0];
    t[1][1] = T[1][1];
    t[1][2] = T[1][2];
    t[2][0] = T[2][0];
    t[2][1] = T[2][1];
    t[2][2] = T[2][2];

    return t;
}

double timeToSpilineTime(double t, SimTK::Vector& time) {
    int timeIndex = 0;
    for (int i = time.size() - 1; i >= 0; i--) {
        if (t >= time[i]) {
            timeIndex = i;
            break;
        }
    }

    if (timeIndex >= (time.size() - 1)) {
        return timeIndex;
    }
    else {
        double t0 = time[timeIndex];
        double t1 = time[timeIndex + 1];

        double interval = t1 - t0;
        double offset = t - t0;

        return timeIndex + offset / interval;
    }
}

SimTK::Quaternion convertMatrixToQuaternion(const SimTK::Rotation & R) {
    double qw, qx, qy, qz;
    double tr = R[0][0] + R[1][1] + R[2][2];

    if (tr > 0) {
        double S = sqrt(tr + 1.0) * 2; // S=4*qw
        qw = 0.25 * S;
        qx = (R[2][1] - R[1][2]) / S;
        qy = (R[0][2] - R[2][0]) / S;
        qz = (R[1][0] - R[0][1]) / S;
    }
    else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        double S = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2; // S=4*qx
        qw = (R[2][1] - R[1][2]) / S;
        qx = 0.25 * S;
        qy = (R[0][1] + R[1][0]) / S;
        qz = (R[0][2] + R[2][0]) / S;
    }
    else if (R[1][1] > R[2][2]) {
        double S = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2; // S=4*qy
        qw = (R[0][2] - R[2][0]) / S;
        qx = (R[0][1] + R[1][0]) / S;
        qy = 0.25 * S;
        qz = (R[1][2] + R[2][1]) / S;
    }
    else {
        double S = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2; // S=4*qz
        qw = (R[1][0] - R[0][1]) / S;
        qx = (R[0][2] + R[2][0]) / S;
        qy = (R[1][2] + R[2][1]) / S;
        qz = 0.25 * S;
    }
    SimTK::Quaternion q(qw, qx, qy, qz);
    q.normalize();

    return q;
}

OpenSim::Output<double>* toOutput(const OpenSim::AbstractOutput& abstractOutput) {
    return &const_cast<OpenSim::Output<double> &>
        (OpenSim::Output<double>::downcast(abstractOutput));
}

void connectComponents(
    const OpenSim::ModelComponent* from, std::string fromOutput,
    OpenSim::ModelComponent* to, std::string toInput) {
    std::cout << "Component: "
              << from->getName() << " with output: "
              << fromOutput
              << " connects to: " << to->getName()
              << " with input: " << toInput
              << " at stage: "
              << to->getInput(toInput).getConnectAtStage() << std::endl;

    //throw OpenSim::Exception("Unimplemented new OpenSim", __FILE__, __LINE__);
    to->updInput(toInput).connect(from->getOutput(fromOutput));
}

/* Calculations ***************************************************************/

void svdInverse(const Matrix& m, Matrix& mInv) {
    FactorSVD svd(m);
    svd.inverse(mInv);
}

void pseudoInverse(const Matrix& m, Matrix& mInv) {
    Matrix mT = m.transpose();
    FactorSVD svd(m * mT);
    svd.inverse(mInv);
    mInv = mT * mInv;
}

void robustInverse(const Matrix& m, Matrix& mInv) {
    Matrix U, V;
    Vector S;
    FactorSVD svd(m);
    svd.getSingularValuesAndVectors(S, U, V);

    /*
      Traditionally, inverse svd is performed as follows:
      A^-1 = v * s^-1 * u^T
      A[mxn], v[nxn], u[mxm], s[mxn]
      Simbody returns u, s, v^T, so v must be transposed
    */

    Matrix SInv(V.nrow(), U.ncol(), 0.0);
    for (int i = 0; i < S.size(); i++) {
        if (S[i] < 0.05) {
            SInv[i][i] = 0;
        }
        else {
            SInv[i][i] = 1.0 / S[i];
        }
    }
    /*cout << calcConditionNumber(m);
      cout << ~V << endl;
      cout << SInv << endl;
      cout << ~U << endl;*/
    //cout << SInv << endl;

    mInv = ~V * SInv * ~U;
    //cout << mInv << endl;
}

void pseudoInverse(const Matrix& m, bool left, Matrix& mInv) {
    if (left) {
        Matrix temp = m * ~m, tempInv;
        svdInverse(temp, tempInv);
        mInv = ~m * tempInv;
    }
    else {
        Matrix temp = ~m * m, tempInv;
        svdInverse(temp, tempInv);
        mInv = tempInv * ~m;
    }
}

double calcConditionNumber(const Matrix& mat) {
    FactorSVD svd(mat);
    Vector eigvalues;
    svd.getSingularValues(eigvalues);

    return max(eigvalues) / min(eigvalues);
}

void calcPositionJacobian(const SimTK::State& s, const OpenSim::Model& model,
                          const std::string& body, const SimTK::Vec3& offset,
                          SimTK::Matrix& J) {
    model.getMatterSubsystem().calcStationJacobian(
        s, model.getBodySet().get(body).getMobilizedBodyIndex(), offset, J);
}

void calcOrientationJacobian(const SimTK::State& s, const OpenSim::Model& model,
                             const std::string& body, const SimTK::Vec3& offset,
                             SimTK::Matrix& J) {
    Matrix temp;
    model.getMatterSubsystem().calcFrameJacobian(
        s, model.getBodySet().get(body).getMobilizedBodyIndex(), offset, temp);

    J = temp(0, 0, 3, temp.ncol());
}

void calcFrameJacobian(const SimTK::State& s, const OpenSim::Model& model,
                       const std::string& body, const SimTK::Vec3& offset,
                       SimTK::Matrix& J) {
    model.getMatterSubsystem().calcFrameJacobian(
        s, model.getBodySet().get(body).getMobilizedBodyIndex(), offset, J);
}

void calcGravity(const SimTK::State& s, const OpenSim::Model& model,
                 SimTK::Vector& g) {
    model.getMatterSubsystem().multiplyBySystemJacobianTranspose(
        s, model.getGravityForce().getBodyForces(s), g);
}

void calcCoriolis(const SimTK::State& s, const OpenSim::Model& model,
                  SimTK::Vector& c) {
    model.getMatterSubsystem().calcResidualForceIgnoringConstraints(
        s, SimTK::Vector(0), SimTK::Vector_<SimTK::SpatialVec>(0),
        SimTK::Vector(0), c);
}

void calcTotalGeneralizedForces(const State& s, const OpenSim::Model& model,
                                Vector& f) {
    // update working state
    State workingState = model.getWorkingState();
    workingState.updTime() = s.getTime();
    workingState.updQ() = s.getQ();
    workingState.updU() = s.getU();
    workingState.updZ() = s.getZ();
    workingState.updY() = s.getY();

    // disable any actuators when computing the total force
    const OpenSim::Set<OpenSim::Actuator>& as = model.getActuators();
    for (int i = 0; i < as.getSize(); i++) {
        as[i].setDisabled(workingState, true);
    }

    // generalized forces and torques should be accessed at a Dynamics stage
    model.realizeDynamics(workingState);

    const Vector_<SpatialVec>& forces = model.getMultibodySystem()
        .getRigidBodyForces(workingState, Stage::Dynamics);
    const Vector& torques = model.getMultibodySystem().getMobilityForces(
        workingState, Stage::Dynamics);

    model.getMatterSubsystem().multiplyBySystemJacobianTranspose(workingState,
                                                                 forces, f);

    // in our conviction we subtract their contribution
    f = -1.0 * torques - f;
}

void calcPositionJDotQDot(const SimTK::State& s, const OpenSim::Model& model,
                          const std::string& body, const SimTK::Vec3& offset,
                          SimTK::Vec3& Jdotu) {
    Jdotu = model.getMatterSubsystem().calcBiasForStationJacobian(
        s, model.getBodySet().get(body).getMobilizedBodyIndex(), offset);
}

void calcOrientationJDotQDot(const SimTK::State& s, const OpenSim::Model& model,
                             const std::string& body, const SimTK::Vec3& offset,
                             SimTK::Vec3& Jdotu) {
    SpatialVec jdu = model.getMatterSubsystem().calcBiasForFrameJacobian(
        s, model.getBodySet().get(body).getMobilizedBodyIndex(), offset);

    Jdotu = jdu[0];
}

void calcFrameJDotQDot(const SimTK::State& s, const OpenSim::Model& model,
                       const std::string& body, const SimTK::Vec3& offset,
                       SimTK::Vector& Jdotu) {
    SpatialVec jdu = model.getMatterSubsystem().calcBiasForFrameJacobian(
        s, model.getBodySet().get(body).getMobilizedBodyIndex(), offset);

    Jdotu = spatialToVector(jdu);
}

void calcLInv(const SimTK::State& s, const OpenSim::Model& model,
              const SimTK::Matrix& J, const SimTK::Matrix& JT, SimTK::Matrix& LInv) {
    int t = J.nrow();

    // Create temporary variables.
    int nu = s.getNU();
    Vector JTCol(nu);
    Vector MInvJTCol(nu);
    LInv.resize(t, t);

    // f_GP is used to pluck out one column at a time of Jt. Exactly one
    // element at a time of f_GP will be 1, the rest are 0.
    Vector f_GP(t, Real(0));

    for (int i = 0; i < t; i++) {
        // select i-th column
        f_GP[i] = 1;
        JTCol = JT * f_GP;
        f_GP[i] = 0;

        // M^{-1} * J^{T} per column
        model.getMatterSubsystem().multiplyByMInv(s, JTCol, MInvJTCol);

        // J * rest
        LInv(i) = J * MInvJTCol;
    }
}

void calcJBarT(const SimTK::State& s, const OpenSim::Model& model,
               const SimTK::Matrix& J, const SimTK::Matrix& L, SimTK::Matrix& JBarT) {
    Matrix MInv;
    model.getMatterSubsystem().calcMInv(s, MInv);

    JBarT = L * J * MInv;
}

void calcActuatorIndicies(const SimTK::State& s,
                          const OpenSim::Model& model,
                          OpenSim::Array<int>& activeActuatorIndicies,
                          OpenSim::Array<int>& disabledActuatorIndicies) {
    // muscle indicies
    activeActuatorIndicies.setSize(0);
    disabledActuatorIndicies.setSize(0);
    const OpenSim::Set<OpenSim::Actuator>& ac = model.getActuators();
    for (int i = 0; i < ac.getSize(); i++) {
        const OpenSim::PathActuator* act =
            dynamic_cast<const OpenSim::PathActuator*>(&ac[i]);
        if (act && !act->isDisabled(s)) {
            activeActuatorIndicies.append(i);
        }

        if (act && act->isDisabled(s)) {
            disabledActuatorIndicies.append(i);
        }
    }
}

void calcCoordinateIndicies(const SimTK::State& s,
                            const OpenSim::Model& model,
                            OpenSim::Array<int>& activeCoordinateIndicies,
                            OpenSim::Array<int>& constrainedCoordinateIndicies) {
    // coordinate indicies
    activeCoordinateIndicies.setSize(0);
    constrainedCoordinateIndicies.setSize(0);
    const OpenSim::CoordinateSet& cs = model.getCoordinateSet();
    for (int i = 0; i < cs.getSize(); i++) {
        const OpenSim::Coordinate& coord = cs[i];
        if (!coord.isConstrained(s)) {
            activeCoordinateIndicies.append(i);
        }
        else {
            constrainedCoordinateIndicies.append(i);
        }
    }
}

void calcMaxActuatorForce(const SimTK::State& s, const OpenSim::Model& model,
                          const OpenSim::Array<int>& activeActuatorIndicies,
                          SimTK::Vector& Fmax) {
    // compute max force
    const OpenSim::Set<OpenSim::Actuator>& ac = model.getActuators();
    Fmax.resize(activeActuatorIndicies.getSize());
    for (int m = 0; m < activeActuatorIndicies.getSize(); m++) {
        // get as path actuators (general case)
        OpenSim::PathActuator* act =
            dynamic_cast<OpenSim::PathActuator*>(
                &ac[activeActuatorIndicies[m]]);
        Fmax[m] = act->getOptimalForce();

        // if the actuator is a muscle specialize
        OpenSim::Muscle* mus = dynamic_cast<OpenSim::Muscle*>(
            &ac[activeActuatorIndicies[m]]);
        if (mus) {
            Fmax[m] = mus->getMaxIsometricForce();
        }
    }
}

void calcMomentArm(const SimTK::State& s, const OpenSim::Model& model,
                   const OpenSim::Array<int>& coordinateIndicies,
                   const OpenSim::Array<int>& actuatorIndicies,
                   SimTK::Matrix& R) {
    const OpenSim::CoordinateSet& cs = model.getCoordinateSet();
    const OpenSim::Set<OpenSim::Actuator>& ac = model.getActuators();

    R.resize(coordinateIndicies.size(), actuatorIndicies.size());
    for (int n = 0; n < coordinateIndicies.size(); n++) {
        for (int m = 0; m < actuatorIndicies.size(); m++) {
            OpenSim::PathActuator* mus =
                dynamic_cast<OpenSim::PathActuator*>(&ac[actuatorIndicies[m]]);
            R[n][m] = mus->computeMomentArm(s, cs[coordinateIndicies[n]]);
        }
    }
}

SimTK::Vector calcActuatorForce(const SimTK::State& s,
                                const OpenSim::Model& model,
                                const OpenSim::Array<int>& muscleIndicies,
                                const SimTK::Vector& activations) {
    const OpenSim::Set<OpenSim::Actuator>& ac = model.getActuators();
    Vector force(muscleIndicies.size());
    for (int i = 0; i < muscleIndicies.getSize(); i++) {
        // if path actuator
        OpenSim::PathActuator* act = dynamic_cast<OpenSim::PathActuator*>(
            &ac[muscleIndicies[i]]);
        if (act) {
            force[i] = activations[i] * act->getOptimalForce();
        }

        // if muscle
        OpenSim::Muscle* mus = dynamic_cast<OpenSim::Muscle*>(&ac[muscleIndicies[i]]);
        if (mus) {
            throw OpenSim::Exception(
                "Model contains muscles, currently there is not support"
                + string(" due to dynamics realization problem"),
		__FILE__, __LINE__);
            //model.realizeDynamics(s);
            force[i] = mus->getMaxIsometricForce() *
                (activations[i] *
                 mus->getActiveForceLengthMultiplier(s) *
                 mus->getForceVelocityMultiplier(s) +
                 mus->getPassiveForceMultiplier(s)) * mus->getCosPennationAngle(s);
        }
    }
    return force;
}

/* Misc **********************************************************************/

Vec3 calcCentroid(const std::vector<Vec3>& A) {
    Vec3 centroid(0);

    for (Vec3 point : A) {
        centroid += point;
    }
    centroid = (1.0 / A.size()) * centroid;
    return centroid;
}

Transform findTransformation(const std::vector<Vec3>& A, const std::vector<Vec3>& B) {
    if (A.size() != B.size()) {
        throw OpenSim::Exception("A and B point set must have the same dimension",
                                 __FILE__, __LINE__);
    }

    Mat33 R;
    Vec3 t;
    if (A.size() == 1) {
        Vec3 cA = calcCentroid(A);
        Vec3 cB = calcCentroid(B);

        R = 1;
        t = cB - cA;
    }
    else if (A.size() == 2) {
        cout << "Warning case not implemented" << endl;
        throw OpenSim::Exception("Undefined response in case of two points",
				 __FILE__, __LINE__);

        Vec3 cA = calcCentroid(A);
        Vec3 cB = calcCentroid(B);

        R = 1;
        t = cB - cA;
    }
    else {
        Vec3 cA = calcCentroid(A);
        Vec3 cB = calcCentroid(B);

        //std::cout << cA << std::endl;
        //std::cout << cB << std::endl;

        Matrix H(3, 3, 0.0);

        for (int i = 0; i < A.size(); i++) {
            //std::cout << A[i] << " - " << B[i] << std::endl;
            H += Vector(A[i] - cA) * ~Vector(B[i] - cB);
        }

        //std::cout << "H: " << H << std::endl;

        Matrix U, V;
        Vector S;
        FactorSVD svd(H);
        svd.getSingularValuesAndVectors(S, U, V);

        /*std::cout << "U: " << U << std::endl;
          std::cout << "S: " << S << std::endl;
          std::cout << "V: " << ~V << std::endl;*/

        Matrix RR = ~V * ~U;
        //std::cout << "RR: " << std::setprecision(2) << RR << std::endl;

        // handle reflexion case
        if (SimTK::det(toMat33(RR)) < 0) {
            //std::cout << "reflexion" << std::endl;

            //std::cout << ~V << std::endl;

            V[2][0] = -1 * V[2][0];
            V[2][1] = -1 * V[2][1];
            V[2][2] = -1 * V[2][2];

            //std::cout << ~V << std::endl;

            RR = ~V * ~U;
            //std::cout << SimTK::det(toMat33(RR)) << std::endl;
        }

        R = toMat33(RR);

        //std::cout << "R: " << std::setprecision(2) << R << std::endl;

        t = -R * cA + cB;

        //std::cout << "T: " << t << std::endl;
    }

    return Transform(Rotation(R), t);
}

void calcInitialState(State& s, const OpenSim::Model& model,
                      const OpenSim::MarkerData& markerData) {
    SimTK::Assembler ik(model.getMultibodySystem());

    const OpenSim::MarkerSet& ms = model.getMarkerSet();

    SimTK::Markers* markers = new SimTK::Markers();
    std::map<std::string, Markers::MarkerIx> markerToIndex;
    Array_<Markers::MarkerIx> markerIx;
    for (int i = 0; i < ms.getSize(); i++) {
        Markers::MarkerIx index = markers->addMarker(
            model.getBodySet().get(ms[i].getFrameName()).getMobilizedBodyIndex(),
            ms[i].get_location());
        markerToIndex.insert(std::pair<std::string, Markers::MarkerIx>(
				 ms[i].getName(), index));
        markerIx.push_back(index);
    }
    markers->defineObservationOrder(markerIx);
    ik.adoptAssemblyGoal(markers);

    int firstFrame, lastFrame;
    markerData.findFrameRange(s.getTime(), s.getTime() + 0.1, firstFrame, lastFrame);
    OpenSim::MarkerFrame frame = markerData.getFrame(firstFrame);
    for (auto marker : markerToIndex) {
        int index = markerData.getMarkerIndex(marker.first);
        if (index != -1) // if marker exist in marker data
        {
            Vec3 pos = frame.getMarker(index);
            markers->moveOneObservation(
                markers->getObservationIxForMarker(marker.second), pos);
        }
    }

    ik.initialize(s);
    ik.assemble(s);
}

void setUseMuscles(State& s, const OpenSim::Model& model, bool useMuscles) {
    for (int i = 0; i < model.getMuscles().getSize(); i++) {
        model.getMuscles().get(i).setDisabled(s, !useMuscles);
    }
}

void setIgnoreActivationDynamics(const OpenSim::Model& model, bool ignore) {
    throw OpenSim::Exception("Don't use this method with any muscle model",
			     __FILE__, __LINE__);
    for (int i = 0; i < model.getMuscles().getSize(); i++) {
        model.getMuscles().get(i).set_ignore_activation_dynamics(ignore);
    }
}

double distance(SimTK::Vec3 a, SimTK::Vec3 b) {
    return sqrt(distanceSq(a, b));
}

double distanceSq(SimTK::Vec3 a, SimTK::Vec3 b) {
    double dist = 0;

    for (int i = 0; i < a.size(); i++) {
        dist += pow(a[i] - b[i], 2);
    }

    return dist;
}

void simulate(SimTK::State& s, OpenSim::Model& model, SimTK::Integrator& integrator,
              double t0, double tf, OpenSim::Storage& storage) {
    //manager
    OpenSim::Manager manager(model, integrator);

    //integrate
    auto t = startClock("Integrating...", true);
    manager.setInitialTime(t0);
    manager.setFinalTime(tf);
    manager.integrate(s);
    stopClock(t, "Finish integration", true);

    //state results
    storage = OpenSim::Storage(manager.getStateStorage());
    storage.setColumnLabels(manager.getStateStorage().getColumnLabels());
    model.updSimbodyEngine().convertRadiansToDegrees(storage);
}

void addReserveActuators(OpenSim::Model& model,
                         std::vector<std::string>& reserveActuatorNames,
                         std::vector<int>& reserveActuatorIndecies) {
    const OpenSim::CoordinateSet& cs = model.getCoordinateSet();
    OpenSim::ForceSet& fs = model.updForceSet();
    for (int i = 0; i < cs.getSize(); i++) {
        std::string name = cs.get(i).getName() + "_reserve";
        reserveActuatorNames.push_back(name);
        if (fs.contains(name)) {
            int index = fs.getIndex(name, 0);
            reserveActuatorIndecies.push_back(index);
        }
        else {
            OpenSim::CoordinateActuator* actuator = new OpenSim::CoordinateActuator();
            actuator->setCoordinate(&cs.get(i));
            actuator->setName(name);
            actuator->setOptimalForce(1);
            actuator->setMinControl(-SimTK::Infinity);
            actuator->setMaxControl(SimTK::Infinity);
            fs.append(actuator);

            int index = fs.getIndex(name, 0);
            reserveActuatorIndecies.push_back(index);
        }
    }
}
