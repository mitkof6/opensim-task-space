#include "IKTrackingTask.h"

#include <OpenSim/Simulation/Model/Model.h>

#include "IKTaskData.h"
#include "OpenSimUtil.h"

using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Vector;
using SimTK::Transform;
using std::vector;
using std::cout;
using std::endl;

/* IKTrackingTask *************************************************************/

IKTrackingTask::IKTrackingTask(const Array<double>& t)
{
    time = Vector(t.size(), &t[0]);

    firstDerivative.push_back(0);

    secondDerivative.push_back(0);
    secondDerivative.push_back(0);
}

IKTrackingTask::~IKTrackingTask()
{
}

void IKTrackingTask::calcTransitions(const SimTK::Transform& T0,
                                     tree_node_<IKTaskData*>& current)
{
    vector<Transform> transitions;

    for (int n = 0; n < time.size() - 1; n++)
    {
        // get marker points at n and n + 1
        double tCurr = time[n];
        double tNext = time[n + 1];
        vector<Vec3> pCurr, pNext;
        for (int i = 0; i < smoothSplines.size(); i++)
        {
            Vec3 xa = smoothSplines[i].calcValue(Vector(1, tCurr));
            pCurr.push_back(xa);

            Vec3 xb = smoothSplines[i].calcValue(Vector(1, tNext));
            pNext.push_back(xb);
        }

        if (pCurr.size() == 2)
        {
            if (n == 0)
            {
                pCurr.push_back(T0.p());
                pNext.push_back(T0.p());
            }
            else
            {
                pCurr.push_back(transitions[n - 1].p());
                pNext.push_back(transitions[n - 1].p());
            }
        }

        Transform Tn = findTransformation(pCurr, pNext);
        transitions.push_back(Tn);
    }

    current.data->setTransitions(transitions);
}

/* IKPositionTrackingTask *****************************************************/

IKPositionTrackingTask::IKPositionTrackingTask(const Array<double>& t,
                                               OpenSim::Function* x,
                                               OpenSim::Function* y,
                                               OpenSim::Function* z)
    : IKTrackingTask(t)
{
    SimTK::Vector_<Vec3> marker(t.size());
    for (int i = 0; i < t.size(); i++)
    {
        marker[i] = Vec3(
            x->calcValue(Vector(1, time[i])),
            y->calcValue(Vector(1, time[i])),
            z->calcValue(Vector(1, time[i])));
    }

    SimTK::SplineFitter<Vec3> fitter =
        SimTK::SplineFitter<Vec3>::fitFromGCV(5, time, marker);
    SimTK::Spline_<Vec3> spline = fitter.getSpline();
    smoothSplines.push_back(spline);
}

IKPositionTrackingTask::~IKPositionTrackingTask()
{
}

void IKPositionTrackingTask::calcGoal(double t, Vector& x,
                                      Vector& u, Vector& a)
{
    Vec3 xd = smoothSplines[0].calcValue(Vector(1, t));
    Vec3 ud = smoothSplines[0].calcDerivative(firstDerivative, Vector(1, t));
    Vec3 ad = smoothSplines[0].calcDerivative(secondDerivative, Vector(1, t));

    x = Vector(xd);
    u = Vector(ud);
    a = Vector(ad);
}

void IKPositionTrackingTask::calcTransitions(const SimTK::Transform& T0,
                                             tree_node_<IKTaskData*>& current)
{
    Super::calcTransitions(T0, current);
}

/* IKOrientationTrackingTask **************************************************/

IKOrientationTrackingTask::IKOrientationTrackingTask(
    const OpenSim::Array<double>& t,
    const OpenSim::GCVSplineSet& markerSplineSet, const std::vector<int>& indicies)
    : IKTrackingTask(t)
{
    for (int ind : indicies)
    {
        SimTK::Vector_<Vec3> marker(t.size());
        for (int i = 0; i < t.size(); i++)
        {
            marker[i] = Vec3(
                markerSplineSet[3 * ind].calcValue(Vector(1, time[i])),
                markerSplineSet[3 * ind + 1].calcValue(Vector(1, time[i])),
                markerSplineSet[3 * ind + 2].calcValue(Vector(1, time[i])));
        }
        SimTK::SplineFitter<Vec3> fitter =
            SimTK::SplineFitter<Vec3>::fitFromGCV(5, time, marker);
        SimTK::Spline_<Vec3> spline = fitter.getSpline();
        smoothSplines.push_back(spline);
    }
}

IKOrientationTrackingTask::~IKOrientationTrackingTask()
{
    delete spline;
}

void IKOrientationTrackingTask::calcGoal(double t, Vector& x, Vector& v, Vector& a)
{
    // orientation goal
    double dt = 0.01;
    double ti = timeToSpilineTime(t, time);
    double tplus = timeToSpilineTime(t + dt, time);
    double tminus = timeToSpilineTime(t - dt, time);

    /*cout << "ti: " << ti << endl;
      cout << "t+: " << tplus << endl;
      cout << "t-: " << tminus << endl;*/

    Quaternion<double> qi = spline->value(ti);
    Quaternion<double> qiplus = spline->value(tplus);
    Quaternion<double> qiminus = spline->value(tminus);
    Quaternion<double> qstar = qi.conj();

    Quaternion<double> dq = (qiplus - qiminus) / (2 * dt);
    Quaternion<double> ddq = (qiplus + qiminus - qi * 2) / pow(dt, 2);

    Quaternion<double> omegaHat = dq * qstar * 2;
    Quaternion<double> alphaHat = (ddq * qstar - (dq * qstar).pow(2)) * 2;

    SimTK::Rotation R(SimTK::Quaternion(qi[0], qi[1], qi[2], qi[3]));

    x = Vector(R.convertRotationToBodyFixedXYZ());
    v = Vector(Vec3(omegaHat[1], omegaHat[2], omegaHat[3]));
    a = Vector(Vec3(alphaHat[1], alphaHat[2], alphaHat[3]));

    /*cout << t << endl;
      cout << "ti: " << ti << endl;
      cout << "t+: " << tplus << endl;
      cout << "t-: " << tminus << endl;

      cout << "a: " << a << endl;
      cout << qi << endl;
      cout << qiplus << endl;
      cout << qiminus << endl;*/
}

void IKOrientationTrackingTask::calcTransitions(const SimTK::Transform& T0,
                                                tree_node_<IKTaskData*>& current)
{
    IKTrackingTask::calcTransitions(T0, current);

    Quaternion<double>* orientations = new Quaternion<double>[time.size()];
    SimTK::Vector_<Vec3> positions(time.size());
    vector<Transform> transitions = current.data->getTransitions();

    calcTransformation(T0, transitions, orientations, positions);

    // for orientational tasks we only track orientations
    spline = new Spline<Quaternion<double>, double>(orientations, time.size());
}

void IKOrientationTrackingTask::calcTransformation(
    const SimTK::Transform& T0,
    const std::vector<SimTK::Transform>& transitions,
    Quaternion<double>* orientations,
    SimTK::Vector_<SimTK::Vec3>& positions)
{
    Transform Tprev = T0;

    SimTK::Quaternion qPrev = Tprev.R().convertRotationToQuaternion();
    orientations[0] = Quaternion<double>(qPrev[0], qPrev[1], qPrev[2], qPrev[3]);

    positions[0] = Tprev.p();

    // compute transformations
    for (int i = 0; i < transitions.size(); i++)
    {
        Transform T = transitions[i] * Tprev;

        SimTK::Quaternion q = T.R().convertRotationToQuaternion();
        for (int i = 0; i < 4; i++)
        {// use two different conversations to handle discontinuities
            if (std::abs(qPrev[i] - q[i]) > 0.1)
            {
                q = convertMatrixToQuaternion(T.R());
            }
        }

        positions[i + 1] = T.p();
        orientations[i + 1] = Quaternion<double>(q[0], q[1], q[2], q[3]);

        Tprev = T;
        qPrev = q;
    }
}

/* IKSpatialTrackingTask ******************************************************/

IKSpatialTrackingTask::IKSpatialTrackingTask(
    const OpenSim::Array<double>& t,
    const OpenSim::GCVSplineSet& markerSplineSet, const std::vector<int>& indicies)
    : IKOrientationTrackingTask(t, markerSplineSet, indicies)
{
}

IKSpatialTrackingTask::~IKSpatialTrackingTask()
{
}

void IKSpatialTrackingTask::calcGoal(double t, Vector& x, Vector& v, Vector& a)
{
    //should be called in order to compute orientation goal
    Super::calcGoal(t, x, v, a);

    // position goal
    int size = smoothSplines.size();
    Vec3 xd = smoothSplines[size - 1].calcValue(Vector(1, t));
    Vec3 vd = smoothSplines[size - 1].calcDerivative(firstDerivative, Vector(1, t));
    Vec3 ad = smoothSplines[size - 1].calcDerivative(secondDerivative, Vector(1, t));

    // append
    x = toVector(Vec3(x[0], x[1], x[2]), xd);
    v = toVector(Vec3(v[0], v[1], v[2]), vd);
    a = toVector(Vec3(a[0], a[1], a[2]), ad);
}

void IKSpatialTrackingTask::calcTransitions(const SimTK::Transform& T0,
                                            tree_node_<IKTaskData*>& current)
{
    IKTrackingTask::calcTransitions(T0, current);

    Quaternion<double>* orientations = new Quaternion<double>[time.size()];
    SimTK::Vector_<Vec3> positions(time.size());
    vector<Transform> transitions = current.data->getTransitions();

    calcTransformation(T0, transitions, orientations, positions);

    // for spatial task we track both orientation and positions
    spline = new Spline<Quaternion<double>, double>(orientations, time.size());

    SimTK::SplineFitter<Vec3> fitter =
	SimTK::SplineFitter<Vec3>::fitFromGCV(5, time, positions);
    SimTK::Spline_<Vec3> spline = fitter.getSpline();
    smoothSplines.push_back(spline);
}
