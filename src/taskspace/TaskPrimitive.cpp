#include "TaskPrimitive.h"

#include <OpenSim/Simulation/Model/Model.h>

#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/* TaskPrimitive *************************************************************/

TaskPrimitive::TaskPrimitive(string body, Vec3 offset, TaskTrackingController* controller)
    : AbstractPrimitive(), body(body), offset(offset), controller(controller)
{
    primitiveType = PrimitiveType::TASK;
}

TaskPrimitive::~TaskPrimitive()
{
}

TaskPrimitive::TaskType TaskPrimitive::getTaskType() const
{
    return taskType;
}

std::string OpenSim::TaskPrimitive::getFromTaskType(TaskType type) const
{
    if (type == POSITION)
    {
        return "POSITION";
    }
    else if (type == ORIENTATION)
    {
        return "ORIENTATION";
    }
    else if (type == SPATIAL)
    {
        return "SPATIAL";
    }
    else
    {
        OpenSim::Exception("Unrecognized task type", __FILE__, __LINE__);
        return "UNKNOWN";
    }
}

string TaskPrimitive::getTaskTypeAsString() const
{
    if (taskType == POSITION)
    {
        return "POSITION";
    }
    else if (taskType == ORIENTATION)
    {
        return "ORIENTATION";
    }
    else if (taskType == SPATIAL)
    {
        return "SPATIAL";
    }
    else
    {
        OpenSim::Exception("Unrecognized task type", __FILE__, __LINE__);
        return "UNKNOWN";
    }
}

string TaskPrimitive::getBody() const
{
    return body;
}

Vec3 TaskPrimitive::getOffset() const
{
    return offset;
}

void TaskPrimitive::setGoal(const State& s, const Vector& goal)
{
    AbstractPrimitive::setGoal(s, controller->calcControl(goal, x(s), v(s)));
}

Vector TaskPrimitive::tau(const State& s, const Vector& goal,
    const SimTK::Vector& tauP) const
{
    return JPT(s) * (LP(s) * (goal + b(s)) + JBarT(s) * tauP);
}

/* PositionTask **************************************************************/

PositionTask::PositionTask(string body, Vec3 offset, TaskTrackingController* controller)
    : TaskPrimitive(body, offset, controller)
{
    xddot = Vector(3, 0.0);
    taskType = TaskType::POSITION;
    setName(body + "_" + getPrimitiveTypeAsString() + "_" + getTaskTypeAsString());
}

PositionTask::~PositionTask()
{
}

Vector PositionTask::x(const State& s) const
{
    //_model->realizePosition(s);

    Vec3 temp;

    _model->getSimbodyEngine().getPosition(s,
        _model->getBodySet().get(body), offset, temp);

    return Vector(temp);
}

Vector PositionTask::v(const State& s) const
{
    //_model->realizeVelocity(s);

    Vec3 temp;

    _model->getSimbodyEngine().getVelocity(s,
        _model->getBodySet().get(body), offset, temp);

    return Vector(temp);
}

Vector PositionTask::a(const State& s) const
{
    //_model->realizeAcceleration(s);

    Vec3 temp;

    _model->getSimbodyEngine().getAcceleration(s,
        _model->getBodySet().get(body), offset, temp);

    return Vector(temp);
}

Matrix PositionTask::J(const State& s) const
{
    PRE_PROCESS_CACHE(s, CACHE_J, value, Matrix);

    calcPositionJacobian(s, *_model, body, offset, value);

    POST_PROCESS_CACHE(s, CACHE_J, value, Matrix);
}

Vector PositionTask::b(const State& s) const
{
    PRE_PROCESS_CACHE(s, CACHE_B, value, Vector);

    Vec3 jdu;
    calcPositionJDotQDot(s, *_model, body, offset, jdu);
    value = Vector(-1.0 * jdu);

    POST_PROCESS_CACHE(s, CACHE_B, value, Vector);
}

/* OrientationTask ***********************************************************/

OrientationTask::OrientationTask(string body, Vec3 offset, TaskTrackingController* controller)
    : TaskPrimitive(body, offset, controller)
{
    xddot = Vector(3, 0.0);
    taskType = TaskType::ORIENTATION;
    setName(body + "_" + getPrimitiveTypeAsString() + "_" + getTaskTypeAsString());
}

OrientationTask::~OrientationTask()
{
}

Vector OrientationTask::x(const State& s) const
{
    //_model->realizePosition(s);

    double dirCos[3][3];
    Vec3 o;

    _model->getSimbodyEngine().getDirectionCosines(s,
        _model->getBodySet().get(body), dirCos);
    _model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
        &o[0], &o[1], &o[2]);

    return Vector(o);
}

SimTK::Vector OrientationTask::v(const SimTK::State& s) const
{
    //_model->realizeVelocity(s);

    Vec3 w;

    _model->getSimbodyEngine().getAngularVelocity(s,
        _model->getBodySet().get(body), w);

    return Vector(w);
}

SimTK::Vector OrientationTask::a(const SimTK::State& s) const
{
    //_model->realizeAcceleration(s);

    Vec3 aw;

    _model->getSimbodyEngine().getAngularAcceleration(s,
        _model->getBodySet().get(body), aw);

    return Vector(aw);
}

SimTK::Matrix OrientationTask::J(const SimTK::State& s) const
{
    PRE_PROCESS_CACHE(s, CACHE_J, value, Matrix);

    calcOrientationJacobian(s, *_model, body, offset, value);

    POST_PROCESS_CACHE(s, CACHE_J, value, Matrix);
}

Vector OrientationTask::b(const State& s) const
{
    PRE_PROCESS_CACHE(s, CACHE_B, value, Vector);

    Vec3 jdu;
    calcOrientationJDotQDot(s, *_model, body, offset, jdu);
    value = Vector(-1.0 * jdu);

    POST_PROCESS_CACHE(s, CACHE_B, value, Vector);
}

/* SpatialTask ***************************************************************/

SpatialTask::SpatialTask(string body, Vec3 offset, TaskTrackingController* controller)
    : TaskPrimitive(body, offset, controller)
{
    xddot = Vector(6, 0.0);
    taskType = TaskType::SPATIAL;
    setName(body + "_" + getPrimitiveTypeAsString() + "_" + getTaskTypeAsString());
}

SpatialTask::~SpatialTask()
{
}

Vector SpatialTask::x(const State& s) const
{
    //_model->realizePosition(s);

    Vec3 p;
    double dirCos[3][3];
    Vec3 o;

    _model->getSimbodyEngine().getPosition(s,
        _model->getBodySet().get(body), offset, p);

    _model->getSimbodyEngine().getDirectionCosines(s,
        _model->getBodySet().get(body), dirCos);
    _model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
        &o[0], &o[1], &o[2]);

    return toVector(o, p);
}

SimTK::Vector SpatialTask::v(const SimTK::State& s) const
{
    //_model->realizeVelocity(s);

    Vec3 w, v;

    _model->getSimbodyEngine().getVelocity(s,
        _model->getBodySet().get(body), offset, v);

    _model->getSimbodyEngine().getAngularVelocity(s,
        _model->getBodySet().get(body), w);

    return toVector(w, v);
}

SimTK::Vector SpatialTask::a(const SimTK::State& s) const
{
    //_model->realizeAcceleration(s);

    Vec3 aw, a;

    _model->getSimbodyEngine().getAcceleration(s,
        _model->getBodySet().get(body), offset, a);

    _model->getSimbodyEngine().getAngularAcceleration(s,
        _model->getBodySet().get(body), aw);

    return toVector(aw, a);
}

SimTK::Matrix SpatialTask::J(const SimTK::State& s) const
{
    PRE_PROCESS_CACHE(s, CACHE_J, value, Matrix);

    calcFrameJacobian(s, *_model, body, offset, value);

    POST_PROCESS_CACHE(s, CACHE_J, value, Matrix);
}

Vector SpatialTask::b(const State& s) const
{
    PRE_PROCESS_CACHE(s, CACHE_B, value, Vector);

    calcFrameJDotQDot(s, *_model, body, offset, value);
    value = -1.0 * value;

    POST_PROCESS_CACHE(s, CACHE_B, value, Vector);
}