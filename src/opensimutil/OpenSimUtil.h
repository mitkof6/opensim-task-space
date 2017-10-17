#ifndef OPEN_SIM_UTIL_H
#define OPEN_SIM_UTIL_H

#include "internal/OpenSimUtilDLL.h"
#include <fstream>
#include <SimTKcommon/Simmatrix.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/MarkerData.h>

/* Macros ********************************************************************/

/**
 * Macro for caching and returning the cache result. It should be used in
 * combination with POST_PROCESS_CACHE.
 *
 * Example:
 *
 * PRE_PROCESS_CACHE(s, CACHE_J, J, Matrix);
 * J = ...
 * POST_PROCESS_CACHE(s, CACHE_J, MATRIX);
 */
#define PRE_PROCESS_CACHE(STATE, CACHE_NAME, VALUE_NAME, TYPE)          \
    if (!isCacheVariableValid(STATE, CACHE_NAME))                       \
    {                                                                   \
    TYPE& VALUE_NAME = updCacheVariableValue<TYPE>(s, CACHE_NAME);

#define POST_PROCESS_CACHE(STATE, CACHE_NAME, VALUE_NAME, TYPE) \
    markCacheVariableValid(STATE, CACHE_NAME);                  \
    return VALUE_NAME;                                          \
    }                                                           \
    return getCacheVariableValue<TYPE>(STATE, CACHE_NAME);

/**
 * Convenient method to printing the dimensions of a matrix/vector.
 */
#define MATRIX_SIZE(NAME, MATRIX)                                       \
    std::cout << #NAME << "\t[" << MATRIX.nrow() << ", " << MATRIX.ncol() << "]" \
    << std::endl;

/* I/O ***********************************************************************/

/**
 * Given a storage containing marker positions, converts to .trc file format and
 * dumps into a file. This function is intended to be used along with
 * MarkerPositionAnalysis.
 */
OpenSimUtil_API void dumpMarkersFromStorage(OpenSim::Storage& markers,
					    std::fstream& file);

/**
 * Reads an ascii array of double values.
 */
OpenSimUtil_API SimTK::Matrix readMatFile(std::string path);

/**
 * Reads a csv data, and skip the first n rows of data.
 */
OpenSimUtil_API SimTK::Matrix readCSVFile(std::string path, int skip);

/**
 * Exports body tree to graphviz dot format.
 */
OpenSimUtil_API void exportBodyGraphviz(OpenSim::Model& model, std::string path);

/**
 * Exports muscle to body connections to graphviz dot format.
 */
OpenSimUtil_API void exportMuscleGraphviz(OpenSim::Model& model, std::string path);

/**
 * Exports vector to file.
 */
OpenSimUtil_API void exportVector(SimTK::Vector vec, std::string path,
				  std::ios_base::openmode mode = std::ofstream::out);

/* Conversations *************************************************************/

/**
 * Helper function that converts a SpatialVec (Vec3, Vec3) to Vector.
 */
OpenSimUtil_API SimTK::Vector spatialToVector(const SimTK::SpatialVec& spatial);

/**
 * Helper function that converts two Vec3 to Vector.
 */
OpenSimUtil_API SimTK::Vector toVector(const SimTK::Vec3& orientation,
				       const SimTK::Vec3& position);

/**
 * Prints the vector v, but converts a subspace of v into degrees based on the
 * first and last arguments.
 */
OpenSimUtil_API SimTK::Vector toDeg(const SimTK::Vector& v, int first, int last);

/**
 * Convert explicitly a matrix (Matrix) to a 3x3 matrix (Mat33).
 */
OpenSimUtil_API SimTK::Mat33 toMat33(SimTK::Matrix T);

/**
 * A helper function that maps simulation time (double), to the nearest index
 * (int) of a vector containing the time instances.
 */
OpenSimUtil_API double timeToSpilineTime(double t, SimTK::Vector& time);

/**
 * Alternative method to convert a rotation matrix to quaternion, which avoid
 * sign flips that can cause problems when performing numerical differentiation
 * of a quaternion.
 */
OpenSimUtil_API SimTK::Quaternion convertMatrixToQuaternion(const SimTK::Rotation& R);

OpenSimUtil_API OpenSim::Output<double>* toOutput(
    const OpenSim::AbstractOutput& abstractOutput);

OpenSimUtil_API void connectComponents(const OpenSim::ModelComponent* from,
				       std::string fromOutput,
				       OpenSim::ModelComponent* to,
				       std::string toInput);

/* Calculations **************************************************************/

OpenSimUtil_API void svdInverse(const SimTK::Matrix& m, SimTK::Matrix& mInv);

OpenSimUtil_API void pseudoInverse(const SimTK::Matrix& m, SimTK::Matrix& mInv);

OpenSimUtil_API void robustInverse(const SimTK::Matrix& m, SimTK::Matrix& mInv);

OpenSimUtil_API void pseudoInverse(const SimTK::Matrix& m, bool left,
				   SimTK::Matrix& mInv);

/**
 * The condition number is the ratio of the max and min eigenvalue of
 * a matrix.
 */
OpenSimUtil_API double calcConditionNumber(const SimTK::Matrix& mat);

OpenSimUtil_API void calcPositionJacobian(const SimTK::State& s,
					  const OpenSim::Model& model,
					  const std::string& body,
					  const SimTK::Vec3& offset,
					  SimTK::Matrix& J);

OpenSimUtil_API void calcOrientationJacobian(const SimTK::State& s,
					     const OpenSim::Model& model,
					     const std::string& body,
					     const SimTK::Vec3& offset,
					     SimTK::Matrix& J);

OpenSimUtil_API void calcFrameJacobian(const SimTK::State& s,
				       const OpenSim::Model& model,
				       const std::string& body,
				       const SimTK::Vec3& offset,
				       SimTK::Matrix& J);

OpenSimUtil_API void calcGravity(const SimTK::State& s,
				 const OpenSim::Model& model, SimTK::Vector& g);

OpenSimUtil_API void calcCoriolis(const SimTK::State& s,
				  const OpenSim::Model& model, SimTK::Vector& c);

OpenSimUtil_API void calcLInv(const SimTK::State& s, const OpenSim::Model& model,
			      const SimTK::Matrix& J, const SimTK::Matrix& JT,
			      SimTK::Matrix& LInv);

OpenSimUtil_API void calcJBarT(const SimTK::State& s, const OpenSim::Model& model,
			       const SimTK::Matrix& J, const SimTK::Matrix& L,
			       SimTK::Matrix& JBarT);

/**
 * Calculates the total force and torques acting on the system, in the joint
 * space. These forces are projected into task space.
 *
 * \f$ f = \tau_{generalized} + J^T_s f_{spatial} \f$
 *
 * A copy state is used in this function, which is realized through Dynamics
 * stage. No need to disable any controller because we use the working model
 * that does not contain any controllers.
 *
 * This method makes use of the following methods:
 *
 * const Vector_<SpatialVec>& getRigidBodyForces(const State&, Stage) const;
 * const Vector&              getMobilityForces (const State&, Stage) const;
 *
 * Note that by default Coriolis forces are not computed into the total
 * contribution.
 */
OpenSimUtil_API void calcTotalGeneralizedForces(const SimTK::State& s,
						const OpenSim::Model& model,
						SimTK::Vector& f);

OpenSimUtil_API void calcPositionJDotQDot(const SimTK::State& s,
					  const OpenSim::Model& model,
					  const std::string& body,
					  const SimTK::Vec3& offset,
					  SimTK::Vec3& Jdotu);

OpenSimUtil_API void calcOrientationJDotQDot(const SimTK::State& s,
					     const OpenSim::Model& model,
					     const std::string& body,
					     const SimTK::Vec3& offset,
					     SimTK::Vec3& Jdotu);

OpenSimUtil_API void calcFrameJDotQDot(const SimTK::State& s,
				       const OpenSim::Model& model,
				       const std::string& body,
				       const SimTK::Vec3& offset,
				       SimTK::Vector& Jdotu);

OpenSimUtil_API void calcActuatorIndicies(const SimTK::State& s,
					  const OpenSim::Model& model,
					  OpenSim::Array<int>& activeActuatorIndicies,
					  OpenSim::Array<int>& disabledActuatorIndicies);

OpenSimUtil_API void calcCoordinateIndicies(const SimTK::State& s,
					    const OpenSim::Model& model,
					    OpenSim::Array<int>& activeCoordinateIndicies,
					    OpenSim::Array<int>& constrainedCoordinateIndicies);

OpenSimUtil_API void calcMaxActuatorForce(const SimTK::State& s,
					  const OpenSim::Model& model,
					  const OpenSim::Array<int>& activeActuatorIndicies,
					  SimTK::Vector& Fmax);

OpenSimUtil_API void calcMomentArm(const SimTK::State& s, const OpenSim::Model& model,
				   const OpenSim::Array<int>& coordinateIndicies,
				   const OpenSim::Array<int>& actuatorIndicies,
				   SimTK::Matrix& R);

OpenSimUtil_API SimTK::Vector calcActuatorForce(const SimTK::State& s,
						const OpenSim::Model& model,
						const OpenSim::Array<int>& muscleIndicies,
						const SimTK::Vector& activations);

/* Misc **********************************************************************/

/**
 * For a given set of 3D points calculates the centroid.
 */
OpenSimUtil_API SimTK::Vec3 calcCentroid(const std::vector<SimTK::Vec3>& A);

/**
 * Finds the transformation between A and B.

 //test normal case

 vector<Vec3> A, B;

 A.push_back(Vec3(0, 0, 0));
 A.push_back(Vec3(0, 1, 0));
 A.push_back(Vec3(0, 0, 1));

 Transform T(SimTK::Rotation(SimTK::convertDegreesToRadians(33), SimTK::ZAxis));
 T.setP(Vec3(1.3, 1.05, 0));
 B.push_back(T * A[0]);
 B.push_back(T * A[1]);
 B.push_back(T * A[2]);

 cout << findTransformation(A, B) << endl;

 R =

 0.8387   -0.5446         0
 0.5446    0.8387         0
 0         0              1.0000

 T =

 1.3000
 1.0500
 0

 // test reflexion case
 vector<Vec3> A, B;

 A.push_back(Vec3(-30.3200, 0, -7.5000));
 A.push_back(Vec3(20.7500, 0, -7.5000));
 A.push_back(Vec3(0, -27.0300, -7.5000));
 A.push_back(Vec3(0, 33.4500, -7.5000));

 B.push_back(Vec3(-33.4243, 47.8792, 10.8745));
 B.push_back(Vec3(17.2760, 42.1348, 11.6758));
 B.push_back(Vec3(-3.3237, 32.9578, -12.4699));
 B.push_back(Vec3(1.7730, 62.1567, 40.0464));

 cout << findTransformation(A, B) << endl;

 ans =

 0.9927    0.0681   -0.0995   -2.9048
 -0.1201    0.4889   -0.8641   38.7298
 -0.0102    0.8697    0.4935   14.8126
 0         0         0    1.0000
*/
OpenSimUtil_API SimTK::Transform findTransformation(const std::vector<SimTK::Vec3>& A,
						    const std::vector<SimTK::Vec3>& B);

/**
 * Given a set of marker data, initializes the state so that the model pose is
 * the same as the pose defined by the marker data at time s.getTime() (which
 * can be defied externally).
 */
OpenSimUtil_API void calcInitialState(SimTK::State& s, const
				      OpenSim::Model& model,
				      const OpenSim::MarkerData& markerData);

/**
 * Disables/enables muscles TODO deprecate
 */
OpenSimUtil_API void setUseMuscles(SimTK::State& s,
				   const OpenSim::Model& model, bool useMuscles);

/**
 * Should be called before the muscle is built.
 */
OpenSimUtil_API void setIgnoreActivationDynamics(const OpenSim::Model& model,
						 bool ignore);

/**
 * Euclidean distance between two 3D points.
 */
OpenSimUtil_API double distance(SimTK::Vec3 a, SimTK::Vec3 b);

/**
 * Squared Euclidean distance between two 3D points.
 */
OpenSimUtil_API double distanceSq(SimTK::Vec3 a, SimTK::Vec3 b);

OpenSimUtil_API void simulate(SimTK::State& s, OpenSim::Model& model,
			      SimTK::Integrator& integrator, double t0,
			      double tf, OpenSim::Storage& storage);

/**
 * Adds reserve actuators to the model, and checks if reserves are defined.
 * Moreover, returns the vectors of names and indicies of these actuators for
 * fast indexing in the model.
 */
OpenSimUtil_API void addReserveActuators(OpenSim::Model& model,
					 std::vector<std::string>& reserveActuatorNames,
					 std::vector<int>& reserveActuatorIndecies);

#endif
