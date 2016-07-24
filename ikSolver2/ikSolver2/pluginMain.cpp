#include "ikSolver2.h"
#include <maya/MFnPlugin.h>

MStatus initializePlugin(MObject obj)
{
	MStatus stat;

	MFnPlugin fnPlugin(obj, "rr", "1.0", "Any");
	stat = fnPlugin.registerNode("ikSolver2",
		ikSolver2::typeId,
		&ikSolver2::creator,
		&ikSolver2::initialize);

	if (stat != MS::kSuccess)
		stat.perror("could not register the ikSolver2 node");

	return stat;
}

MStatus uninitializePlugin(MObject object)
{
	MFnPlugin pluginFn;
	pluginFn.deregisterNode(ikSolver2::typeId);

	return MS::kSuccess;

}