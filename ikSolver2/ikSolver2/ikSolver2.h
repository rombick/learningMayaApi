#ifndef ikSolver2_H
#define ikSolver2_H

#include <maya/MTypeId.h>
#include <maya/MPxNode.h>

class ikSolver2 :public MPxNode
{
public:
	virtual MStatus compute(const MPlug& plug, MDataBlock& dataBlock);
	static void* creator();
	static MStatus initialize();

public:
	static MTypeId typeId;
	static MObject inputTranslateX;
	static MObject inputTranslateY;
	static MObject inputTranslateZ;
	static MObject inputTranslate;
	static MObject outputRotateX;
	static MObject outputRotateY;
	static MObject outputRotateZ;
	static MObject outputRotate;

	static MObject driverMatrix;
	static MObject upVectorMatrix;
};

#endif