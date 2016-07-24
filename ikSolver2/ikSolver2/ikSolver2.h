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

	static MObject joint1initLength;
	static MObject joint2initLength;
	
	static MObject jnt1OutputRotateX;
	static MObject jnt1OutputRotateY;
	static MObject jnt1OutputRotateZ;
	static MObject jnt1OutputRotate;

	static MObject jnt2OutputRotateX;
	static MObject jnt2OutputRotateY;
	static MObject jnt2OutputRotateZ;
	static MObject jnt2OutputRotate;

	static MObject effectorMatrix;
	static MObject poleVectorMatrix;
	static MObject baseMatrix;

};

#endif